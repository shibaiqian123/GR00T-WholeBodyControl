#pragma once
#include <nlohmann/json.hpp>
#include "startup_audit.hpp"
#include <zmq.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <random>
#include <thread>
#include <unordered_map>

// Optional loopback request endpoint. The owner holds mutex during target-buffer
// changes and DDS writes. No socket thread can write motors or change modes.
class SingleStepEndpoint {
public:
 using Json=nlohmann::json;
 std::mutex mutex;
 std::string phase="idle", active, session;
 bool ready=false, stopped=false;
 std::array<double,64> token{};
 std::array<double,29> q_min{},q_max{},max_delta{};
 Json receipt;
 double ready_time=0;
 static double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
 static void validate_environment(const Json& c,int domain,const std::string& interface){
  if(c.value("simulation_only",false) && (domain!=38 || interface!="lo" || std::getenv("CYCLONEDDS_URI")))
   throw std::runtime_error("simulation-only single-step requires DDS domain 38, lo, and no CYCLONEDDS_URI override");
 }
 explicit SingleStepEndpoint(const std::string& config){
  std::ifstream f(config);Json c;f>>c;
  if(c.at("followup")!="planner"||c.at("units")!="rad"||c.at("joint_order")!="hardware_29")throw std::runtime_error("single-step config must specify planner, rad, hardware_29");
  q_min=finite_array<29>(c.at("q_min"));q_max=finite_array<29>(c.at("q_max"));max_delta=finite_array<29>(c.at("max_delta"));
  if(c.value("constraint_source",std::string{}).empty())throw std::runtime_error("single-step constraint_source required");
  for(int i=0;i<29;++i)if(q_min[i]>=q_max[i]||max_delta[i]<=0)throw std::runtime_error("invalid single-step limits");
  std::random_device rd;session=std::to_string(rd())+"-"+std::to_string(rd());
  const int port=c.at("port").get<int>();
  if(port!=15558)throw std::runtime_error("single-step endpoint requires isolated loopback port 15558");
  // Bind before spawning: occupied port is a startup error, never kill other services.
  socket_.set(zmq::sockopt::linger,0);socket_.set(zmq::sockopt::rcvtimeo,100);
  socket_.set(zmq::sockopt::sndtimeo,100);socket_.set(zmq::sockopt::maxmsgsize,int64_t(16384));
  socket_.bind("tcp://127.0.0.1:15558");
  worker_=std::thread([this]{serve();});
 }
 ~SingleStepEndpoint(){done_=true;if(worker_.joinable())worker_.join();}
 template<size_t N>static std::array<double,N> finite_array(const Json& j){
  if(!j.is_array()||j.size()!=N)throw std::runtime_error("wrong array shape");
  std::array<double,N> out{};for(size_t i=0;i<N;++i){if(!j[i].is_number())throw std::runtime_error("not numeric");out[i]=j[i].get<double>();if(!std::isfinite(out[i]))throw std::runtime_error("nonfinite value");}return out;
 }
 void update_ready(bool value){ready=value;ready_time=now();}
 void save(){if(!active.empty()){
  records_[active]=receipt;
  if(startup_audit::writer().enabled()){
   startup_audit::Row row("single_step_phase");row.text("request_id",active).text("session",session).text("phase",receipt.value("phase",std::string{})).text("error",receipt.value("error",std::string{}))
    .scalar("decode_count",receipt.value("decode_count",0)).scalar("dds_write_attempts",receipt.value("dds_write_attempts",0))
    .scalar("dds_write_succeeded",receipt.value("dds_write_succeeded",false)).scalar("followup_confirmed",receipt.value("followup_confirmed",false));
   startup_audit::writer().send(row);
  }
 }}
 void fail(const std::string& why){if(!active.empty()&&phase!="complete"&&phase!="failed"){receipt["phase"]="failed";receipt["error"]=why;receipt["finished_at"]=now();save();}phase="failed";stopped=true;ready=false;}
 void cancel(){fail("stop latched");}
 bool within_limits(const std::array<double,29>& q,const std::array<double,29>& actual){
  for(int i=0;i<29;++i)if(!std::isfinite(q[i])||!std::isfinite(actual[i])||q[i]<q_min[i]||q[i]>q_max[i]||std::abs(q[i]-actual[i])>max_delta[i])return false;
  return true;
 }
 bool write_allowed()const{return phase!="token_written";}
 void token_write_result(bool ok){
  if(phase!="token_pending")throw std::logic_error("unexpected one-shot write");
  receipt["dds_write_attempts"]=1;receipt["dds_write_succeeded"]=ok;receipt["dds_write_time"]=now();
  if(ok){phase="token_written";receipt["phase"]=phase;save();}else fail("one-shot DDS write failed; no retry");
 }
 void planner_write_result(bool ok){
  if(phase!="planner_pending")throw std::logic_error("unexpected planner return write");
  receipt["planner_dds_write_succeeded"]=ok;
  if(ok){phase="complete";receipt["phase"]=phase;receipt["followup_confirmed"]=true;receipt["planner_write_time"]=now();save();}
  else fail("planner return DDS write failed");
 }
 Json handle(const Json& req){
  std::lock_guard<std::mutex> lock(mutex);
  if(req.at("op")=="status"){
   if(req.contains("request_id"))return records_.at(req.at("request_id").get<std::string>());
   return Json{{"ok",true},{"session",session},{"ready",ready&&!stopped&&now()-ready_time<.25},{"phase",phase},{"followup","planner"}};
  }
  if(req.at("op")!="execute_once"||req.at("session")!=session||req.at("execute")!=true)throw std::runtime_error("explicit execute_once with current session required");
  const auto id=req.at("request_id").get<std::string>();
  if(id.empty()||id.size()>128)throw std::runtime_error("invalid request ID");
  auto t=finite_array<64>(req.at("token"));
  for(auto v:t)if(!std::isfinite(static_cast<float>(v)))throw std::runtime_error("token overflows float32");
  auto old=records_.find(id);
  if(old!=records_.end()){
   if(old->second.at("token")!=Json(t))throw std::runtime_error("request ID payload mismatch");
   return old->second;
  }
  if(stopped||!ready||now()-ready_time>=.25)throw std::runtime_error("fresh active PLANNER required; no automatic startup");
  if(phase!="idle"&&phase!="complete")throw std::runtime_error("single-step busy");
  if(records_.size()>=256)throw std::runtime_error("deduplication ledger full; restart requires new session");
  token=t;active=id;phase="accepted";
  receipt=Json{{"ok",true},{"request_id",id},{"session",session},{"phase",phase},{"token",t},{"accepted_at",now()},{"decode_count",0},{"dds_write_attempts",0},{"dds_write_succeeded",false},{"reached_target",false},{"followup","planner"},{"followup_confirmed",false}};
  save();return receipt;
 }
private:
 zmq::context_t context_{1};zmq::socket_t socket_{context_,zmq::socket_type::rep};
 std::atomic<bool> done_{false};std::thread worker_;
 std::unordered_map<std::string,Json> records_;
 void serve(){while(!done_){try{
  zmq::message_t msg;if(!socket_.recv(msg,zmq::recv_flags::none))continue;
  Json reply;try{reply=handle(Json::parse(msg.to_string()));}catch(const std::exception& e){reply=Json{{"ok",false},{"error",e.what()}};}
  socket_.send(zmq::buffer(reply.dump()),zmq::send_flags::none);
 }catch(const std::exception&){std::lock_guard<std::mutex> lock(mutex);fail("endpoint transport error");done_=true;}}}
};
