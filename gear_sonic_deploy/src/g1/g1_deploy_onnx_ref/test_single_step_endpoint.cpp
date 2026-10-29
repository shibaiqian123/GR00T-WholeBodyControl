#include "single_step_endpoint.hpp"
#include <cassert>
#include <cstdio>
#include <unistd.h>
#include <iostream>
int main(){
 using E=SingleStepEndpoint;using J=E::Json;
 // Synthetic protocol fixture only. These are NOT physical robot limits.
 J c={{"port",15558},{"followup","planner"},{"units","rad"},{"joint_order","hardware_29"},{"constraint_source","synthetic test only"},{"q_min",std::vector<double>(29,-10)},{"q_max",std::vector<double>(29,10)},{"max_delta",std::vector<double>(29,1)}};
 std::string path="/tmp/single-step-test-"+std::to_string(getpid())+".json";{std::ofstream f(path);f<<c;}
 unsetenv("CYCLONEDDS_URI");
 J simulation=c;simulation["simulation_only"]=true;
 E::validate_environment(simulation,38,"lo");
 for(auto domain:{0,1,37}){bool rejected=false;try{E::validate_environment(simulation,domain,"lo");}catch(...){rejected=true;}assert(rejected);}
 {bool rejected=false;try{E::validate_environment(simulation,38,"eth0");}catch(...){rejected=true;}assert(rejected);}
 setenv("CYCLONEDDS_URI","override",1);
 {bool rejected=false;try{E::validate_environment(simulation,38,"lo");}catch(...){rejected=true;}assert(rejected);}
 unsetenv("CYCLONEDDS_URI");
 E e(path);std::remove(path.c_str());
 zmq::context_t ctx;zmq::socket_t client(ctx,zmq::socket_type::req);client.set(zmq::sockopt::rcvtimeo,1000);client.set(zmq::sockopt::linger,0);client.connect("tcp://127.0.0.1:15558");
 client.send(zmq::buffer(std::string("{\"op\":\"status\"}")),zmq::send_flags::none);zmq::message_t reply;assert(client.recv(reply));assert(J::parse(reply.to_string())["ready"]==false);
 J req={{"op","execute_once"},{"session",e.session},{"request_id","test"},{"token",std::vector<double>(64,0)},{"execute",true}};
 bool bad=false;try{e.handle(req);}catch(...){bad=true;}assert(bad); // Not running PLANNER.
 {std::lock_guard<std::mutex> g(e.mutex);e.update_ready(true);}
 auto invalid=req;invalid["token"][3]=nullptr;bad=false;try{e.handle(invalid);}catch(...){bad=true;}assert(bad);
 auto receipt=e.handle(req);assert(receipt["phase"]=="accepted");
 assert(e.handle(req)==receipt); // Lost reply replay is idempotent.
 J other=req;other["token"][0]=.1;bad=false;try{e.handle(other);}catch(...){bad=true;}assert(bad);
 other=req;other["request_id"]="second";bad=false;try{e.handle(other);}catch(...){bad=true;}assert(bad);
 other=req;other["session"]="old-session";bad=false;try{e.handle(other);}catch(...){bad=true;}assert(bad);
 std::array<double,29> q{},actual{};assert(e.within_limits(q,actual));q[3]=1.01;assert(!e.within_limits(q,actual));
 // Run the same sender gate used in LowCommandWriter at a faster cadence.
 {std::lock_guard<std::mutex> g(e.mutex);
  e.phase="token_pending";int writes=0;
  for(int tick=0;tick<10;++tick)if(e.write_allowed()){++writes;e.token_write_result(true);}
  assert(writes==1);assert(e.receipt["dds_write_attempts"]==1);
  e.phase="planner_pending";assert(e.write_allowed());e.planner_write_result(true);
  assert(e.receipt["followup_confirmed"]==true);assert(e.receipt["reached_target"]==false);
  e.cancel();
 }
 receipt=e.handle(req);assert(receipt["phase"]=="complete");assert(receipt["dds_write_attempts"]==1);
 assert(!e.handle(J{{"op","status"}})["ready"].get<bool>());
 // Exercise a failed write and stop before sending using the production transition methods.
 {std::lock_guard<std::mutex> g(e.mutex);
  e.phase="token_pending";e.receipt["dds_write_attempts"]=0;e.token_write_result(false);
  assert(e.stopped);assert(e.receipt["dds_write_attempts"]==1);assert(!e.receipt["dds_write_succeeded"].get<bool>());
  e.phase="accepted";e.receipt["dds_write_attempts"]=0;e.cancel();assert(e.phase=="failed");assert(e.receipt["dds_write_attempts"]==0);
 }
 std::cout<<"single-step protocol, transport, envelope, deduplication, sender gate and stop tests passed\n";
}
