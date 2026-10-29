#pragma once
// Opt-in diagnostics. Per-producer SPSC queues keep disk I/O and consumer locks
// out of the control threads. Callers must stop producers before destruction.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>
namespace startup_audit {
inline double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
struct Row {
 std::ostringstream out;
 explicit Row(const char* kind){out<<std::setprecision(17)<<"{\"schema\":1,\"time\":"<<now()<<",\"kind\":"<<std::quoted(kind);}
 template<class T> Row& scalar(const char* key,T v){out<<",\""<<key<<"\":"<<v;return *this;}
 Row& text(const char* key,const std::string& v){out<<",\""<<key<<"\":"<<std::quoted(v);return *this;}
 template<class T> Row& array(const char* key,const T& v){out<<",\""<<key<<"\":[";bool first=true;for(auto x:v){if(!first)out<<',';first=false;out<<x;}out<<']';return *this;}
 std::string finish(){return out.str()+"}";}
};
class Writer {
 struct Producer {
  std::vector<std::string> slots;
  std::atomic<size_t> head{0},tail{0};
  explicit Producer(size_t capacity):slots(capacity+1){}
  bool push(std::string&& line){
   const auto h=head.load(std::memory_order_relaxed),next=(h+1)%slots.size();
   if(next==tail.load(std::memory_order_acquire))return false;
   slots[h]=std::move(line);head.store(next,std::memory_order_release);return true;
  }
  bool pop(std::string& line){
   const auto t=tail.load(std::memory_order_relaxed);
   if(t==head.load(std::memory_order_acquire))return false;
   line=std::move(slots[t]);tail.store((t+1)%slots.size(),std::memory_order_release);return true;
  }
 };
 std::ofstream file;
 std::mutex registry_mu,wake_mu;
 std::condition_variable cv;
 std::vector<std::shared_ptr<Producer>> producers;
 std::atomic<bool> done{false};
 std::thread thread;
 std::atomic<unsigned long> dropped{0},accepted{0};
 unsigned long written=0,write_errors=0;
 const size_t capacity;
 std::shared_ptr<Producer> producer(){
  // Weak ownership prevents reuse of a queue belonging to a destroyed Writer.
  thread_local Writer* owner=nullptr;
  thread_local std::weak_ptr<Producer> cached;
  auto p=owner==this?cached.lock():nullptr;
  if(!p){p=std::make_shared<Producer>(capacity);{std::lock_guard<std::mutex> l(registry_mu);producers.push_back(p);}owner=this;cached=p;}
  return p;
 }
 void consume(){
  for(;;){
   std::vector<std::shared_ptr<Producer>> snapshot;
   {std::lock_guard<std::mutex> l(registry_mu);snapshot=producers;}
   bool any=false;
   for(auto& p:snapshot){
    std::string line;
    // Bounded round-robin draining avoids starving other producer streams.
    for(size_t i=0;i<capacity && p->pop(line);++i){any=true;file<<line<<'\n';if(file)++written;else ++write_errors;}
   }
   if(done.load(std::memory_order_acquire) && !any)break;
   if(!any){std::unique_lock<std::mutex> l(wake_mu);cv.wait_for(l,std::chrono::milliseconds(1));}
  }
  file<<"{\"kind\":\"audit_end\",\"dropped\":"<<dropped.load()
      <<",\"accepted\":"<<accepted.load()<<",\"written\":"<<written
      <<",\"write_errors\":"<<write_errors<<"}\n";
  file.flush();
 }
 public:
 explicit Writer(size_t queue_capacity=1024):capacity(queue_capacity){
  const char* path=std::getenv("SONIC_STARTUP_AUDIT");if(!path||!*path)return;
  if(!capacity)throw std::invalid_argument("Audit queue capacity must be positive");
  file.open(path,std::ios::out|std::ios::app);
  if(!file)throw std::runtime_error("Cannot open requested startup audit file");
  thread=std::thread([this]{consume();});
 }
 bool enabled()const{return thread.joinable();}
 void send(Row& r){
  if(!enabled())return;
  auto p=producer();auto line=r.finish();
  if(!p->push(std::move(line))){++dropped;return;}
  ++accepted;cv.notify_one();
 }
 ~Writer(){if(!enabled())return;done.store(true,std::memory_order_release);cv.notify_one();thread.join();}
};
inline Writer& writer(){static Writer w;return w;}
}
