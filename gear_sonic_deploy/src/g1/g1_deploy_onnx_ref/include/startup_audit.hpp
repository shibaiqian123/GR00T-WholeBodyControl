#pragma once
// Opt-in diagnostics only. No DDS imports or control decisions.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
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
 std::ofstream file;std::mutex mu;std::condition_variable cv;std::deque<std::string> queue;bool done=false;std::thread thread;std::atomic<unsigned long> dropped{0};
 public:
 Writer(){const char* path=std::getenv("SONIC_STARTUP_AUDIT");if(!path||!*path)return;file.open(path,std::ios::out|std::ios::app);if(!file)return;thread=std::thread([this]{for(;;){std::unique_lock<std::mutex> l(mu);cv.wait(l,[this]{return done||!queue.empty();});if(queue.empty()&&done)break;auto line=std::move(queue.front());queue.pop_front();l.unlock();file<<line<<'\n';}file<<"{\"kind\":\"audit_end\",\"dropped\":"<<dropped.load()<<"}\n";file.flush();});}
 bool enabled()const{return thread.joinable();}
 void send(Row& r){if(!enabled())return;auto line=r.finish();std::unique_lock<std::mutex> l(mu,std::try_to_lock);if(!l.owns_lock()||queue.size()>=1024){++dropped;return;}queue.push_back(std::move(line));cv.notify_one();}
 ~Writer(){if(!enabled())return;{std::lock_guard<std::mutex> l(mu);done=true;}cv.notify_one();thread.join();}
};
inline Writer& writer(){static Writer w;return w;}
}
