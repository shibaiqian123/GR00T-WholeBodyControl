#pragma once
// Optional localhost telemetry. No robot protocol fields, disk writes, or waits.
#include <atomic>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <string>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace sonic_latency {
inline uint64_t mono_ns() noexcept {
  timespec ts{}; clock_gettime(CLOCK_MONOTONIC, &ts);
  return uint64_t(ts.tv_sec)*1000000000ULL+uint64_t(ts.tv_nsec);
}
inline uint64_t wall_ns() noexcept {
  timespec ts{}; clock_gettime(CLOCK_REALTIME, &ts);
  return uint64_t(ts.tv_sec)*1000000000ULL+uint64_t(ts.tv_nsec);
}
class Tracer {
  int fd_=-1;
  sockaddr_in address_{};
  char host_[80]{}, boot_[80]{}, identity_[160]{};
  unsigned every_=10;
  std::atomic<uint64_t> events_{0}, cycles_{0}, writes_{0}, dropped_{0};
  static bool safe(const char* s) noexcept {
    if(!s || !*s) return false;
    for(;*s;++s) if(!std::isalnum(static_cast<unsigned char>(*s)) && *s!='_' && *s!='-' && *s!='.') return false;
    return true;
  }
 public:
  Tracer() noexcept {
    try {
      const char* endpoint=std::getenv("LATENCY_TRACE_UDP");
      if(!endpoint || std::strncmp(endpoint,"127.0.0.1:",10)!=0) return;
      char* end=nullptr; long port=std::strtol(endpoint+10,&end,10);
      if(!end || *end || port<1 || port>65535) return;
      const char* host=std::getenv("LATENCY_TRACE_HOST");
      if(!safe(host) || std::strlen(host)>=sizeof(host_)) return;
      std::snprintf(host_,sizeof(host_),"%s",host);
      std::string boot;
      std::ifstream file("/proc/sys/kernel/random/boot_id"); std::getline(file,boot);
      if(!safe(boot.c_str()) || boot.size()>=sizeof(boot_)) return;
      std::snprintf(boot_,sizeof(boot_),"%s",boot.c_str());
      std::snprintf(identity_,sizeof(identity_),"%s:%ld",boot_,static_cast<long>(getpid()));
      const char* rate=std::getenv("LATENCY_TRACE_EVERY");
      if(rate) {
        long n=std::strtol(rate,&end,10);
        if(!end || *end || n<1 || n>10000) return;
        every_=static_cast<unsigned>(n);
      }
      address_.sin_family=AF_INET;
      address_.sin_port=htons(static_cast<uint16_t>(port));
      inet_pton(AF_INET,"127.0.0.1",&address_.sin_addr);
      fd_=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    } catch(...) { if(fd_>=0)close(fd_);fd_=-1; }
  }
  ~Tracer(){if(fd_>=0)close(fd_);}
  bool enabled() const noexcept { return fd_>=0; }
  uint64_t cycle() noexcept {
    if(!enabled()) return 0;
    auto n=cycles_.fetch_add(1,std::memory_order_relaxed)+1;
    return n%every_==0?n:0;
  }
  uint64_t write() noexcept {return enabled()?writes_.fetch_add(1,std::memory_order_relaxed)+1:0;}
  void emit(const char* category,uint64_t id,const char* stage,uint64_t parent=0,uint64_t at=0) noexcept {
    if(!enabled() || id==0)return;
    char trace_id[240];
    std::snprintf(trace_id,sizeof(trace_id),"sonic:%s:%s:%llu",identity_,category,(unsigned long long)id);
    emit_id(trace_id,stage,parent,at);
  }
  void token(const char* session,long long frame) noexcept {
    if(!enabled() || frame<0)return;
    char trace_id[100];
    std::snprintf(trace_id,sizeof(trace_id),"%s:frame:%lld",session,frame);
    emit_id(trace_id,"sonic_token_decoded",0,0);
  }
 private:
  void emit_id(const char* trace_id,const char* stage,uint64_t parent,uint64_t at) noexcept {
    const auto now=mono_ns(), wall=wall_ns();
    if(at==0)at=now;
    if(at>now)return;
    const auto event=events_.fetch_add(1,std::memory_order_relaxed);
    char line[1200];
    int n=std::snprintf(line,sizeof(line),
      "{\"v\":1,\"event_id\":\"%s:%llu\",\"trace_id\":\"%s\","
      "\"stage\":\"%s\",\"clock\":{\"host\":\"%s\",\"boot\":\"%s\",\"mono_ns\":%llu,\"wall_ns\":%llu},"
      "\"meta\":{\"decoder_cycle\":%llu,\"sample_every\":%u,\"motor_execution_confirmed\":false},\"emitter_drops\":%llu}",
      identity_,(unsigned long long)event,trace_id,
      stage,host_,boot_,(unsigned long long)at,(unsigned long long)(wall-(now-at)),
      (unsigned long long)parent,every_,(unsigned long long)dropped_.load(std::memory_order_relaxed));
    if(n<=0 || n>=static_cast<int>(sizeof(line)) || sendto(fd_,line,n,MSG_DONTWAIT,
       reinterpret_cast<const sockaddr*>(&address_),sizeof(address_))!=n)
      dropped_.fetch_add(1,std::memory_order_relaxed);
  }
};
// Construct before main, so getenv/socket/file reads never happen on control ticks.
inline Tracer tracer;
}
