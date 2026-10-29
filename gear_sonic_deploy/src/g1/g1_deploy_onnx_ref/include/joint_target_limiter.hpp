#pragma once
#include <array>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <fstream>
#include <nlohmann/json.hpp>

// Position-command slew limiter, NOT a balance controller or physical velocity guarantee.
// All vectors are 29 joints in the existing hardware order, units rad / rad/s.
class JointTargetLimiter {
public:
 using Vec=std::array<double,29>;
 Vec lo,hi,velocity,tracking,previous{};
 bool initialized=false;
 double last_time=0;
 static bool finite(double x) { // Robust even in this project's -ffast-math build.
   uint64_t bits;std::memcpy(&bits,&x,sizeof(bits));
   return (bits & 0x7ff0000000000000ULL)!=0x7ff0000000000000ULL;
 }
 static Vec read(const nlohmann::json& j) {
   if(!j.is_array()||j.size()!=29)throw std::runtime_error("limiter requires 29 values");
   Vec v{};for(int i=0;i<29;++i){v[i]=j.at(i).get<double>();if(!finite(v[i]))throw std::runtime_error("nonfinite limiter config");}return v;
 }
 explicit JointTargetLimiter(const nlohmann::json& c) {
   if(c.at("units")!="rad"||c.at("joint_order")!="hardware_29"||c.at("constraint_source").get<std::string>().empty())throw std::runtime_error("limiter units/order/source required");
   lo=read(c.at("q_min"));hi=read(c.at("q_max"));velocity=read(c.at("max_velocity"));tracking=read(c.at("max_tracking_error"));
   for(int i=0;i<29;++i)if(lo[i]>=hi[i]||velocity[i]<=0||tracking[i]<=0)throw std::runtime_error("invalid limiter bounds");
 }
 void reset(){initialized=false;}
 Vec apply(const Vec& raw,const Vec& actual,const Vec& seed,double now,double nominal_dt) {
   if(!finite(now)||!finite(nominal_dt)||nominal_dt<=0)throw std::runtime_error("invalid limiter clock");
   double dt=initialized?std::min(nominal_dt,now-last_time):nominal_dt;
   if(dt<=0||!finite(dt))throw std::runtime_error("nonadvancing limiter clock");
   Vec next{};const auto& prior=initialized?previous:seed;
   for(int i=0;i<29;++i){
     if(!finite(raw[i])||!finite(actual[i])||!finite(prior[i])||raw[i]<lo[i]||raw[i]>hi[i]||actual[i]<lo[i]||actual[i]>hi[i]||prior[i]<lo[i]||prior[i]>hi[i])throw std::runtime_error("limiter nonfinite or position outside bounds");
     if(std::abs(prior[i]-actual[i])>tracking[i])throw std::runtime_error("limiter prior target tracking error");
     next[i]=prior[i]+std::clamp(raw[i]-prior[i],-velocity[i]*dt,velocity[i]*dt);
     if(std::abs(next[i]-actual[i])>tracking[i])throw std::runtime_error("limiter target tracking error");
   }
   previous=next;last_time=now;initialized=true;return next;
 }
};
