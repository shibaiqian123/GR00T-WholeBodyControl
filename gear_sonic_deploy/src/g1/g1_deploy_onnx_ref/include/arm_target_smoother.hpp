#pragma once
#include "joint_target_limiter.hpp"
#include <cmath>

// Explicitly configured shoulder/elbow target shaping. No token processing or motor I/O.
class ArmTargetSmoother {
public:
 using Vec=JointTargetLimiter::Vec;
 static constexpr std::array<int,8> joints={15,16,17,18,22,23,24,25};
 Vec lo,hi,start{},previous{};
 double blend_time,tau,velocity,elapsed=0,last_time=0,alpha=0,weight=0,dt=0;
 bool active=false,last_encoder=true,reseeded=false;
 explicit ArmTargetSmoother(const nlohmann::json& c) {
   if(!c.at("simulation_only").is_boolean() || c.at("units")!="rad" || c.at("joint_order")!="hardware_29")throw std::runtime_error("arm smoother requires explicit simulation_only boolean, rad hardware_29 config");
   const std::array<std::string,8> names={"left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint","left_elbow_joint","right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint","right_elbow_joint"};
   if(c.at("joint_names").size()!=29)throw std::runtime_error("arm smoother requires 29 joint names");
   for(size_t k=0;k<joints.size();++k)if(c.at("joint_names").at(joints[k])!=names[k])throw std::runtime_error("arm smoother joint mapping mismatch");
   lo=JointTargetLimiter::read(c.at("q_min"));hi=JointTargetLimiter::read(c.at("q_max"));
   blend_time=c.at("T_blend");tau=c.at("tau");velocity=c.at("v_max");
   for(double x:{blend_time,tau,velocity})if(!JointTargetLimiter::finite(x)||x<=0)throw std::runtime_error("positive finite smoothing parameters required");
   for(int i:joints)if(lo[i]>=hi[i])throw std::runtime_error("invalid shoulder/elbow bounds");
 }
 void reset(){active=false;last_encoder=true;elapsed=0;}
 Vec apply(const Vec& raw,const Vec& sent,bool using_encoder,double now,double nominal_dt){
   if(!JointTargetLimiter::finite(now)||!JointTargetLimiter::finite(nominal_dt)||nominal_dt<=0)throw std::runtime_error("invalid smoothing time");
   for(double x:raw)if(!JointTargetLimiter::finite(x))throw std::runtime_error("nonfinite decoder target");
   reseeded=false;
   if(!active && using_encoder)return raw; // Do not modify initial PLANNER control.
   bool begin=!active || using_encoder!=last_encoder;
   for(int i:joints){
     if(raw[i]<lo[i]||raw[i]>hi[i])throw std::runtime_error("raw shoulder/elbow target outside limits");
     if(begin && (!JointTargetLimiter::finite(sent[i])||sent[i]<lo[i]||sent[i]>hi[i]))throw std::runtime_error("invalid last published smoothing seed");
   }
   double step=begin?nominal_dt:std::min(nominal_dt,now-last_time);
   if(!JointTargetLimiter::finite(step)||step<=0)throw std::runtime_error("nonadvancing smoothing time");
   // Mode changes reseed from successful DDS publication, never zero/current raw.
   if(begin){start=sent;previous=sent;elapsed=0;reseeded=true;}
   else elapsed+=step; // No catch-up jump on a delayed cycle.
   dt=step;double u=std::clamp(elapsed/blend_time,0.0,1.0);
   weight=3*u*u-2*u*u*u;alpha=dt/(tau+dt);
   Vec out=raw;
   for(int i:joints){
     double blended=(1-weight)*start[i]+weight*raw[i];
     double delta=std::clamp(alpha*(blended-previous[i]),-velocity*dt,velocity*dt);
     out[i]=previous[i]+delta;
   }
   previous=out;last_time=now;last_encoder=using_encoder;active=true;return out;
 }
};
