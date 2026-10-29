#pragma once
#include "joint_target_limiter.hpp"
#include <cmath>

// Full-body common takeover weight, followed by independent optional filtering.
// All joints here are bounded revolute joints: never wrap angles or interpolate tokens.
class JointTargetSmoother {
public:
 using Vec=JointTargetLimiter::Vec;
 Vec lo,hi,tau,velocity{},tracking,start{},alpha{};
 std::array<bool,29> rate_limited{};
 double blend_time,elapsed=0,last_time=0,weight=0,dt=0;
 bool active=false,last_encoder=true,reseeded=false;
 static std::array<std::string,29> names(){return {
 "left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint","left_knee_joint","left_ankle_pitch_joint","left_ankle_roll_joint",
 "right_hip_pitch_joint","right_hip_roll_joint","right_hip_yaw_joint","right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint",
 "waist_yaw_joint","waist_roll_joint","waist_pitch_joint",
 "left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint","left_elbow_joint","left_wrist_roll_joint","left_wrist_pitch_joint","left_wrist_yaw_joint",
 "right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint","right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"};}
 explicit JointTargetSmoother(const nlohmann::json& c){
   if(c.value("schema_version",0)!=2)throw std::runtime_error("smoothing requires schema_version=2: full-body takeover, per-joint parameters and tracking gate");
   if(!c.at("simulation_only").is_boolean()||c.at("units")!="rad"||c.at("joint_order")!="hardware_29")throw std::runtime_error("invalid smoothing units/order/deployment");
   auto expected=names();if(c.at("joint_names").size()!=29)throw std::runtime_error("29 joint names required");
   for(int i=0;i<29;++i)if(c.at("joint_names").at(i)!=expected[i])throw std::runtime_error("smoothing joint mapping mismatch");
   if(!c.contains("tracking_error_source")||!c["tracking_error_source"].is_string()||c["tracking_error_source"].get<std::string>().empty())throw std::runtime_error("verified tracking_error_source required; draft config cannot run");
   lo=JointTargetLimiter::read(c.at("q_min"));hi=JointTargetLimiter::read(c.at("q_max"));tau=JointTargetLimiter::read(c.at("tau"));
   tracking=JointTargetLimiter::read(c.at("max_start_tracking_error"));
   blend_time=c.at("T_blend");if(!JointTargetLimiter::finite(blend_time)||blend_time<=0)throw std::runtime_error("invalid blend time");
   if(!c.at("v_max").is_array()||c["v_max"].size()!=29)throw std::runtime_error("29 v_max entries required; null means disabled");
   for(int i=0;i<29;++i){
     if(lo[i]>=hi[i]||tau[i]<0||tracking[i]<=0)throw std::runtime_error("invalid per-joint limits");
     rate_limited[i]=!c["v_max"][i].is_null();
     if(rate_limited[i]){velocity[i]=c["v_max"][i].get<double>();if(!JointTargetLimiter::finite(velocity[i])||velocity[i]<=0)throw std::runtime_error("invalid per-joint rate limit");}
   }
 }
 void reset(){active=false;last_encoder=true;elapsed=0;reseeded=false;}
 Vec apply(const Vec& raw,const Vec& sent,const Vec& measured,bool using_encoder,double now,double nominal_dt){
   if(!JointTargetLimiter::finite(now)||!JointTargetLimiter::finite(nominal_dt)||nominal_dt<=0)throw std::runtime_error("invalid smoothing time");
   for(double x:raw)if(!JointTargetLimiter::finite(x))throw std::runtime_error("nonfinite decoder target");
   if(!active&&using_encoder){reseeded=false;return raw;} // Initial PLANNER remains unchanged.
   bool begin=!active||using_encoder!=last_encoder;
   for(int i=0;i<29;++i){
     if(!JointTargetLimiter::finite(sent[i])||!JointTargetLimiter::finite(measured[i]))throw std::runtime_error("nonfinite measured/published target");
     if(raw[i]<lo[i]||raw[i]>hi[i])throw std::runtime_error("raw joint target outside limits: "+names()[i]);
     if(sent[i]<lo[i]||sent[i]>hi[i]||measured[i]<lo[i]||measured[i]>hi[i])throw std::runtime_error("published/measured joint outside limits: "+names()[i]);
     if(begin&&std::abs(sent[i]-measured[i])>tracking[i])throw std::runtime_error("takeover tracking error: "+names()[i]);
   }
   double step=begin?nominal_dt:std::min(nominal_dt,now-last_time);
   if(!JointTargetLimiter::finite(step)||step<=0)throw std::runtime_error("nonadvancing smoothing time");
   // Commit state only after every joint passes validation. Use last published q
   // for each filter update, never an unacknowledged queued target.
   const Vec anchor=begin?sent:start;
   double next_elapsed=begin?0:elapsed+step;
   double u=std::clamp(next_elapsed/blend_time,0.0,1.0),w=3*u*u-2*u*u*u;
   Vec out{},next_alpha{};
   for(int i=0;i<29;++i){
     next_alpha[i]=step/(tau[i]+step);
     const double blended=anchor[i]+w*(raw[i]-anchor[i]);
     double delta=next_alpha[i]*(blended-sent[i]);
     if(rate_limited[i])delta=std::clamp(delta,-velocity[i]*step,velocity[i]*step);
     out[i]=sent[i]+delta;
   }
   start=anchor;elapsed=next_elapsed;weight=w;dt=step;alpha=next_alpha;
   last_time=now;last_encoder=using_encoder;active=true;reseeded=begin;return out;
 }
};
