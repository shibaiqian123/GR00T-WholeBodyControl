#include "include/arm_target_smoother.hpp"
#include <cassert>
#include <limits>
#include <iostream>
template<class F>void rejects(F f){bool yes=false;try{f();}catch(const std::exception&){yes=true;}assert(yes);}
nlohmann::json config(){
 using V=ArmTargetSmoother::Vec;V lo,hi;lo.fill(-10);hi.fill(10);
 std::vector<std::string> names(29,"unchanged");
 std::array<std::string,8> n={"left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint","left_elbow_joint","right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint","right_elbow_joint"};
 for(int k=0;k<8;++k)names[ArmTargetSmoother::joints[k]]=n[k];
 return {{"simulation_only",true},{"units","rad"},{"joint_order","hardware_29"},{"joint_names",names},{"q_min",lo},{"q_max",hi},{"T_blend",.5},{"tau",.1},{"v_max",.5}};
}
int main(){
 auto c=config();ArmTargetSmoother s(c);using V=ArmTargetSmoother::Vec;
 V raw{},sent{};for(int i=0;i<29;++i){sent[i]=.01*i;raw[i]=sent[i]+.5;}
 assert(s.apply(raw,sent,true,0,.02)==raw);assert(!s.active);
 auto first=s.apply(raw,sent,false,1,.02);assert(s.reseeded);assert(s.weight==0);
 for(int i:ArmTargetSmoother::joints)assert(first[i]==sent[i]);
 auto next=s.apply(raw,first,false,1.02,.02);
 double u=.02/.5,w=3*u*u-2*u*u*u;
 assert(std::abs(next[15]-(sent[15]+(1./6)*w*.5))<1e-12);
 assert(!s.reseeded);assert(std::abs(s.alpha-1./6)<1e-12);
 for(int i=0;i<29;++i)if(std::find(s.joints.begin(),s.joints.end(),i)==s.joints.end())assert(next[i]==raw[i]);
 // A new chunk/raw target does not reseed; delayed cycles never accumulate a burst.
 raw[15]=-1;auto delayed=s.apply(raw,next,false,20,.02);
 assert(!s.reseeded);assert(std::abs(s.elapsed-.04)<1e-12);
 for(int i:s.joints)assert(std::abs(delayed[i]-next[i])<=.01+1e-12);
 // Returning to PLANNER also seeds from last successful publication, not old internal output.
 V published=delayed;published[15]=.123;
 auto back=s.apply(raw,published,true,20.02,.02);assert(s.reseeded);assert(back[15]==published[15]);
 auto state=s.previous;raw[15]=std::numeric_limits<double>::quiet_NaN();
 rejects([&]{s.apply(raw,back,true,21,.02);});assert(s.previous==state);
 raw[15]=11;rejects([&]{s.apply(raw,back,true,21,.02);});assert(s.previous==state);
 raw[15]=0;rejects([&]{s.apply(raw,back,true,20,.02);});
 s.reset();assert(s.apply(raw,back,true,30,.02)==raw);assert(!s.active);
 c["simulation_only"]=false;ArmTargetSmoother hardware_config(c);
 c["simulation_only"]="false";rejects([&]{ArmTargetSmoother invalid(c);});
 c=config();c["tau"]=0;rejects([&]{ArmTargetSmoother invalid(c);});
 c=config();c["joint_names"][15]="wrong";rejects([&]{ArmTargetSmoother invalid(c);});
 std::cout<<"arm target smoother tests passed\n";
}
