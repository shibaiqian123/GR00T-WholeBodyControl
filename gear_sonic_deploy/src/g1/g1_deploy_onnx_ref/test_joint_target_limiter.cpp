#include "include/joint_target_limiter.hpp"
#include <cassert>
#include <limits>
#include <iostream>
template<class F> void rejects(F f){bool rejected=false;try{f();}catch(const std::exception&){rejected=true;}assert(rejected);}
int main(){
 using V=JointTargetLimiter::Vec;V lo,hi,v,t,raw,actual{},seed{};lo.fill(-10);hi.fill(10);t.fill(5);
 for(int i=0;i<29;++i){v[i]=0.1*(i+1);raw[i]=i%2?2:-2;}
 nlohmann::json c={{"units","rad"},{"joint_order","hardware_29"},{"constraint_source","synthetic unit test only"},{"q_min",lo},{"q_max",hi},{"max_velocity",v},{"max_tracking_error",t}};
 JointTargetLimiter l(c);auto first=l.apply(raw,actual,seed,1,.02);
 for(int i=0;i<29;++i)assert(std::abs(first[i]-(i%2?1:-1)*v[i]*.02)<1e-12);
 auto late=l.apply(raw,actual,seed,10,.02); // no accumulated catch-up after a delay
 for(int i=0;i<29;++i)assert(std::abs(late[i]-2*first[i])<1e-12);
 auto fast=l.apply(raw,actual,seed,10.001,.02);
 for(int i=0;i<29;++i)assert(std::abs(fast[i]-late[i])<=v[i]*.001+1e-12);
 auto prior=l.previous;raw[17]=11;rejects([&]{l.apply(raw,actual,seed,11,.02);});assert(l.previous==prior);
 raw[17]=std::numeric_limits<double>::quiet_NaN();rejects([&]{l.apply(raw,actual,seed,11,.02);});
 raw[17]=0;actual[0]=9;rejects([&]{l.apply(raw,actual,seed,11,.02);});actual[0]=0;
 rejects([&]{l.apply(raw,actual,seed,10,.02);});
 l.reset();seed.fill(.5);auto reset=l.apply(raw,actual,seed,12,.02);
 for(int i=0;i<29;++i)assert(std::abs(reset[i]-.5)<=v[i]*.02+1e-12);
 c["max_velocity"][1]=0;rejects([&]{JointTargetLimiter bad(c);});
 std::cout<<"joint target limiter tests passed\n";
}
