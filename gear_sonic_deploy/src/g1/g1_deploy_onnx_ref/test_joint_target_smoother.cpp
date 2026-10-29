#include "include/joint_target_smoother.hpp"
#include <cassert>
#include <iostream>
#include <limits>
template<class F>void rejects(F f){bool yes=false;try{f();}catch(const std::exception&){yes=true;}assert(yes);}
nlohmann::json config(){using V=JointTargetSmoother::Vec;V lo,hi,tau{},track;lo.fill(-4);hi.fill(4);track.fill(.2);
 nlohmann::json rates=nlohmann::json::array();for(int i=0;i<29;++i)rates.push_back(nullptr);
 return {{"schema_version",2},{"simulation_only",true},{"units","rad"},{"joint_order","hardware_29"},{"joint_names",JointTargetSmoother::names()},{"q_min",lo},{"q_max",hi},{"tau",tau},{"v_max",rates},{"max_start_tracking_error",track},{"tracking_error_source","synthetic unit test only"},{"T_blend",.5}};}
int main(){using V=JointTargetSmoother::Vec;auto c=config();JointTargetSmoother s(c);V raw{},sent{},measured{};
 for(int i=0;i<29;++i){sent[i]=.01*i;measured[i]=sent[i];raw[i]=sent[i]+.1+.01*i;}
 assert(s.apply(raw,sent,measured,true,0,.02)==raw);assert(!s.active);
 auto first=s.apply(raw,sent,measured,false,1,.02);assert(first==sent);assert(s.reseeded);assert(s.weight==0);
 auto second=s.apply(raw,first,measured,false,1.02,.02);double u=.04,w=3*u*u-2*u*u*u;
 for(int i=0;i<29;++i)assert(std::abs(second[i]-(sent[i]+w*(raw[i]-sent[i])))<1e-12);
 // Whole body common w; different joint params affect only the optional second stage.
 c["tau"][15]=.1;c["v_max"][15]=.5;JointTargetSmoother grouped(c);
 grouped.apply(raw,sent,measured,false,2,.02);auto mixed=grouped.apply(raw,sent,measured,false,2.02,.02);
 assert(std::abs(mixed[15]-(sent[15]+w*(raw[15]-sent[15])/6))<1e-12);
 assert(std::abs(mixed[0]-second[0])<1e-12);assert(std::abs(mixed[20]-second[20])<1e-12);
 // No chunk reset or catch-up: a new raw target plus long delay only advances one dt.
 raw[20]=-.3;auto delayed=grouped.apply(raw,mixed,measured,false,30,.02);
 assert(!grouped.reseeded);assert(std::abs(grouped.elapsed-.04)<1e-12);
 assert(std::abs(delayed[15]-mixed[15])<=.01+1e-12);
 // Filtering uses supplied last published target, never prior queued output.
 c["T_blend"]=.01;JointTargetSmoother published(c);published.apply(raw,sent,measured,false,40,.02);
 auto cmd=published.apply(raw,sent,measured,false,40.02,.02);
 auto again=published.apply(raw,sent,measured,false,40.04,.02);
 assert(std::abs(cmd[15]-again[15])<1e-12);
 // Return to encoder mode checks actual tracking before reseeding any joint.
 V bad=measured;bad[10]-=.3;double before=grouped.elapsed;
 rejects([&]{grouped.apply(raw,delayed,bad,true,31,.02);});assert(grouped.elapsed==before);
 auto returned=grouped.apply(raw,delayed,delayed,true,31,.02);assert(returned==delayed);assert(grouped.reseeded);
 // Bounded wrists are not wrapped: +3.13 -> -3.13 follows the bounded numeric path.
 auto wc=config();JointTargetSmoother wrist(wc);V a{},z{},q{};a[20]=-3.13;z[20]=q[20]=3.13;
 wrist.apply(a,z,q,false,0,.02);auto mid=wrist.apply(a,z,q,false,.02,.02);assert(mid[20]<3.13);assert(mid[20]>-3.13);
 a[20]=5;rejects([&]{wrist.apply(a,mid,q,false,.04,.02);});
 a[20]=std::numeric_limits<double>::quiet_NaN();rejects([&]{wrist.apply(a,mid,q,false,.04,.02);});
 grouped.reset();assert(!grouped.active);assert(grouped.apply(raw,sent,measured,true,50,.02)==raw);
 c=config();c["max_start_tracking_error"][2]=nullptr;rejects([&]{JointTargetSmoother invalid(c);});
 c=config();c["joint_names"][0]="wrong";rejects([&]{JointTargetSmoother invalid(c);});
 std::cout<<"joint target smoother tests passed\n";
}
