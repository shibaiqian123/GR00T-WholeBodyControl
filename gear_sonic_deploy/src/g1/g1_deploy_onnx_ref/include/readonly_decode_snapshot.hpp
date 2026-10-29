#pragma once
#include <array>
#include <vector>
#include <cmath>
#include <stdexcept>
// Pure packing. Token is supplied separately to the offline decoder.
// No buffers, DDS, motor commands, state mutation or policy inference here.
template<class Entry,class Gravity>
std::array<double,994> PackReadonlyDecodeSnapshot(const std::vector<Entry>& history,Gravity gravity) {
 if(history.size()!=10)throw std::runtime_error("Expected ten actual/padded logger entries");
 std::array<double,994> out{};
 for(size_t f=0;f<10;++f){
  const auto& e=history[f];
  if(e.body_q.size()!=29||e.body_dq.size()!=29||e.last_action.size()!=29)
   throw std::runtime_error("Incomplete logger history");
  const auto g=gravity(e.base_quat);
  for(size_t j=0;j<3;++j){out[64+f*3+j]=e.base_ang_vel[j];out[964+f*3+j]=g[j];}
  for(size_t j=0;j<29;++j){out[94+f*29+j]=e.body_q[j];out[384+f*29+j]=e.body_dq[j];out[674+f*29+j]=e.last_action[j];}
 }
 for(double v:out)if(!std::isfinite(v))throw std::runtime_error("Nonfinite snapshot");
 return out;
}
