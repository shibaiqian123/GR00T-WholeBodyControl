// Offline only: no DDS, sockets, robot interface or G1Deploy construction.
#include "control_policy.hpp"
#include "policy_parameters.hpp"
#include "startup_audit.hpp"
#include <fstream>
#include <cmath>
int main(int argc,char** argv){
 if(argc!=4){std::cerr<<"usage: replay_sonic_decoder model.onnx observations.f32 actions.f32\n";return 2;}
 std::ifstream in(argv[2],std::ios::binary|std::ios::ate);
 if(!in || in.tellg()<=0 || in.tellg()%(994*sizeof(float))!=0){std::cerr<<"Expected nonempty complete 994-float records\n";return 2;}
 const auto count=static_cast<size_t>(in.tellg())/(994*sizeof(float));in.seekg(0);
 std::vector<float> observations(count*994);in.read(reinterpret_cast<char*>(observations.data()),observations.size()*sizeof(float));
 for(float x:observations)if(!std::isfinite(x)){std::cerr<<"Nonfinite input\n";return 2;}
 PolicyEngine policy;if(!policy.Initialize(argv[1],false)||policy.GetInputDimension()!=994||!policy.CaptureGraph())return 3;
 std::ofstream out(argv[3],std::ios::binary);if(!out)return 4;
 for(size_t n=0;n<count;++n){auto& input=policy.GetInputBuffer();std::copy_n(observations.begin()+n*994,994,input.begin());if(!policy.Infer())return 5;const auto& action=policy.GetActionBuffer();if(action.size()!=29)return 6;for(float x:action)if(!std::isfinite(x))return 7;out.write(reinterpret_cast<const char*>(action.data()),29*sizeof(float));}
 startup_audit::Row meta("mapping");meta.array("mapping",isaaclab_to_mujoco).array("scale",g1_action_scale).array("default_angles",default_angles);
 std::ofstream mapping(std::string(argv[3])+".mapping.json");mapping<<meta.finish()<<'\n';
 return out&&mapping?0:8;
}
