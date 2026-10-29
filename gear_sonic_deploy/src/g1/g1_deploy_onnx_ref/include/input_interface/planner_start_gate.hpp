#pragma once
namespace sonic {
class PlannerStartGate {
 public:
  enum class Result { idle, waiting, ready, failed, timeout };
  bool pending=false;
  double began=0;
  void begin(double now){pending=true;began=now;}
  Result poll(bool enabled,bool initialized,bool motion_ready,double now){
    if(!pending)return Result::idle;
    if(!enabled){pending=false;return Result::failed;}
    if(motion_ready){
      pending=false;return initialized?Result::ready:Result::failed;
    }
    if(now-began>5.0){pending=false;return Result::timeout;}
    return Result::waiting;
  }
};
}
