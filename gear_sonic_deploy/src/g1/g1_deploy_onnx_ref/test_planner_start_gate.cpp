#include "input_interface/planner_start_gate.hpp"
#include <cassert>
#include <mutex>
#include <thread>
int main(){
 using G=sonic::PlannerStartGate;using R=G::Result;
 G g;assert(g.poll(true,true,true,0)==R::idle);
 g.begin(0);assert(g.poll(true,false,false,0)==R::waiting);
 assert(g.poll(true,true,false,5)==R::waiting);
 assert(g.poll(true,true,false,5.001)==R::timeout);
 g.begin(10);assert(g.poll(false,false,false,10.1)==R::failed);
 g.begin(20);assert(g.poll(true,false,true,20.1)==R::failed);
 g.begin(30);assert(g.poll(true,true,true,30.1)==R::ready);
 // Input holds the same mutex as CONTROL. Waiting must return and release it.
 std::mutex mutex;bool motion_ready=false;g.begin(40);
 {std::lock_guard<std::mutex> lock(mutex);assert(g.poll(true,true,motion_ready,40)==R::waiting);}
 std::thread control([&]{std::lock_guard<std::mutex> lock(mutex);motion_ready=true;});control.join();
 {std::lock_guard<std::mutex> lock(mutex);assert(g.poll(true,true,motion_ready,40.02)==R::ready);}
}
