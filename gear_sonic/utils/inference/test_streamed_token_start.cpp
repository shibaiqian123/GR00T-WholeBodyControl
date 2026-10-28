#include <initializer_list>
#include <cassert>
#include "streamed_token_start.hpp"
int main() {
  for (bool request : {false,true}) for (bool stop : {false,true})
  for (bool token : {false,true}) for (bool initial : {false,true}) {
    bool started=initial,play=true,heading=false;
    auto r=sonic::ApplyTokenStart(request,stop,token,started,play,heading);
    bool applied=request&&!stop&&token&&!initial;
    assert((r==sonic::TokenStartResult::Applied)==applied);
    assert(started==(initial||applied));
    assert(play==!applied);assert(heading==applied);
  }
  bool started=false,play=true,heading=false;
  assert(sonic::ApplyTokenStart(true,false,false,started,play,heading)==sonic::TokenStartResult::MissingToken);
  assert(sonic::ApplyTokenStart(false,false,true,started,play,heading)==sonic::TokenStartResult::NoRequest);
  assert(!started); // Token arrival alone cannot replay a rejected start.
  assert(sonic::ApplyTokenStart(true,true,true,started,play,heading)==sonic::TokenStartResult::Stopped);
  assert(!started); // Concurrent stop wins.
}
