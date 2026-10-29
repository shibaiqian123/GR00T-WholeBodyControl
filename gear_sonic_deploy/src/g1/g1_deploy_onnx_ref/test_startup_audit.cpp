#include "startup_audit.hpp"
#include <array>
#include <iostream>
int main(int argc,char**argv){
 if(argc<3)return 2;
 setenv("SONIC_STARTUP_AUDIT",argv[1],1);
 const bool overflow=std::string(argv[2])=="overflow";
 {startup_audit::Writer w(overflow?1:1024);std::vector<std::thread> threads;
 for(int t=0;t<8;t++)threads.emplace_back([&,t]{
  std::array<double,994> payload{};
  for(int i=0;i<1000;i++){
   startup_audit::Row row("stress");row.scalar("producer",t).scalar("sequence",i);if(!overflow)row.array("input",payload);w.send(row);
   if(!overflow)std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
 });for(auto& t:threads)t.join();}
}
