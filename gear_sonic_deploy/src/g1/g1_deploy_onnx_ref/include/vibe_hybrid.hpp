#pragma once
// Simulation-only RPC adapter. Inference worker uses the existing SONIC TRT library.
#include <zmq.hpp>
#include <nlohmann/json.hpp>
#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>
inline std::string vibe_hybrid_endpoint;
inline constexpr uint32_t VIBE_MAGIC=0x56494245;
class VibeHybridClient {
  zmq::context_t context_{1};
  zmq::socket_t socket_{context_,zmq::socket_type::req};
public:
  explicit VibeHybridClient(const std::string& endpoint) {
    socket_.set(zmq::sockopt::linger,0);
    socket_.set(zmq::sockopt::rcvtimeo,500);
    socket_.set(zmq::sockopt::sndtimeo,500);
    socket_.connect(endpoint);
  }
  std::array<float,29> call(uint32_t tick,uint32_t epoch,bool pose,
                            const std::array<float,29>& candidate,
                            const std::array<double,4>& heading={1,0,0,0}) {
    nlohmann::json request={{"tick",tick},{"epoch",epoch},{"mode",pose?"POSE":"SONIC"},
                            {"target",candidate},{"heading",heading}};
    const auto bytes=request.dump();
    if(!socket_.send(zmq::buffer(bytes),zmq::send_flags::none))
      throw std::runtime_error("Hybrid worker send timeout");
    zmq::message_t response;
    if(!socket_.recv(response,zmq::recv_flags::none))
      throw std::runtime_error("Hybrid worker receive timeout");
    auto result=nlohmann::json::parse(response.to_string());
    if(!result.at("ok").get<bool>())
      throw std::runtime_error(result.value("error","Hybrid worker rejected input"));
    if(result.at("tick").get<uint32_t>()!=tick || result.at("epoch").get<uint32_t>()!=epoch)
      throw std::runtime_error("Hybrid reply clock mismatch");
    auto target=result.at("target").get<std::array<float,29>>();
    for(float x:target)if(!std::isfinite(x))throw std::runtime_error("Nonfinite hybrid target");
    return target;
  }
};
