#include "g1_chassis/unitree_loco_client.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>

// The real SDK2 LocoClient type lives inside unitree_sdk2.
// This is the same path used by go2_sport_daemon: SDK2 is built as a
// submodule and linked privately so the ABI never leaks into ROS.
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

namespace g1_chassis {

bool UnitreeLocoClient::Initialize(const std::string &network_interface,
                                    std::string *error) {
  try {
    std::cerr << "[g1-sdk] ChannelFactory init: interface=" << network_interface
              << "\n";
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    std::cerr << "[g1-sdk] LocoClient create/init\n";
    client_ = new unitree::robot::g1::LocoClient();
    static_cast<unitree::robot::g1::LocoClient *>(client_)->Init();
    static_cast<unitree::robot::g1::LocoClient *>(client_)->SetTimeout(10.0F);

    std::cerr << "[g1-sdk] SDK2 initialized\n";
    return true;
  } catch (const std::exception &exception) {
    if (error != nullptr) {
      *error = exception.what();
    }
    std::cerr << "[g1-sdk] Init failed: " << exception.what() << "\n";
    client_ = nullptr;
    return false;
  } catch (...) {
    if (error != nullptr) {
      *error = "unknown exception during SDK2 init";
    }
    std::cerr << "[g1-sdk] Init failed: unknown exception\n";
    client_ = nullptr;
    return false;
  }
}

std::int32_t UnitreeLocoClient::PrepareArm() {
  if (!client_) return -1;
  // G1 LocoClient is ready once Init() succeeds. No additional arm step.
  return 0;
}

std::int32_t UnitreeLocoClient::SetVelocity(float vx, float vy, float omega,
                                             float duration) {
  if (!client_) return -1;
  return static_cast<unitree::robot::g1::LocoClient *>(client_)->SetVelocity(
      vx, vy, omega, duration);
}

std::int32_t UnitreeLocoClient::StopMove() {
  if (!client_) return -1;
  return static_cast<unitree::robot::g1::LocoClient *>(client_)->StopMove();
}

std::int32_t UnitreeLocoClient::BalanceStand() {
  if (!client_) return -1;
  return static_cast<unitree::robot::g1::LocoClient *>(client_)->BalanceStand();
}

std::int32_t UnitreeLocoClient::Damp() {
  if (!client_) return -1;
  return static_cast<unitree::robot::g1::LocoClient *>(client_)->Damp();
}

std::int32_t UnitreeLocoClient::StandUp() {
  if (!client_) return -1;
  return static_cast<unitree::robot::g1::LocoClient *>(client_)->StandUp();
}

}  // namespace g1_chassis
