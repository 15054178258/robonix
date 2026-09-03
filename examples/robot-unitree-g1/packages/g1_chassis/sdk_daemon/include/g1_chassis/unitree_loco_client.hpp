#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "g1_chassis/loco_client_interface.hpp"

namespace g1_chassis {

/**
 * @brief G1 SDK2 LocoClient adaptor.
 *
 * Wraps the official Unitree SDK2 LocoClient and implements ILocoClient.
 * Every SDK2 call is logged and returns a result code that the daemon core
 * uses to decide whether the RPC succeeded on the wire.
 */
class UnitreeLocoClient final : public ILocoClient {
 public:
  UnitreeLocoClient() = default;
  ~UnitreeLocoClient() override = default;

  bool Initialize(const std::string &network_interface,
                   std::string *error) override;

  std::int32_t PrepareArm() override;
  std::int32_t SetVelocity(float vx, float vy, float omega,
                            float duration) override;
  std::int32_t StopMove() override;
  std::int32_t BalanceStand() override;
  std::int32_t Damp() override;
  std::int32_t StandUp() override;

 private:
  // Internal pointer to the SDK2 LocoClient; protected by the daemon's mutex.
  void *client_{nullptr};  // opaque unitree::robot::g1::LocoClient*
};

}  // namespace g1_chassis
