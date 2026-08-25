#pragma once

#include <cstdint>
#include <string>

namespace go2_chassis {

class ISportClient {
 public:
  virtual ~ISportClient() = default;
  virtual bool Initialize(const std::string &network_interface,
                          std::string *error) = 0;
  // Confirm that the official sport-client path completed initialization.
  // Runtime zero preparation and watchdogs remain independent of this check.
  virtual std::int32_t PrepareArm() = 0;
  virtual std::int32_t ClassicWalk(bool enabled) = 0;
  virtual std::int32_t Move(float vx, float vy, float wz) = 0;
  virtual std::int32_t StopMove() = 0;
};

}  // namespace go2_chassis
