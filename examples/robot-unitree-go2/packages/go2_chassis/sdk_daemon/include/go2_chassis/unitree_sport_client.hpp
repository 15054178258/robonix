#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "go2_chassis/rpc_response_guard.hpp"
#include "go2_chassis/sport_client_interface.hpp"

namespace unitree::robot::go2 {
class SportClient;
}

namespace go2_chassis {

class AuditedSportClient;
class UnitreeRpcObserver;

class UnitreeSportClient final : public ISportClient {
 public:
  UnitreeSportClient();
  ~UnitreeSportClient() override;

  UnitreeSportClient(const UnitreeSportClient &) = delete;
  UnitreeSportClient &operator=(const UnitreeSportClient &) = delete;

  bool Initialize(const std::string &network_interface,
                  std::string *error) override;
  std::int32_t PrepareArm() override;
  std::int32_t ClassicWalk(bool enabled) override;
  std::int32_t Move(float vx, float vy, float wz) override;
  std::int32_t StopMove() override;

 private:
  std::int32_t VerifiedCall(std::int64_t api_id,
                            const std::function<std::int32_t()> &call,
                            std::int32_t expected_priority,
                            bool expected_noreply,
                            const std::string &expected_parameter);

  std::unique_ptr<AuditedSportClient> client_;
  std::unique_ptr<UnitreeRpcObserver> rpc_observer_;
};

}  // namespace go2_chassis
