#pragma once

#include <cstdint>
#include <string>

namespace g1_chassis {

/**
 * @brief Interface the G1 SDK2 LocoClient.
 *
 * The daemon delegates all SDK2 calls through this interface so that tests
 * can inject a mock client and verify the daemon core without a real robot.
 */
class ILocoClient {
 public:
  virtual ~ILocoClient() = default;

  /**
   * @brief Initialize the SDK2 client on the given network interface.
   * @return true on success, false on failure.
   */
  virtual bool Initialize(const std::string &network_interface,
                          std::string *error) = 0;

  /**
   * @brief Confirm that the SDK2 LocoClient path completed initialization.
   * @return 0 on success, non-zero if unavailable.
   */
  virtual std::int32_t PrepareArm() = 0;

  /**
   * @brief Send a velocity command (v_x, v_y, yaw_rate) with duration.
   * @return 0 on success, non-zero on failure.
   */
  virtual std::int32_t SetVelocity(float vx, float vy, float omega,
                                    float duration) = 0;

  /**
   * @brief Stop all movement immediately.
   * @return 0 on success, non-zero on failure.
   */
  virtual std::int32_t StopMove() = 0;

  /**
   * @brief Enter balance-stand (feet on ground, balance maintained).
   * @return 0 on success, non-zero on failure.
   */
  virtual std::int32_t BalanceStand() = 0;

  /**
   * @brief Damp the robot — relax joints, fall onto feet.
   * @return 0 on success, non-zero on failure.
   */
  virtual std::int32_t Damp() = 0;

  /**
   * @brief Stand up from damp/sit position.
   * @return 0 on success, non-zero on failure.
   */
  virtual std::int32_t StandUp() = 0;
};

}  // namespace g1_chassis
