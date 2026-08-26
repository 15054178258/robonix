#include <catch2/catch_test_macros.hpp>
#include "g1_chassis/protocol.hpp"

using namespace g1_chassis;

TEST_CASE("Packet sizes are exact", "[ipc]") {
  REQUIRE(sizeof(CommandPacket) == 24);
  REQUIRE(sizeof(ReplyPacket) == 16);
}

TEST_CASE("Reply code values", "[ipc]") {
  REQUIRE(static_cast<uint8_t>(ReplyCode::kOk) == 0);
  REQUIRE(static_cast<uint8_t>(ReplyCode::kDisabled) == 1);
  REQUIRE(static_cast<uint8_t>(ReplyCode::kDisarmed) == 2);
  REQUIRE(static_cast<uint8_t>(ReplyCode::kFaulted) == 3);
  REQUIRE(static_cast<uint8_t>(ReplyCode::kMalformed) == 4);
  REQUIRE(static_cast<uint8_t>(ReplyCode::kSdkError) == 5);
}

TEST_CASE("MakeReply produces correct structure", "[ipc]") {
  ReplyPacket reply = MakeReply(42, ReplyCode::kOk, true, false);
  REQUIRE(reply.type == static_cast<uint8_t>(PacketType::kReply));
  REQUIRE(reply.sequence == 42);
  REQUIRE(reply.code == static_cast<uint8_t>(ReplyCode::kOk));
  REQUIRE(reply.armed == 1);
  REQUIRE(reply.faulted == 0);
}

TEST_CASE("MotionWatchdogDeploymentEligible", "[config]") {
  REQUIRE(MotionWatchdogDeploymentEligible(true, 300) == true);
  REQUIRE(MotionWatchdogDeploymentEligible(false, 300) == false);
  REQUIRE(MotionWatchdogDeploymentEligible(true, 500) == false);
}
