#include <catch2/catch.hpp>
#include <type_traits>
#include "g1_chassis/protocol.hpp"

using namespace g1_chassis;

TEST_CASE("CommandPacket struct layout is compact", "[ipc]") {
  // Expected: 1(type) + 1(seq) + 4(vx) + 4(vy) + 4(omega) + 10(reserved) = 24
  // The struct should have no padding between fields.
  static_assert(std::is_trivially_copyable<CommandPacket>::value,
                "CommandPacket must be trivially copyable for IPC");
  REQUIRE(sizeof(CommandPacket) == 24);
}

TEST_CASE("ReplyPacket struct layout is compact", "[ipc]") {
  // Expected: 1(type) + 1(seq) + 1(code) + 9(reserved) + 1(armed) + 1(faulted) + 2(reserved2) = 16
  static_assert(std::is_trivially_copyable<ReplyPacket>::value,
                "ReplyPacket must be trivially copyable for IPC");
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

TEST_CASE("PacketType values", "[ipc]") {
  REQUIRE(static_cast<uint8_t>(PacketType::kCmd) == 1);
  REQUIRE(static_cast<uint8_t>(PacketType::kReply) == 2);
}
