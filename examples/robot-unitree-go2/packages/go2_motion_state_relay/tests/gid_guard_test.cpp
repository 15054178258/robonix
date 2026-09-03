#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>

#include "go2_motion_state_relay/gid_guard.hpp"

namespace {

using go2_motion_state_relay::GraphObservation;
using go2_motion_state_relay::HeartbeatTimestampGuard;
using go2_motion_state_relay::PublisherGid;
using go2_motion_state_relay::PublisherGidGuard;

PublisherGid Gid(const std::uint8_t seed) {
  PublisherGid gid{};
  for (std::size_t index = 0U; index < gid.size(); ++index) {
    gid[index] = static_cast<std::uint8_t>(seed + index);
  }
  return gid;
}

PublisherGid GraphGid(const std::uint8_t seed) {
  PublisherGid gid{};
  for (std::size_t index = 0U; index < 16U; ++index) {
    gid[index] = static_cast<std::uint8_t>(seed + index);
  }
  return gid;
}

PublisherGid MessageGid(const std::uint8_t seed) {
  PublisherGid gid{};
  for (std::size_t index = 0U; index < 8U; ++index) {
    gid[index] = static_cast<std::uint8_t>(seed + index);
  }
  return gid;
}

GraphObservation Exact(const PublisherGid &gid) {
  return GraphObservation{1U, true, true, gid};
}

void TestThreePollFreezeAndNoPrebindPublish() {
  PublisherGidGuard guard;
  const auto graph_gid = GraphGid(1U);
  const auto message_gid = MessageGid(41U);
  assert(!guard.ObserveMessage(message_gid, false));
  assert(!guard.faulted());
  guard.ObserveGraph(Exact(graph_gid));
  assert(!guard.graph_bound());
  assert(!guard.ObserveMessage(message_gid, true));
  guard.ObserveGraph(Exact(graph_gid));
  assert(!guard.graph_bound());
  guard.ObserveGraph(Exact(graph_gid));
  assert(guard.graph_bound());
  assert(!guard.message_bound());
  assert(guard.stable_graph_polls() == 3U);
  assert(!guard.ObserveMessage(message_gid, true));
  assert(!guard.ObserveMessage(message_gid, true));
  assert(guard.ObserveMessage(message_gid, true));
  assert(guard.bound());
  assert(guard.stable_message_samples() == 3U);
  assert(guard.frozen_graph_gid() == graph_gid);
  assert(guard.frozen_message_gid() == message_gid);
  assert(guard.per_message_gid_verified());
}

void TestZeroAndAmbiguousPublishersFaultSticky() {
  PublisherGidGuard zero_publishers;
  zero_publishers.ObserveGraph(GraphObservation{});
  assert(zero_publishers.faulted());
  assert(zero_publishers.fault()->reason == "publisher_count_not_one");
  assert(zero_publishers.fault()->publisher_count == 0U);
  zero_publishers.ObserveGraph(Exact(GraphGid(2U)));
  assert(!zero_publishers.bound());

  PublisherGidGuard multiple_publishers;
  multiple_publishers.ObserveGraph(
      GraphObservation{2U, true, true, GraphGid(2U)});
  assert(multiple_publishers.faulted());
  assert(multiple_publishers.fault()->publisher_count == 2U);

  PublisherGidGuard zero_gid;
  zero_gid.ObserveGraph(GraphObservation{1U, true, true, PublisherGid{}});
  assert(zero_gid.faulted());
  assert(zero_gid.fault()->reason == "graph_publisher_gid_zero");
}

void TestIdentityAndQosAreExact() {
  PublisherGidGuard identity;
  identity.ObserveGraph(GraphObservation{1U, false, true, GraphGid(3U)});
  assert(identity.faulted());
  assert(identity.fault()->reason == "publisher_identity_mismatch");

  PublisherGidGuard qos;
  qos.ObserveGraph(GraphObservation{1U, true, false, GraphGid(3U)});
  assert(qos.faulted());
  assert(qos.fault()->reason == "publisher_qos_mismatch");
}

void TestCandidateChangeCannotRebind() {
  PublisherGidGuard guard;
  const auto first = GraphGid(4U);
  const auto second = GraphGid(5U);
  guard.ObserveGraph(Exact(first));
  guard.ObserveGraph(Exact(second));
  assert(guard.faulted());
  assert(guard.fault()->reason ==
         "graph_publisher_gid_changed_before_freeze");
  assert(guard.fault()->graph_gid == first);
  assert(guard.fault()->observed_gid == second);
  assert(guard.fault()->observed_gid_domain ==
         go2_motion_state_relay::GidDomain::kGraph);
  guard.ObserveGraph(Exact(first));
  guard.ObserveGraph(Exact(first));
  assert(!guard.bound());
}

void TestFrozenGraphAndMessageChangesFault() {
  const auto first = GraphGid(6U);
  const auto second = GraphGid(7U);

  PublisherGidGuard graph_guard;
  for (int poll = 0; poll < 3; ++poll) {
    graph_guard.ObserveGraph(Exact(first));
  }
  graph_guard.ObserveGraph(Exact(second));
  assert(graph_guard.faulted());
  assert(graph_guard.fault()->reason == "graph_publisher_gid_changed");
  assert(graph_guard.fault()->graph_gid == first);
  assert(graph_guard.fault()->observed_gid == second);

  PublisherGidGuard message_guard;
  for (int poll = 0; poll < 3; ++poll) {
    message_guard.ObserveGraph(Exact(first));
  }
  const auto message_gid = MessageGid(47U);
  const auto changed_message_gid = MessageGid(71U);
  for (int sample = 0; sample < 3; ++sample) {
    (void)message_guard.ObserveMessage(message_gid, true);
  }
  assert(message_guard.bound());
  assert(!message_guard.ObserveMessage(changed_message_gid, true));
  assert(message_guard.faulted());
  assert(message_guard.fault()->reason == "message_publisher_gid_changed");
  assert(message_guard.fault()->graph_gid == first);
  assert(message_guard.fault()->message_gid == message_gid);
  assert(message_guard.fault()->observed_gid == changed_message_gid);
  assert(!message_guard.ObserveMessage(message_gid, true));

  PublisherGidGuard implementation;
  for (int poll = 0; poll < 3; ++poll) {
    implementation.ObserveGraph(Exact(first));
  }
  assert(!implementation.ObserveMessage(message_gid, false));
  assert(implementation.faulted());
  assert(implementation.fault()->reason ==
         "message_publisher_gid_implementation_mismatch");

  PublisherGidGuard disappeared;
  for (int poll = 0; poll < 3; ++poll) {
    disappeared.ObserveGraph(Exact(first));
  }
  disappeared.ObserveGraph(GraphObservation{});
  assert(disappeared.faulted());
  assert(disappeared.fault()->reason == "publisher_count_not_one");
  assert(disappeared.fault()->graph_gid == first);
  assert(!disappeared.fault()->observed_gid.has_value());
  assert(disappeared.fault()->publisher_count == 0U);

  PublisherGidGuard candidate_disappeared;
  candidate_disappeared.ObserveGraph(Exact(first));
  candidate_disappeared.ObserveGraph(GraphObservation{});
  assert(candidate_disappeared.faulted());
  assert(candidate_disappeared.fault()->graph_gid == first);
}

void TestHexEncodingIsExactTwentyFourBytes() {
  const auto gid = Gid(8U);
  const std::string encoded = go2_motion_state_relay::GidToHex(gid);
  assert(encoded.size() == 48U);
  assert(encoded.substr(0U, 4U) == "0809");
  assert(encoded.substr(44U, 4U) == "1e1f");
  assert(go2_motion_state_relay::IsNonzeroGid(gid));
  assert(!go2_motion_state_relay::IsNonzeroGid(PublisherGid{}));
}

void TestCycloneGidShapesAreExplicitAndFailClosed() {
  const auto graph_gid = GraphGid(9U);
  const auto message_gid = MessageGid(29U);
  assert(go2_motion_state_relay::IsCycloneGraphGidShape(graph_gid));
  assert(go2_motion_state_relay::IsCycloneMessageGidShape(message_gid));
  assert(!go2_motion_state_relay::IsCycloneMessageGidShape(graph_gid));

  auto bad_graph = graph_gid;
  bad_graph[23] = 1U;
  PublisherGidGuard graph_guard;
  graph_guard.ObserveGraph(Exact(bad_graph));
  assert(graph_guard.faulted());
  assert(graph_guard.fault()->reason ==
         "graph_publisher_gid_shape_mismatch");

  PublisherGidGuard message_guard;
  for (int poll = 0; poll < 3; ++poll) {
    message_guard.ObserveGraph(Exact(graph_gid));
  }
  auto bad_message = message_gid;
  bad_message[8] = 1U;
  assert(!message_guard.ObserveMessage(bad_message, true));
  assert(message_guard.faulted());
  assert(message_guard.fault()->reason ==
         "message_publisher_gid_shape_mismatch");
}

void TestInitialDiscoveryGraceIsBoundedAndNeverReopens() {
  constexpr std::int64_t started_ns = 10'000'000'000LL;
  assert(go2_motion_state_relay::MayDeferInitialPublisherAbsence(
      false, started_ns, started_ns));
  assert(go2_motion_state_relay::MayDeferInitialPublisherAbsence(
      false, started_ns,
      started_ns +
          go2_motion_state_relay::kInitialGraphDiscoveryTimeoutNs - 1LL));
  assert(!go2_motion_state_relay::MayDeferInitialPublisherAbsence(
      false, started_ns,
      started_ns + go2_motion_state_relay::kInitialGraphDiscoveryTimeoutNs));
  assert(!go2_motion_state_relay::MayDeferInitialPublisherAbsence(
      true, started_ns, started_ns + 1LL));
  assert(!go2_motion_state_relay::MayDeferInitialPublisherAbsence(
      false, started_ns, started_ns - 1LL));
}

void TestHeartbeatUsesSenderAgeAndLatchesRegression() {
  HeartbeatTimestampGuard heartbeat;
  assert(!heartbeat.Fresh(1'000LL, 200LL));
  assert(heartbeat.Observe(1'000LL));
  assert(heartbeat.Fresh(1'200LL, 200LL));
  assert(!heartbeat.Fresh(1'201LL, 200LL));

  // A heartbeat that sat buffered while the worker was stalled remains stale;
  // reading it at "now" does not refresh its sender timestamp.
  HeartbeatTimestampGuard buffered;
  assert(buffered.Observe(2'000LL));
  assert(!buffered.Fresh(10'000LL, 200LL));

  assert(!heartbeat.Observe(1'000LL));
  assert(heartbeat.faulted());
  assert(heartbeat.reason() == "supervisor_heartbeat_nonmonotonic");
  assert(!heartbeat.Observe(1'300LL));
  assert(!heartbeat.Fresh(1'300LL, 200LL));
}

}  // namespace

int main() {
  TestThreePollFreezeAndNoPrebindPublish();
  TestZeroAndAmbiguousPublishersFaultSticky();
  TestIdentityAndQosAreExact();
  TestCandidateChangeCannotRebind();
  TestFrozenGraphAndMessageChangesFault();
  TestHexEncodingIsExactTwentyFourBytes();
  TestCycloneGidShapesAreExplicitAndFailClosed();
  TestInitialDiscoveryGraceIsBoundedAndNeverReopens();
  TestHeartbeatUsesSenderAgeAndLatchesRegression();
  std::cout << "gid_guard_test: PASS\n";
  return 0;
}
