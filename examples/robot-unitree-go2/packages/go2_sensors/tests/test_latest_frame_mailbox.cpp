#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

#include "go2_sensors/latest_frame_mailbox.hpp"

namespace
{

go2_sensors::CameraFrameRecord frame(
  const std::uint64_t sequence, const std::uint64_t generation = 1U)
{
  go2_sensors::CameraFrameRecord result;
  result.header.sequence = sequence;
  result.connection_generation = generation;
  result.jpeg.assign(32U, static_cast<std::uint8_t>(sequence & 0xffU));
  return result;
}

void test_latest_replaces_pending()
{
  go2_sensors::LatestFrameMailbox mailbox;
  assert(mailbox.put(frame(1U)).accepted);
  const auto second = mailbox.put(frame(2U));
  assert(second.accepted);
  assert(second.replaced);
  assert(mailbox.depth() == 1U);
  const auto selected = mailbox.wait_take();
  assert(selected.has_value());
  assert(selected->header.sequence == 2U);
  assert(mailbox.depth() == 0U);
}

void test_generation_discard()
{
  go2_sensors::LatestFrameMailbox mailbox;
  assert(mailbox.put(frame(8U, 2U)).accepted);
  assert(!mailbox.discard_generation(1U));
  assert(mailbox.discard_generation(2U));
  assert(mailbox.depth() == 0U);
}

void test_slow_consumer_never_accumulates_fifo()
{
  using namespace std::chrono_literals;
  constexpr std::uint64_t kFrameCount = 240U;
  go2_sensors::LatestFrameMailbox mailbox;
  std::atomic<std::uint64_t> last_consumed{0U};
  std::atomic<std::size_t> maximum_depth{0U};

  std::thread consumer([&]() {
    while (true) {
      auto selected = mailbox.wait_take();
      if (!selected.has_value()) {
        return;
      }
      last_consumed.store(selected->header.sequence);
      std::this_thread::sleep_for(4ms);
    }
  });

  std::size_t replacements = 0U;
  for (std::uint64_t sequence = 1U; sequence <= kFrameCount; ++sequence) {
    const auto result = mailbox.put(frame(sequence));
    assert(result.accepted);
    if (result.replaced) {
      ++replacements;
    }
    maximum_depth.store(std::max(maximum_depth.load(), mailbox.depth()));
    std::this_thread::sleep_for(200us);
  }

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (last_consumed.load() != kFrameCount &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(1ms);
  }
  mailbox.close();
  consumer.join();

  assert(replacements > 0U);
  assert(maximum_depth.load() == 1U);
  assert(last_consumed.load() == kFrameCount);
  std::cout << "latest-frame stress: produced=" << kFrameCount
            << " replacements=" << replacements
            << " max_pending_depth=" << maximum_depth.load() << "\n";
}

void test_close_is_fail_closed_and_wakes_waiter()
{
  go2_sensors::LatestFrameMailbox mailbox;
  std::atomic<bool> woke{false};
  std::thread waiter([&]() {
    assert(!mailbox.wait_take().has_value());
    woke.store(true);
  });
  assert(!mailbox.close());
  waiter.join();
  assert(woke.load());
  assert(!mailbox.put(frame(1U)).accepted);
}

}  // namespace

int main()
{
  test_latest_replaces_pending();
  test_generation_discard();
  test_slow_consumer_never_accumulates_fifo();
  test_close_is_fail_closed_and_wakes_waiter();
  std::cout << "latest camera frame mailbox tests passed\n";
  return 0;
}
