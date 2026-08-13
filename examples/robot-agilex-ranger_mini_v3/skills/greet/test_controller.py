# SPDX-License-Identifier: Apache-2.0
import unittest

from greet_skill.controller import GreetController


class FakeMotionMonitor:
    def is_moving(self):
        return False

    def close(self):
        pass


class ControllerLifecycleTest(unittest.TestCase):
    def test_active_start_is_idempotent_and_restart_gets_new_run(self):
        controller = GreetController(
            camera_topic="/camera",
            speak_endpoint="http://speech",
            detector=object(),
            greeter=object(),
            motion_monitor=FakeMotionMonitor(),
            period_s=0.01,
        )
        first = controller.start()
        self.assertEqual(first, controller.start())
        self.assertEqual(controller.cancel(first), (True, "greeting watch stopped"))
        second = controller.start()
        self.assertNotEqual(first, second)
        controller.stop_runtime()


if __name__ == "__main__":
    unittest.main()
