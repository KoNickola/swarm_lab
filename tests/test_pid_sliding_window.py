"""Tests for the scalar PID sliding integral window."""

import unittest

from swarm.algorithms.pid import PID


class PIDSlidingWindowTest(unittest.TestCase):
    def build_controller(self, window_s: float = 1.0) -> PID:
        return PID(
            kp=0.0,
            ki=1.0,
            kd=0.0,
            integral_limit=100.0,
            output_limit=None,
            integral_window_s=window_s,
        )

    def test_integral_contains_only_recent_window(self) -> None:
        controller = self.build_controller()
        controller.update(target=1.0, measured=0.0, dt=0.6)

        output = controller.update(target=3.0, measured=0.0, dt=0.6)

        self.assertAlmostEqual(output, 2.2)
        self.assertAlmostEqual(controller.integral, 2.2)
        self.assertAlmostEqual(controller._window_duration_s, 1.0)

    def test_window_handles_sample_longer_than_window(self) -> None:
        controller = self.build_controller(window_s=0.5)

        output = controller.update(target=2.0, measured=0.0, dt=0.8)

        self.assertAlmostEqual(output, 1.0)

    def test_reset_clears_window_history(self) -> None:
        controller = self.build_controller()
        controller.update(target=2.0, measured=0.0, dt=0.5)

        controller.reset()
        output = controller.update(target=1.0, measured=0.0, dt=0.25)

        self.assertAlmostEqual(output, 0.25)
        self.assertAlmostEqual(controller._window_duration_s, 0.25)

    def test_window_duration_must_be_positive(self) -> None:
        with self.assertRaises(ValueError):
            self.build_controller(window_s=0.0)


if __name__ == "__main__":
    unittest.main()
