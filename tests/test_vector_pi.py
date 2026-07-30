"""Tests for the vector PI sliding integral window."""

import unittest

from swarm.algorithms.pid import VectorPIController


class VectorPIControllerTest(unittest.TestCase):
    def build_controller(self, window_s: float = 1.0) -> VectorPIController:
        return VectorPIController(
            kp=0.0,
            ki=1.0,
            integral_limit=100.0,
            output_limit=100.0,
            integral_window_s=window_s,
        )

    def test_integral_contains_only_recent_window(self) -> None:
        controller = self.build_controller()
        controller.update((1.0, -2.0), 0.6)
        output = controller.update((3.0, 4.0), 0.6)

        self.assertAlmostEqual(output[0], 2.2)
        self.assertAlmostEqual(output[1], 1.6)
        self.assertAlmostEqual(controller._window_duration_s, 1.0)

    def test_window_handles_sample_longer_than_window(self) -> None:
        controller = self.build_controller(window_s=0.5)

        output = controller.update((2.0, -1.0), 0.8)

        self.assertAlmostEqual(output[0], 1.0)
        self.assertAlmostEqual(output[1], -0.5)

    def test_reset_clears_window_history(self) -> None:
        controller = self.build_controller()
        controller.update((2.0,), 0.5)

        controller.reset()
        output = controller.update((1.0,), 0.25)

        self.assertAlmostEqual(output[0], 0.25)

    def test_window_duration_must_be_positive(self) -> None:
        with self.assertRaises(ValueError):
            self.build_controller(window_s=0.0)


if __name__ == "__main__":
    unittest.main()
