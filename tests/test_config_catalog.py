"""Checks for selectable experiment configuration naming and loading."""

from __future__ import annotations

import importlib
import pkgutil
import unittest

import configs
from swarm.domain.config import ExperimentConfig
from run_experiment import _load_config


EXPECTED_CONFIGS = {
    "bearing_two",
    "bearing_three",
    "bearing_four",
    "bearing_six",
}
LEGACY_CONFIGS = {
    "two_vehicle",
    "three_vehicle",
    "four_vehicle",
    "six_vehicle",
}


class ConfigCatalogTest(unittest.TestCase):
    def test_all_config_modules_follow_algorithm_name_convention(self) -> None:
        module_names = {module.name for module in pkgutil.iter_modules(configs.__path__)}
        self.assertTrue(EXPECTED_CONFIGS.issubset(module_names))
        self.assertTrue(LEGACY_CONFIGS.isdisjoint(module_names))

        for module_name in sorted(module_names):
            module = importlib.import_module("configs." + module_name)
            config = getattr(module, "CONFIG", None)
            self.assertIsInstance(config, ExperimentConfig)
            config.validate()
            self.assertEqual(config.name, module_name)
            self.assertTrue(module_name.startswith(config.algorithm + "_"))

    def test_cli_loader_accepts_renamed_config(self) -> None:
        config = _load_config("bearing_six")

        self.assertEqual(config.name, "bearing_six")
        self.assertEqual(config.algorithm, "bearing")


if __name__ == "__main__":
    unittest.main()
