import unittest

import numpy as np

from deploy.protocol import (
    EXEC_HORIZON,
    MODEL_HORIZON,
    execution_source_indices,
    zero_order_hold_20hz_to_50hz,
)


class ProtocolTest(unittest.TestCase):
    def test_mapping_is_deterministic_and_does_not_interpolate(self):
        source = np.arange(MODEL_HORIZON * 66, dtype=np.float32).reshape(MODEL_HORIZON, 66)
        result, indices = zero_order_hold_20hz_to_50hz(source)
        self.assertEqual(result.shape, (EXEC_HORIZON, 66))
        np.testing.assert_array_equal(indices, execution_source_indices())
        np.testing.assert_array_equal(result, source[indices])
        self.assertEqual(int(indices[0]), 0)
        # Fifty 50 Hz ticks cover 0.98 s, so the last selected 20 Hz row is 19.
        # Rows 20..31 remain future-model horizon; do not time-stretch them.
        self.assertEqual(int(indices[-1]), 19)

    def test_rejects_nonfinite_or_wrong_shape(self):
        with self.assertRaises(ValueError):
            zero_order_hold_20hz_to_50hz(np.zeros((MODEL_HORIZON - 1, 66), dtype=np.float32))
        bad = np.zeros((MODEL_HORIZON, 66), dtype=np.float32)
        bad[0, 0] = np.nan
        with self.assertRaises(ValueError):
            zero_order_hold_20hz_to_50hz(bad)


if __name__ == "__main__":
    unittest.main()
