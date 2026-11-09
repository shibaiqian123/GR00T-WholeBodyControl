import unittest

from deploy.check_server import validate


def valid_info():
    hashes = {
        key: "a" * 64
        for key in (
            "checkpoint_sha256",
            "config_sha256",
            "statistics_sha256",
            "text_cache_sha256",
            "text_manifest_sha256",
            "task_sha256",
        )
    }
    return dict(
        hashes,
        state_dim=31,
        action_shape=[50, 66],
        token_shape=[50, 64],
        native_action_shape=[32, 66],
        native_control_hz=20,
        control_hz=50,
        time_adapter="zero_order_hold_latest_20hz_row_on_50hz_ticks",
        token_interpolation=False,
        rtc_mode="disabled",
        rtc_enabled=False,
        gripper_output_units="motor_output_radians",
        gripper_label_mode="continuous",
    )


class ServerContractTest(unittest.TestCase):
    def test_accepts_exact_contract(self):
        self.assertEqual(validate(valid_info())["state_dim"], 31)

    def test_rejects_dit4dit_or_wrong_timing(self):
        for key, value in (
            ("rtc_mode", "test_time"),
            ("native_control_hz", 50),
            ("time_adapter", "linear"),
            ("gripper_label_mode", "binary"),
        ):
            info = valid_info()
            info[key] = value
            with self.assertRaises(ValueError, msg=key):
                validate(info)


if __name__ == "__main__":
    unittest.main()
