import asyncio
import unittest

try:
    from deploy.service import create_app
except ModuleNotFoundError as exc:
    if exc.name != "fastapi":
        raise
    create_app = None


class FakePolicy:
    info = {"rtc_mode": "disabled", "state_dim": 31}


@unittest.skipIf(create_app is None, "FastAPI is not installed in this test environment")
class ServiceTest(unittest.TestCase):
    def test_info_has_no_motor_and_disabled_rtc(self):
        app = create_app(FakePolicy(), 10, enable_preview_source=True)
        routes = {route.path: route for route in app.routes if hasattr(route, "path")}
        info = asyncio.run(routes["/info"].endpoint())
        health = asyncio.run(routes["/health"].endpoint())
        self.assertEqual(info["rtc_mode"], "disabled")
        self.assertFalse(info["rtc_enabled"])
        self.assertFalse(health["motor_interfaces"])
        self.assertTrue(info["preview_source_enabled"])

    def test_bad_horizon_rejected(self):
        for horizon in (0, 50):
            with self.assertRaises(ValueError):
                create_app(FakePolicy(), horizon)


if __name__ == "__main__":
    unittest.main()
