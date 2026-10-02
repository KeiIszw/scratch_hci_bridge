import math
import socket
import time
import unittest
from threading import Thread
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from scratch_hci_bridge.command_state import CommandState, ExecutionMode
from scratch_hci_bridge.http_server import create_http_server
from scratch_hci_bridge.trajectory_scheduler import TrajectoryScheduler


def _get(url: str) -> tuple[int, str, str | None]:
    try:
        with urlopen(Request(url), timeout=2.0) as response:
            return (
                response.status,
                response.read().decode("utf-8"),
                response.headers.get("Access-Control-Allow-Origin"),
            )
    except HTTPError as error:
        with error:
            return (
                error.code,
                error.read().decode("utf-8"),
                error.headers.get("Access-Control-Allow-Origin"),
            )


class HttpServerTest(unittest.TestCase):
    def test_parallel_swing_http_commands_are_accepted_and_safely_split(
        self,
    ) -> None:
        state = CommandState()
        scheduler = TrajectoryScheduler(
            batch_wait_sec=0.01,
            feedback_timeout_sec=0.0,
        )
        scheduler.update_from_joint_state(
            ["swing_joint", "boom_joint", "arm_joint", "bucket_joint"],
            [0.0, math.radians(-80.0), math.radians(140.0), math.radians(90.0)],
        )
        callback_results: list[bool] = []

        def command_callback(command: str, value: float) -> None:
            if command in ("serial", "parallel"):
                callback_results.append(scheduler.set_execution_mode(command))
            else:
                callback_results.append(
                    scheduler.update_http_target(command, value)
                )

        server = create_http_server(
            "127.0.0.1", 0, state, command_callback
        )
        thread = Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base_url = f"http://127.0.0.1:{server.server_port}/"

        try:
            self.assertEqual(
                _get(f"{base_url}?type=command&cmd=parallel&val=1")[0],
                200,
            )
            self.assertEqual(
                _get(f"{base_url}?type=command&cmd=swing_left&val=100")[0],
                200,
            )
            self.assertEqual(
                _get(f"{base_url}?type=command&cmd=swing_left&val=100")[0],
                200,
            )
            self.assertEqual(callback_results, [True, True, True])

            first = scheduler.pending_snapshot(now=time.monotonic() + 0.02)
            self.assertIsNotNone(first)
            assert first is not None
            self.assertEqual(len(first.commands), 1)
            self.assertTrue(scheduler.mark_submitted(first))
            self.assertTrue(scheduler.mark_completed(first, successful=True))

            second = scheduler.pending_snapshot(now=time.monotonic() + 0.02)
            self.assertIsNotNone(second)
            assert second is not None
            self.assertEqual(len(second.commands), 1)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)

    def test_hci_command_http_contract(self) -> None:
        state = CommandState()
        received: list[tuple[str, float]] = []
        server = create_http_server(
            "127.0.0.1",
            0,
            state,
            lambda command, value: received.append((command, value)),
        )
        thread = Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base_url = f"http://127.0.0.1:{server.server_port}/"

        try:
            self.assertEqual(
                _get(f"{base_url}?type=command&cmd=boom_angle&val=-30"),
                (200, "OK", "*"),
            )
            self.assertEqual(state.snapshot().joints_deg["boom"], -30.0)
            self.assertEqual(received, [("boom_angle", -30.0)])
            for joint in ("blade", "body", "swing", "boom", "arm", "bucket", "thumb"):
                with self.subTest(joint=joint):
                    self.assertEqual(
                        _get(f"{base_url}?type=command&cmd={joint}_angle&val=40")[0],
                        200,
                    )
                    self.assertEqual(received[-1], (f"{joint}_angle", 40.0))
                    self.assertEqual(
                        _get(f"{base_url}?type=command&cmd={joint}&val=40")[0],
                        400,
                    )
            _get(f"{base_url}?type=command&cmd=boom_angle&val=-30")


            self.assertEqual(
                _get(f"{base_url}?type=command&cmd=boom_up&val=-10"),
                (200, "OK", "*"),
            )
            self.assertEqual(state.snapshot().joints_deg["boom"], -30.0)
            self.assertEqual(received[-1], ("boom_up", -10.0))

            self.assertEqual(
                _get(f"{base_url}?type=command&cmd=bucket_scoop&val=5"),
                (200, "OK", "*"),
            )
            self.assertEqual(received[-1], ("bucket_scoop", 5.0))

            self.assertEqual(
                _get(f"{base_url}?type=command&cmd=swing_left&val=-10"),
                (200, "OK", "*"),
            )
            self.assertEqual(received[-1], ("swing_left", -10.0))

            self.assertEqual(
                _get(f"{base_url}?type=command&cmd=boom_speed&val=5"),
                (200, "OK", "*"),
            )
            self.assertEqual(
                state.snapshot().joint_speed_limits_deg_s["boom"], 5.0
            )
            self.assertEqual(received[-1], ("boom_speed", 5.0))

            self.assertEqual(
                _get(f"{base_url}?type=command&cmd=parallel&val=1"),
                (200, "OK", "*"),
            )
            self.assertEqual(
                state.snapshot().execution_mode, ExecutionMode.PARALLEL
            )
            self.assertEqual(received[-1], ("parallel", 1.0))

            self.assertEqual(
                _get(f"{base_url}?type=command&cmd=serial&val=0"),
                (200, "OK", "*"),
            )
            self.assertEqual(state.snapshot().execution_mode, ExecutionMode.SERIAL)
            self.assertEqual(received[-1], ("serial", 0.0))

            status, body, _ = _get(
                f"{base_url}?type=command&cmd=boom_speed&val=-1"
            )
            self.assertEqual(
                (status, body), (400, "joint speed must be non-negative")
            )

            status, body, _ = _get(
                f"{base_url}?type=command&cmd=boom_speed&val=5e-324"
            )
            self.assertEqual(
                (status, body),
                (400, "joint speed would exceed ROS Duration range"),
            )

            status, body, _ = _get(
                f"{base_url}?type=command&cmd=boom_speed&val=1e-100"
            )
            self.assertEqual(
                (status, body),
                (400, "joint speed would exceed ROS Duration range"),
            )

            result = _get(f"{base_url}?type=reporter&cmd=boom_angle&val=0")
            self.assertEqual(result, (400, "type=command is required", "*"))

            status, body, _ = _get(
                f"{base_url}?type=command&cmd=boom_angle&val=nan"
            )
            self.assertEqual((status, body), (400, "val must be a finite number"))

            status, body, _ = _get(
                f"{base_url}?type=command&cmd=unknown&val=1"
            )
            self.assertEqual((status, body), (400, "unknown cmd: unknown"))

            status, body, _ = _get(
                f"{base_url}?type=command&cmd=swing_right&val=180"
            )
            self.assertEqual(
                (status, body),
                (400, "relative swing angle must be less than 180 degrees"),
            )

            status, body, _ = _get(
                f"{base_url}?type=command&cmd=end_scoop&val=5"
            )
            self.assertEqual((status, body), (400, "unknown cmd: end_scoop"))

            status, body, _ = _get(
                f"{base_url}?type=command&cmd=base_left&val=10"
            )
            self.assertEqual((status, body), (400, "unknown cmd: base_left"))

            status, body, _ = _get(f"{base_url}?type=command&cmd=boom_angle")
            self.assertEqual((status, body), (400, "cmd and val are required"))

            status, body, _ = _get(
                f"{base_url}?type=command&cmd=boom_angle&val=48&val=-83"
            )
            self.assertEqual(
                (status, body), (400, "cmd and val must appear once")
            )

            status, body, _ = _get(
                f"{base_url}?type=command&cmd=boom_angle&val=1&extra=value"
            )
            self.assertEqual(
                (status, body),
                (400, "only type, cmd and val are allowed"),
            )

            overflow_state = CommandState(
                drive_linear_scale=1e308,
                drive_angular_scale=1e308,
            )
            overflow_state.apply_command("left_crawler_speed", 2.0)
            server.RequestHandlerClass.state = overflow_state
            status, body, _ = _get(
                f"{base_url}?type=command&cmd=right_crawler_speed&val=2"
            )
            self.assertEqual(
                (status, body),
                (400, "command would produce a non-finite value"),
            )
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)

    def test_idle_connection_does_not_block_shutdown(self) -> None:
        server = create_http_server("127.0.0.1", 0, CommandState())
        server_thread = Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        client = socket.create_connection(
            ("127.0.0.1", server.server_port), timeout=2.0
        )
        client.sendall(b"G")
        time.sleep(0.05)

        shutdown_thread = Thread(target=server.shutdown, daemon=True)
        shutdown_thread.start()
        shutdown_thread.join(timeout=2.0)

        try:
            self.assertFalse(shutdown_thread.is_alive())
        finally:
            client.close()
            server.server_close()
            server_thread.join(timeout=2.0)


if __name__ == "__main__":
    unittest.main()
