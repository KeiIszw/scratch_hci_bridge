import socket
import time
import unittest
from threading import Thread
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from scratch_hci_bridge.command_state import CommandState
from scratch_hci_bridge.http_server import create_http_server


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
                _get(f"{base_url}?type=command&cmd=boom&val=-30"),
                (200, "OK", "*"),
            )
            self.assertEqual(state.snapshot().joints_deg["boom"], -30.0)
            self.assertEqual(received, [("boom", -30.0)])

            status, body, _ = _get(
                f"{base_url}?type=command&cmd=boom_speed&val=1"
            )
            self.assertEqual((status, body), (400, "unknown cmd: boom_speed"))

            result = _get(f"{base_url}?type=reporter&cmd=boom&val=0")
            self.assertEqual(result, (400, "type=command is required", "*"))

            status, body, _ = _get(
                f"{base_url}?type=command&cmd=boom&val=nan"
            )
            self.assertEqual((status, body), (400, "val must be a finite number"))

            status, body, _ = _get(
                f"{base_url}?type=command&cmd=unknown&val=1"
            )
            self.assertEqual((status, body), (400, "unknown cmd: unknown"))

            status, body, _ = _get(f"{base_url}?type=command&cmd=boom")
            self.assertEqual((status, body), (400, "cmd and val are required"))

            status, body, _ = _get(
                f"{base_url}?type=command&cmd=boom&val=48&val=-83"
            )
            self.assertEqual(
                (status, body), (400, "cmd and val must appear once")
            )

            status, body, _ = _get(
                f"{base_url}?type=command&cmd=boom&val=1&extra=value"
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
