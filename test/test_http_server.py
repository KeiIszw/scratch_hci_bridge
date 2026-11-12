import math
import socket
import time
import unittest
from threading import Thread
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from scratch_hci_bridge.command_state import CommandState
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
