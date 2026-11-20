import socket
import unittest

from box_bringup.runtime_guard import ensure_ports_available


class RuntimeGuardTests(unittest.TestCase):
    def test_occupied_port_stops_startup_with_actionable_message(self):
        with socket.socket() as server:
            server.bind(('127.0.0.1', 0))
            server.listen()
            port = server.getsockname()[1]
            with self.assertRaisesRegex(RuntimeError, f'127.0.0.1:{port}.*이미 사용 중'):
                ensure_ports_available([('rosbridge WebSocket', port)])

    def test_released_port_is_allowed(self):
        with socket.socket() as server:
            server.bind(('127.0.0.1', 0))
            port = server.getsockname()[1]
        ensure_ports_available([('DB HTTP', port)])

    def test_invalid_port_is_rejected(self):
        for port in (0, 65536, True):
            with self.subTest(port=port), self.assertRaisesRegex(RuntimeError, '포트 범위 오류'):
                ensure_ports_available([('rosbridge WebSocket', port)])

    def test_duplicate_requested_ports_are_rejected(self):
        with self.assertRaisesRegex(RuntimeError, '같은 포트'):
            ensure_ports_available([('DB HTTP', 8765), ('rosbridge WebSocket', 8765)])


if __name__ == '__main__':
    unittest.main()
