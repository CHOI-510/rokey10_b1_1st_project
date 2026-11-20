"""Check shared HTTP/WebSocket ports before starting robot nodes."""
import errno
import socket


def ensure_ports_available(endpoints):
    """Raise an actionable error if a requested local endpoint is occupied."""
    seen = {}
    for name, port in endpoints:
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise RuntimeError(f'{name} 포트 범위 오류: {port}')
        if port in seen:
            raise RuntimeError(f'{name}와 {seen[port]}가 같은 포트 {port}를 사용합니다.')
        seen[port] = name
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.bind(('127.0.0.1', port))
            except OSError as exc:
                if exc.errno != errno.EADDRINUSE:
                    raise
                raise RuntimeError(
                    f'{name} 포트 127.0.0.1:{port}가 이미 사용 중입니다. '
                    '기존 system.launch.py(fake/real) 또는 별도 서버를 먼저 종료하세요. '
                    '중복 관제 실행을 막기 위해 로봇 노드를 시작하지 않습니다. '
                    'rosbridge만 별도로 실행 중이라면 rosbridge:=false를 사용하세요.'
                ) from exc
