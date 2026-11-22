"""FastAPI gateway: rosbridge robot commands/status and DB HTTP inventory."""
from contextlib import asynccontextmanager
from datetime import UTC, datetime
import json
import logging
import os
from threading import Lock
from typing import Literal
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from app.calibration import read_calibration, read_locations
from app.rosbridge import RosbridgeClient, RosbridgeError

logger = logging.getLogger("robot-api")
BACKEND = os.getenv("ROBOT_BACKEND", "rosbridge")
if BACKEND not in {"rosbridge", "box", "mock"}:
    raise ValueError("ROBOT_BACKEND must be rosbridge, box or mock")
BOX_URL = os.getenv("BOX_API_URL", "http://127.0.0.1:8765").rstrip("/")
BOX_TOKEN = os.getenv("BOX_START_TOKEN", "")
BOX_TIMEOUT = float(os.getenv("BOX_API_TIMEOUT", "8"))

bridge = RosbridgeClient(os.getenv("ROSBRIDGE_URL", "ws://127.0.0.1:9090"),
                         timeout=float(os.getenv("ROSBRIDGE_TIMEOUT", "8")),
                         service_ns=os.getenv("ROBOT_SERVICE_NS", "/dsr01/dsr_controller2"),
                         telemetry_period=float(os.getenv("ROBOT_TELEMETRY_PERIOD", "0.1")))


@asynccontextmanager
async def lifespan(_app: FastAPI):
    if BACKEND == "rosbridge":
        bridge.start()
    try:
        yield
    finally:
        if BACKEND == "rosbridge":
            bridge.close()


app = FastAPI(title="Robot Control API", version="0.3.0", lifespan=lifespan,
              description="Bridge to box_project's resident ROS 2 controller.")
app.add_middleware(
    CORSMiddleware,
    allow_origins=os.getenv("CORS_ORIGINS", "http://localhost:3000,http://127.0.0.1:3000").split(","),
    allow_methods=["GET", "POST"], allow_headers=["*"],
)


class RobotStatus(BaseModel):
    mode: Literal["rosbridge", "box", "mock"]
    state: Literal["idle", "countdown", "running", "stopped", "error", "unknown"]
    last_command: Literal["none", "start", "stop", "cancel", "start_now", "recover"] = "none"
    updated_at: datetime
    stage: str = ""
    job_status: str = ""
    message: str = ""
    robot_connected: bool = False
    robot_hardware_mode: str = ""
    robot_hint: str = ""
    connected: bool = True
    can_start: bool = False
    start_at: str | None = None
    started_at: str | None = None
    box_seq: int = 0
    job_id: str = ""
    ended_at: str | None = None
    stop: list[str] | None = None


def utc_now() -> datetime:
    return datetime.now(UTC)


_lock = Lock()
_mock_status = RobotStatus(mode="mock", state="idle", updated_at=utc_now(), can_start=True)


def box_request(path: str, method: str = "GET", body: dict | None = None) -> dict:
    headers = {"Accept": "application/json"}
    if BOX_TOKEN:
        headers["X-Box-Token"] = BOX_TOKEN
    if body is not None:
        headers["Content-Type"] = "application/json"
    request = Request(f"{BOX_URL}{path}", method=method, headers=headers,
                      data=json.dumps(body).encode() if body is not None else b"" if method == "POST" else None)
    try:
        with urlopen(request, timeout=BOX_TIMEOUT) as response:
            result = json.load(response)
        if not isinstance(result, dict):
            raise ValueError("Expected a JSON object")
        return result
    except HTTPError as exc:
        raise HTTPException(502, f"box_project 응답 오류 ({exc.code})") from exc
    except (URLError, TimeoutError, OSError, ValueError) as exc:
        logger.warning("box_project request failed: %s %s: %s", method, path, exc)
        detail = "box_project 연결 실패: ROS launch와 DB 노드(8765)를 확인하세요."
        if method == "POST":
            detail += " 명령 처리 여부를 확인할 수 없습니다. 상태를 확인하세요."
        raise HTTPException(503, detail) from exc


def box_status() -> RobotStatus:
    if BACKEND == "rosbridge":
        # UI follows the same live /control_status stream as monitor_gui.
        # Never block the live status response on the DB HTTP gateway.
        data = bridge.snapshot()
    else:
        data = box_request("/state")
    connected = data.get("control_connected") is True
    stage = str(data.get("stage") or "")
    job_status = str(data.get("status") or "")
    if not connected:
        state = "unknown"
    elif stage in {"BEFORE", "PREFLIGHT", "STANDBY"}:
        state = "countdown"
    elif stage == "READY":
        state = {"STOPPED": "stopped", "ERROR": "error"}.get(job_status, "idle")
    elif stage == "STOPPED":
        state = "stopped"
    elif stage == "ERROR":
        state = "error"
    elif stage in {"DONE", "CANCELED"}:
        state = "idle"
    else:
        state = "running"
    hardware_mode = str(data.get("robot_hardware_mode") or "")
    robot_hint = str(data.get("robot_hint") or "")
    if connected and data.get("robot_monitor") == "dsr" and not hardware_mode:
        robot_hint = "관제 상태에 robot_hardware_mode가 없습니다. 최신 관제의 상태 발행 코드를 확인하세요."
    return RobotStatus(
        mode="rosbridge" if BACKEND == "rosbridge" else "box", state=state, updated_at=utc_now(), stage=stage,
        job_status=job_status, connected=connected,
        robot_connected=(connected and data.get("robot_monitor") == "dsr"
                         and data.get("robot_hardware_mode") == "real"
                         and data.get("robot_connected") is True
                         and type(data.get("robot_state")) is int),
        can_start=connected and stage == "READY",
        robot_hardware_mode=hardware_mode, robot_hint=robot_hint,
        message=(str(data.get("reason") or robot_hint) if connected else "관제 응답 없음: ROS launch를 확인하세요."),
        start_at=data.get("start_at"), started_at=data.get("started_at"),
        box_seq=int(data.get("box_seq") or 0),
        job_id=str(data.get("job_id") or ""), ended_at=data.get("ended_at"), stop=data.get("stop"),
    )


@app.get("/health")
def health() -> dict[str, str]:
    return {"status": "ok", "backend": BACKEND}


@app.get("/robot/status", response_model=RobotStatus)
def get_robot_status() -> RobotStatus:
    if BACKEND in {"rosbridge", "box"}:
        return box_status()
    with _lock:
        return _mock_status.model_copy()


class RobotTelemetry(BaseModel):
    connected: bool
    source: str = "dsr_read_services"
    frame: Literal["BASE"] = "BASE"
    received_at: datetime | None = None
    age_ms: int | None = None
    tcp_connected: bool = False
    tcp_age_ms: int | None = None
    tcp_received_at: datetime | None = None
    joints_deg: tuple[float, float, float, float, float, float] | None = None
    tcp_mm: tuple[float, float, float] | None = None
    orientation_deg: tuple[float, float, float] | None = None
    message: str = ""


@app.get("/robot/telemetry", response_model=RobotTelemetry)
def get_robot_telemetry() -> RobotTelemetry:
    if BACKEND != "rosbridge":
        return RobotTelemetry(connected=False, message="실측 좌표에는 rosbridge real 모드 연결이 필요합니다.")
    return RobotTelemetry(**bridge.telemetry.snapshot())


def command_robot(command: Literal["start", "stop", "cancel", "start_now", "recover"]) -> RobotStatus:
    global _mock_status
    if BACKEND == "mock":
        with _lock:
            state = {"start": "running", "start_now": "running", "stop": "stopped", "cancel": "idle", "recover": "idle"}[command]
            _mock_status = RobotStatus(mode="mock", state=state, last_command=command,
                                       updated_at=utc_now(), can_start=state != "running")
            return _mock_status.model_copy()
    # Do not serialize stop behind status/start requests, and never retry a POST.
    if BACKEND == "rosbridge":
        service = "/emergency_stop" if command == "stop" else "/job_command"
        args = {"source": "ui"} if command == "stop" else {"command": command, "source": "ui"}
        try:
            response = bridge.call_service(service, args)
        except RosbridgeError as exc:
            raise HTTPException(503, str(exc)) from exc
        result = {"ok": response.get("success"), "message": response.get("message", "")}
    else:
        result = box_request({"start": "/start", "start_now": "/start_now", "stop": "/estop", "cancel": "/cancel", "recover": "/recover"}[command], "POST")
    if result.get("ok") is not True:
        raise HTTPException(409, result.get("message") or "관제가 명령을 거절했습니다.")
    # Acceptance is not a robot state change. The UI polls actual control state.
    return RobotStatus(mode="rosbridge" if BACKEND == "rosbridge" else "box", state="unknown", last_command=command,
                       updated_at=utc_now(), message=result.get("message") or "명령 접수 · 관제 상태 확인 중")


@app.post("/robot/start", response_model=RobotStatus)
def start_robot() -> RobotStatus:
    return command_robot("start")


@app.post("/robot/start_now", response_model=RobotStatus)
def start_robot_now() -> RobotStatus:
    return command_robot("start_now")


@app.post("/robot/stop", response_model=RobotStatus)
def stop_robot() -> RobotStatus:
    return command_robot("stop")


@app.post("/robot/cancel", response_model=RobotStatus)
def cancel_robot() -> RobotStatus:
    return command_robot("cancel")


class Inventory(BaseModel):
    a: int = Field(ge=0, le=6)
    b: int = Field(ge=0, le=6)
    c: int = Field(ge=0, le=6)
    d: int = Field(ge=0, le=6)
    u: int = Field(ge=0)


@app.get("/robot/inventory")
def get_inventory() -> dict:
    if BACKEND == "mock":
        return dict(a=0, b=0, c=0, d=0, u=0)
    return box_request("/inventory")


@app.post("/robot/inventory")
def set_inventory(inventory: Inventory) -> dict:
    if BACKEND == "mock":
        return {"ok": True, "message": "모의 재고 설정"}
    result = box_request("/inventory", "POST", {**inventory.model_dump(), "source": "ui"})
    if result.get("ok") is not True:
        raise HTTPException(409, result.get("message") or "재고 설정 거절")
    return result


@app.get("/robot/inventory/calibration")
def get_inventory_calibration() -> dict:
    return read_calibration()


@app.get("/robot/locations")
def get_robot_locations() -> list[dict]:
    return read_locations()


@app.post("/robot/recover", response_model=RobotStatus)
def recover_robot() -> RobotStatus:
    return command_robot("recover")
