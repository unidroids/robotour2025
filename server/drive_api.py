from fastapi import APIRouter
from fastapi.responses import JSONResponse, FileResponse
import asyncio
import socket
from sse_starlette.sse import EventSourceResponse

router = APIRouter()

DRIVE_HOST = "127.0.0.1"
DRIVE_PORT = 9003

DRIVE_CMD_MAP = {
    "status"    : "PING",
    "start"     : "START",
    "stop"      : "STOP",
    "break"     : "BREAK",
    "forward"   : "PWM 20 20",
    "backward"  : "PWM -20 -20",
}

def send_drive(cmd: str, timeout=20) -> str:
    with socket.create_connection((DRIVE_HOST, DRIVE_PORT), timeout=timeout) as s:
        s.sendall((cmd + "\n").encode())
        data = s.recv(4096)
    return data.decode(errors="ignore").strip()

@router.get("/drive_test")
async def drive_test_page():
    return FileResponse("/opt/projects/robotour/server/static/drive_test.html")

@router.get("/drive/{action}")
async def drive_action(action: str):
    action = action.lower()
    if action not in DRIVE_CMD_MAP:
        return JSONResponse(status_code=400, content={"error":"bad action"})
    try:
        resp = await asyncio.to_thread(send_drive, DRIVE_CMD_MAP[action])
        return {"action":action, "response":resp}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error":str(e)})

