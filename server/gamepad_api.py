from fastapi import APIRouter
from fastapi.responses import JSONResponse, FileResponse
import asyncio
import socket
from sse_starlette.sse import EventSourceResponse

router = APIRouter()

GAMEPAD_HOST = "127.0.0.1"
GAMEPAD_PORT = 9005

GAMEPAD_CMD_MAP = {
    "status"    : "PING",
    "start"     : "START",
    "data"      : "DATA",
    "stop"      : "STOP",
}

def send_gamepad(cmd: str, timeout=3) -> str:
    with socket.create_connection((GAMEPAD_HOST, GAMEPAD_PORT), timeout=timeout) as s:
        s.sendall((cmd + "\n").encode())
        data = s.recv(4096)
    return data.decode(errors="ignore").strip()

@router.get("/gamepad_test")
async def gamepad_test_page():
    return FileResponse("/opt/projects/robotour/server/static/gamepad_test.html")

@router.get("/gamepad/{action}")
async def gamepad_action(action: str):
    action = action.lower()
    if action not in GAMEPAD_CMD_MAP:
        return JSONResponse(status_code=400, content={"error":"bad action"})
    try:
        resp = await asyncio.to_thread(send_gamepad, GAMEPAD_CMD_MAP[action])
        return {"action":action, "response":resp}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error":str(e)})