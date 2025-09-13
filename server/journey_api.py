from fastapi import APIRouter
from fastapi.responses import JSONResponse, FileResponse
import asyncio
import socket
from sse_starlette.sse import EventSourceResponse

router = APIRouter()

JOURNEY_HOST = "127.0.0.1"
JOURNEY_PORT = 9004

JOURNEY_CMD_MAP = {
    "status"    : "PING",
    "demo"      : "DEMO",
    "manual"    : "MANUAL",
    "point"     : "POINT",
    "auto"      : "AUTO",
    "stop"      : "STOP",
}

def send_journey(cmd: str, timeout=20) -> str:
    with socket.create_connection((JOURNEY_HOST, JOURNEY_PORT), timeout=timeout) as s:
        s.sendall((cmd + "\n").encode())
        data = s.recv(4096)
    return data.decode(errors="ignore").strip()

@router.get("/journey_test")
async def journey_test_page():
    return FileResponse("/opt/projects/robotour/server/static/journey_test.html")

@router.get("/journey/{action}")
async def journey_action(action: str):
    action = action.lower()
    if action not in JOURNEY_CMD_MAP:
        return JSONResponse(status_code=400, content={"error":"bad action"})
    try:
        resp = await asyncio.to_thread(send_journey, JOURNEY_CMD_MAP[action])
        return {"action":action, "response":resp}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error":str(e)})

