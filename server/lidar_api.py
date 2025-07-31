from fastapi import APIRouter
from fastapi.responses import JSONResponse, FileResponse
import asyncio
import socket
from sse_starlette.sse import EventSourceResponse

router = APIRouter()

LIDAR_HOST = "127.0.0.1"
LIDAR_PORT = 9002

LIDAR_CMD_MAP = {
    "status"    : "PING",
    "start"     : "START",
    "stop"      : "STOP",
    "distance"  : "DISTANCE",
}

def send_lidar(cmd: str, timeout=150) -> str:
    with socket.create_connection((LIDAR_HOST, LIDAR_PORT), timeout=timeout) as s:
        s.sendall((cmd + "\n").encode())
        data = s.recv(4096)
    return data.decode(errors="ignore").strip()

@router.get("/lidar_test")
async def lidar_test_page():
    return FileResponse("/opt/projects/robotour/server/static/lidar_test.html")

@router.get("/lidar/{action}")
async def lidar_action(action: str):
    action = action.lower()
    if action not in LIDAR_CMD_MAP:
        return JSONResponse(status_code=400, content={"error":"bad action"})
    try:
        resp = await asyncio.to_thread(send_lidar, LIDAR_CMD_MAP[action])
        return {"action":action, "response":resp}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error":str(e)})

@router.get("/lidar_stream")
async def lidar_stream():
    async def event_generator():
        try:
            while True:
                resp = await asyncio.to_thread(send_lidar, "DISTANCE")
                yield f"data: {resp}\n\n"
                await asyncio.sleep(0.2)
        except asyncio.CancelledError:
            print("🛑 SSE klient odpojen")
            return
    return EventSourceResponse(event_generator())
