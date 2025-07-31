from fastapi import APIRouter
from fastapi.responses import JSONResponse, FileResponse
import asyncio
import socket

router = APIRouter()

CAM_HOST = "127.0.0.1"
CAM_PORT = 9001

CAM_CMD_MAP = {
    "status": "PING",
    "start" : "RUN",
    "stop"  : "STOP",
    "qr"    : "QR",
    "lcam"  : "LCAM",
    "rcam"  : "RCAM",
}

def send_cam(cmd: str, timeout=150) -> str:
    with socket.create_connection((CAM_HOST, CAM_PORT), timeout=timeout) as s:
        s.sendall((cmd+"\n").encode())
        data = s.recv(4096)
    return data.decode(errors="ignore").strip()

@router.get("/camera_test")
async def camera_test_page():
    return FileResponse("/opt/projects/robotour/server/static/camera_test.html")

@router.get("/camera/{action}")
async def camera_action(action: str):
    action = action.lower()
    if action not in CAM_CMD_MAP:
        return JSONResponse(status_code=400, content={"error":"bad action"})
    try:
        resp = await asyncio.to_thread(send_cam, CAM_CMD_MAP[action])
        return {"action":action, "response":resp}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error":str(e)})
