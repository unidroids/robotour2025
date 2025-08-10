from fastapi import APIRouter
from fastapi.responses import JSONResponse
import socket, json

router = APIRouter(prefix="/api/gamepad", tags=["gamepad"])
HOST = "127.0.0.1"
PORT = 9005
TIMEOUT = 1.0


def _send(cmd: str):
    with socket.create_connection((HOST, PORT), timeout=TIMEOUT) as s:
        s.sendall((cmd + "
").encode("utf-8"))
        data = s.recv(65536)
    try:
        return json.loads(data.decode("utf-8").strip())
    except Exception:
        return {"raw": data.decode("utf-8", errors="replace")}

@router.get("/status")
def status():
    return JSONResponse(_send("STATUS"))

@router.post("/wheels")
@router.get("/wheels")
def wheels():
    return JSONResponse(_send("WHEELS"))

@router.post("/drive")
@router.get("/drive")
def drive():
    return JSONResponse(_send("DRIVE"))

@router.post("/stop")
@router.get("/stop")
def stop():
    return JSONResponse(_send("STOP"))