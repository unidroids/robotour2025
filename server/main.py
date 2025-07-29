import subprocess
from fastapi import FastAPI
#from fastapi.responses import JSONResponse
#from fastapi import Request, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

from fastapi.responses import JSONResponse
from fastapi.requests import Request
from fastapi.exception_handlers import http_exception_handler
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
import logging

import socket
import platform
import os

import asyncio
import datetime

from starlette.middleware.base import BaseHTTPMiddleware

class NoCacheMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request, call_next):
        response = await call_next(request)
        # globální „no-cache“ hlavičky
        response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        response.headers["Pragma"]        = "no-cache"
        response.headers["Expires"]       = "0"
        return response

app = FastAPI()

app.add_middleware(NoCacheMiddleware)        

app.mount("/static", StaticFiles(directory="/opt/projects/robotour/server/static", html=True, check_dir=True), name="static")




# Přidej 404 handler
@app.exception_handler(StarletteHTTPException)
async def custom_http_exception_handler(request: Request, exc: StarletteHTTPException):
    if exc.status_code == 404:
        print(f"[404] Not found: {request.url}")
        return JSONResponse(status_code=404, content={"error": "Not found", "url": str(request.url)})
    return await http_exception_handler(request, exc)

@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request: Request, exc: RequestValidationError):
    print(f"[422] Validation error at {request.url}")
    return JSONResponse(status_code=422, content={"error": "Invalid request", "details": exc.errors()})

@app.get("/")
async def root():
    #return JSONResponse(content={"message": "Hello from FastAPI!"})
    return FileResponse("/opt/projects/robotour/server/static/service.html")
    #return FileResponse("/opt/projects/robotour/server/static/view_test.html")
    #return FileResponse("/opt/projects/robotour/server/static/camera_test.html")
    

@app.get("/favicon.ico")
async def favicon():
    return FileResponse("/opt/projects/robotour/server/static/favicon.ico")

@app.post("/shutdown")
async def shutdown(request: Request):
    # body = await request.json()
    # token = body.get("token")
    # if token != SHUTDOWN_TOKEN:
    #     raise HTTPException(status_code=403, detail="Unauthorized")

    try:
        subprocess.Popen(["sudo", "/sbin/poweroff"])
        return {"status": "shutdown initiated"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/info")
async def info():
    hostname = socket.gethostname()
    try:
        ip = socket.gethostbyname(hostname)
    except:
        ip = "unknown"

    return {
        "hostname": hostname,
        "ip": ip,
        "system": platform.system(),
        "release": platform.release(),
        "cpu_count": os.cpu_count()
    }

### LIDAR TEST ###

@app.get("/lidar_test")
async def lidar_test_page():
    return FileResponse("/opt/projects/robotour/server/static/lidar_test.html")

LIDAR_HOST = "127.0.0.1"
LIDAR_PORT = 9002

LIDAR_CMD_MAP = {
    "status"    : "PING",   # služba vrátí PONG
    "start"     : "START",
    "stop"      : "STOP",
    "distance"  : "DISTANCE",
}

def send_lidar(cmd: str, timeout=150) -> str:
    with socket.create_connection((LIDAR_HOST, LIDAR_PORT), timeout=timeout) as s:
        s.sendall((cmd+"\n").encode())
        data = s.recv(4096)
    return data.decode(errors="ignore").strip()

@app.get("/lidar/{action}")
async def lidar_action(action: str):
    action = action.lower()
    if action not in LIDAR_CMD_MAP:
        return JSONResponse(status_code=400, content={"error":"bad action"})
    try:
        # spustíme blokující volání v thread-poolu, aby neblokovalo event-loop
        resp = await asyncio.to_thread(send_lidar, LIDAR_CMD_MAP[action])
        return {"action":action, "response":resp}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error":str(e)})    

### CAMERA TEST ###

@app.get("/camera_test")
async def camera_test_page():
    return FileResponse("/opt/projects/robotour/server/static/camera_test.html")

CAM_HOST = "127.0.0.1"
CAM_PORT = 9001

CAM_CMD_MAP = {
    "status": "PING",   # kamerová služba vrátí PONG
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


@app.get("/camera/{action}")
async def camera_action(action: str):
    action = action.lower()
    if action not in CAM_CMD_MAP:
        return JSONResponse(status_code=400, content={"error":"bad action"})
    try:
        # spustíme blokující volání v thread-poolu, aby neblokovalo event-loop
        resp = await asyncio.to_thread(send_cam, CAM_CMD_MAP[action])
        return {"action":action, "response":resp}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error":str(e)})    


