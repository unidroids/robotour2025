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

@app.get("/lidar_test")
async def lidar_test():
    log_path = "/data/logs/fastapi/lidar_test.log"
    binary = "/opt/projects/robotour/unitree_lidar_sdk/bin/example_lidar_udp"
    start_time = datetime.datetime.now().isoformat()

    async def run_lidar():
        try:
            with open(log_path, "a") as f:
                f.write(f"\n[{start_time}] Spouštím {binary}...\n")
            proc = await asyncio.create_subprocess_exec(
                binary,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            stdout, stderr = await proc.communicate()
            end_time = datetime.datetime.now().isoformat()

            with open(log_path, "a") as f:
                f.write(f"[{end_time}] Dokončeno, návratový kód {proc.returncode}\n")
                f.write(stdout.decode() if stdout else "")
                f.write(stderr.decode() if stderr else "")
        except Exception as e:
            with open(log_path, "a") as f:
                f.write(f"[chyba] {str(e)}\n")

    # Spusť proces na pozadí (nebude čekat na dokončení)
    asyncio.create_task(run_lidar())

    return {"status": "lidar test started", "path": binary}

@app.get("/camera_test")
async def camera_test_page():
    return FileResponse("/opt/projects/robotour/server/static/camera_test.html")


@app.get("/camera/{action}")
async def camera_action(action: str):
    action = action.lower()
    if action not in CMD_MAP:
        return JSONResponse(status_code=400, content={"error":"bad action"})
    try:
        # spustíme blokující volání v thread-poolu, aby neblokovalo event-loop
        resp = await asyncio.to_thread(send_cam, CMD_MAP[action])
        return {"action":action, "response":resp}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error":str(e)})    





CAM_HOST = "127.0.0.1"
CAM_PORT = 9001

CMD_MAP = {
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

