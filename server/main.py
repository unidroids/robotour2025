from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from fastapi.requests import Request
from fastapi.exception_handlers import http_exception_handler
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
import subprocess
import socket
import platform
import os

from camera_api import router as camera_router
from lidar_api import router as lidar_router
from drive_api import router as drive_router
from journey_api import router as journey_router
from gamepad_api import router as gamepad_router

from nocache import NoCacheMiddleware

app = FastAPI()

app.add_middleware(NoCacheMiddleware)
app.mount("/static", StaticFiles(directory="/opt/projects/robotour/server/static", html=True, check_dir=True), name="static")
app.include_router(camera_router)
app.include_router(lidar_router)
app.include_router(drive_router)
app.include_router(journey_router)
app.include_router(gamepad_router)

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
    return FileResponse("/opt/projects/robotour/server/static/service.html")

@app.get("/favicon.ico")
async def favicon():
    return FileResponse("/opt/projects/robotour/server/static/favicon.ico")

@app.post("/shutdown")
async def shutdown(request: Request):
    try:
        subprocess.Popen(["sudo", "/sbin/poweroff"])
        return {"status": "shutdown initiated"}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error": str(e)})

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
