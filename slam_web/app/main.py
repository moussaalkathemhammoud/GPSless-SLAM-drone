from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pathlib import Path

from .slam_process import start_slam, stop_slam
from .slam_routes import router as slam_router
from .udp_listener import start_udp_listener

app = FastAPI()
app.include_router(slam_router)

# ---------- Path-safe frontend mounting ----------
BASE_DIR = Path(__file__).resolve().parent.parent  # slam_web/

FRONTEND_DIR = BASE_DIR / "frontend"

app.mount(
    "/frontend",
    StaticFiles(directory=FRONTEND_DIR),
    name="frontend"
)

@app.get("/")
def index():
    return FileResponse(FRONTEND_DIR / "index.html")

# ---------- SLAM endpoints ----------
@app.on_event("startup")
def startup():
    start_udp_listener()
