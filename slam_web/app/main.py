from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pathlib import Path

from .slam_process import start_slam, stop_slam
from .slam_routes import router as slam_router
from .mission_routes import router as mission_router
from .udp_listener import start_udp_listener
from .drone_process import stop_drone
from .static_geo_init import init_static_origin_at_spawn

app = FastAPI()
app.include_router(slam_router)
app.include_router(mission_router)

# ---------- Path-safe frontend mounting ----------
BASE_DIR = Path(__file__).resolve().parent.parent  # slam_web/
REPO_DIR = Path(__file__).resolve().parents[2]    # repo root (contains ./static)

FRONTEND_DIR = BASE_DIR / "frontend"

app.mount(
    "/frontend",
    StaticFiles(directory=FRONTEND_DIR),
    name="frontend"
)

# ---------- Static assets (map image, etc.) ----------
# NOTE: This is additive and does not change any existing API endpoints.
STATIC_DIR = REPO_DIR / "static"
if STATIC_DIR.exists():
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

@app.get("/")
def index():
    return FileResponse(FRONTEND_DIR / "index.html")

# ---------- SLAM endpoints ----------
@app.on_event("startup")
def startup():
    start_udp_listener()
    # Optional: lock fake GPS origin to the drone's spawn pose (no interactive calibration).
    init_static_origin_at_spawn()

@app.on_event("shutdown")
def shutdown():
    # Ensure child processes terminate with the app.
    stop_drone()
    stop_slam()
