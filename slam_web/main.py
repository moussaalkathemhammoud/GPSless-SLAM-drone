from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from app.slam_routes import router as slam_router
from app.mission_routes import router as mission_router
from app.drone_process import stop_drone
from app.slam_process import stop_slam
from app.udp_listener import start_udp_listener
from app.static_geo_init import init_static_origin_at_spawn



app = FastAPI()

app.include_router(slam_router)
app.include_router(mission_router)

@app.on_event("startup")
def startup():
    # Needed when running `slam_web/main.py` directly.
    start_udp_listener()
    init_static_origin_at_spawn()

@app.on_event("shutdown")
def shutdown():
    # Ensure child processes terminate with the app.
    stop_drone()
    stop_slam()

@app.get("/")
def root():
    return {"status": "FastAPI running"}
