from fastapi import APIRouter
from pydantic import BaseModel
from .slam_process import start_slam, stop_slam, slam_status
from .udp_listener import latest_pose, trajectory
from .drone_process import start_drone, stop_drone
from .kf_reader import read_keyframes
from .drone_motion import goto_target

router = APIRouter(prefix="/slam", tags=["SLAM"])

@router.post("/start")
def start():
    return start_slam()


@router.post("/stop")
def stop():
    return stop_slam()

@router.get("/status")
def status():
    return slam_status()

@router.get("/pose")
def pose():
    return latest_pose

@router.get("/trajectory")
def traj():
    return read_keyframes()

@router.get("/ready")
def ready():
    return {
        "slam_running": True,
        "udp_listener": True,
        "pose": latest_pose
    }

@router.post("/start")
def start():
    start_slam()
    start_drone()
    return {
        "ok": True,
        "msg": "SLAM + Drone started"
    }

@router.post("/stop")
def stop():
    stop_drone()
    stop_slam()
    return {
        "ok": True,
        "msg": "SLAM + Drone stopped"
    }

@router.get("/trajectory")
def get_trajectory():
    return trajectory


class GotoCmd(BaseModel):
    x: float
    y: float
    z: float

@router.post("/goto")
def goto(cmd: GotoCmd):
    goto_target(cmd.x, cmd.y, cmd.z)
    return {"ok": True, "msg": "Target accepted"}
