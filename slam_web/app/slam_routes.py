from fastapi import APIRouter
from pydantic import BaseModel
import json
import time

# Import mission database (local module in slam_web/app/)
from .mission_database import get_database, UICommand

from .slam_process import start_slam, stop_slam, slam_status
from .udp_listener import latest_pose, trajectory
from .kf_reader import read_keyframes
from .drone_motion import goto_target

router = APIRouter(prefix="/slam", tags=["SLAM"])


def log_ui_command(command_type: str, args_dict: dict, http_status: int, response_dict: dict, error_msg: str = None):
    """Helper function to log UI commands to database"""
    try:
        db = get_database()
        cmd = UICommand(
            timestamp=time.time(),
            command_type=command_type,
            args_json=json.dumps(args_dict),
            http_status=http_status,
            response_json=json.dumps(response_dict),
            error_msg=error_msg
        )
        db.log_ui_command(cmd)
    except Exception as e:
        print(f"[slam_routes] Failed to log command {command_type}: {e}")


@router.post("/start")
def start():
    """Start ORB-SLAM3 and drone motion control"""
    try:
        result = start_slam()
        log_ui_command("START_SLAM", {}, 200, result)
        return result
    except Exception as e:
        error_response = {"ok": False, "error": str(e)}
        log_ui_command("START_SLAM", {}, 500, error_response, str(e))
        return error_response


@router.post("/stop")
def stop():
    """Stop ORB-SLAM3 and drone motion"""
    try:
        result = stop_slam()
        log_ui_command("STOP_SLAM", {}, 200, result)
        return result
    except Exception as e:
        error_response = {"ok": False, "error": str(e)}
        log_ui_command("STOP_SLAM", {}, 500, error_response, str(e))
        return error_response


@router.get("/status")
def status():
    """Get current SLAM process status"""
    return slam_status()


@router.get("/pose")
def pose():
    """Get latest SLAM pose from UDP listener"""
    return latest_pose

# @router.get("/trajectory")
# def traj():
#     return read_keyframes()
@router.get("/trajectory")
def get_trajectory():
    """Get SLAM trajectory (keyframes)"""
    return trajectory

@router.get("/ready")
def ready():
    """Health check endpoint"""
    return {
        "slam_running": True,
        "udp_listener": True,
        "pose": latest_pose
    }

# @router.post("/start")
# def start():
#     start_slam()
#     start_drone()
#     return {
#         "ok": True,
#         "msg": "SLAM + Drone started"
#     }

# @router.post("/stop")
# def stop():
#     stop_drone()
#     stop_slam()
#     return {
#         "ok": True,
#         "msg": "SLAM + Drone stopped"
#     }

# @router.get("/trajectory")
# def get_trajectory():
#     return trajectory


class GotoCmd(BaseModel):
    x: float
    y: float
    z: float


@router.post("/goto")
def goto(cmd: GotoCmd):
    """Send navigation target to drone"""
    try:
        goto_target(cmd.x, cmd.y, cmd.z)
        response = {
            "ok": True,
            "msg": "Target accepted",
            "target": {"x": cmd.x, "y": cmd.y, "z": cmd.z}
        }
        log_ui_command("GOTO", {"x": cmd.x, "y": cmd.y, "z": cmd.z}, 200, response)
        return response
    except Exception as e:
        error_response = {"ok": False, "error": str(e)}
        log_ui_command("GOTO", {"x": cmd.x, "y": cmd.y, "z": cmd.z}, 500, error_response, str(e))
        return error_response
