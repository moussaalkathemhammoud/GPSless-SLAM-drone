#!/usr/bin/env python3
"""
Mission Database Module
=======================
SQLite database for persistent storage of SLAM UI command logging.

Tables:
- ui_commands: User action audit trail from FastAPI endpoints
- system_logs: Process events and debugging

Author: CSC499 Capstone Team
Date: January 2026
"""

import sqlite3
import json
import logging
from datetime import datetime
from pathlib import Path
from typing import List, Dict, Optional
from dataclasses import dataclass
import threading

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class UICommand:
    """UI user command record"""
    timestamp: float
    command_type: str  # "START_SLAM", "STOP_SLAM", "GOTO", etc.
    args_json: str  # JSON serialized arguments
    http_status: int  # HTTP response code
    response_json: str  # JSON response from backend
    error_msg: Optional[str] = None


class MissionDatabase:
    """
    SQLite database for SLAM UI command logging.
    Thread-safe implementation.
    """

    def __init__(self, db_path: str = None):
        """
        Initialize database connection.
        
        Args:
            db_path: Path to SQLite database file. Default: slam_web/data/mission.db
        """
        if db_path is None:
            # Default to slam_web/data/mission.db (relative to this file)
            db_path = str(Path(__file__).parent.parent / "data" / "mission.db")

        self.db_path = db_path
        Path(db_path).parent.mkdir(parents=True, exist_ok=True)
        
        self.lock = threading.RLock()
        self.connection = sqlite3.connect(self.db_path, check_same_thread=False)
        self.connection.row_factory = sqlite3.Row
        
        logger.info(f"[SLAMDb] Database initialized: {self.db_path}")
        
        self._create_tables()

    def _create_tables(self):
        """Create database tables if they don't exist"""
        cursor = self.connection.cursor()

        # UI commands table: log of user actions from web interface
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS ui_commands (
                command_id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL NOT NULL,
                command_type TEXT NOT NULL,
                args_json TEXT,
                http_status INTEGER,
                response_json TEXT,
                error_msg TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)

        # System logs table: process events and debugging
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS system_logs (
                log_id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL NOT NULL,
                level TEXT NOT NULL,
                component TEXT NOT NULL,
                message TEXT NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)

        self.connection.commit()
        logger.info("[SLAMDb] Tables created/verified")

    # ===== UI Commands Logging =====

    def log_ui_command(self, command: UICommand) -> bool:
        """Log a UI user command to database"""
        try:
            with self.lock:
                cursor = self.connection.cursor()
                cursor.execute("""
                    INSERT INTO ui_commands (
                        timestamp, command_type, args_json, http_status, 
                        response_json, error_msg
                    ) VALUES (?, ?, ?, ?, ?, ?)
                """, (
                    command.timestamp, command.command_type, command.args_json,
                    command.http_status, command.response_json, command.error_msg
                ))
                self.connection.commit()
                logger.info(f"[SLAMDb] UI command logged: {command.command_type}")
                return True
        except Exception as e:
            logger.error(f"[SLAMDb] Error logging UI command: {e}")
            return False

    def get_ui_commands(self, limit: int = 100) -> List[Dict]:
        """Retrieve recent UI commands"""
        try:
            with self.lock:
                cursor = self.connection.cursor()
                cursor.execute("""
                    SELECT * FROM ui_commands 
                    ORDER BY timestamp DESC 
                    LIMIT ?
                """, (limit,))
                rows = cursor.fetchall()
                return [dict(row) for row in rows]
        except Exception as e:
            logger.error(f"[SLAMDb] Error retrieving UI commands: {e}")
            return []

    def get_ui_commands_by_type(self, command_type: str, limit: int = 50) -> List[Dict]:
        """Retrieve UI commands by type (e.g., 'GOTO', 'START_SLAM')"""
        try:
            with self.lock:
                cursor = self.connection.cursor()
                cursor.execute("""
                    SELECT * FROM ui_commands 
                    WHERE command_type = ? 
                    ORDER BY timestamp DESC 
                    LIMIT ?
                """, (command_type, limit))
                rows = cursor.fetchall()
                return [dict(row) for row in rows]
        except Exception as e:
            logger.error(f"[SLAMDb] Error retrieving UI commands by type: {e}")
            return []

    # ===== System Logs =====

    def log_event(self, level: str, component: str, message: str) -> bool:
        """Log system event"""
        try:
            with self.lock:
                cursor = self.connection.cursor()
                cursor.execute("""
                    INSERT INTO system_logs (timestamp, level, component, message)
                    VALUES (?, ?, ?, ?)
                """, (datetime.now().timestamp(), level, component, message))
                self.connection.commit()
                logger.info(f"[SLAMDb] Event logged: {component} - {message}")
                return True
        except Exception as e:
            logger.error(f"[SLAMDb] Error logging event: {e}")
            return False

    def get_logs(self, limit: int = 100) -> List[Dict]:
        """Get recent system logs"""
        try:
            with self.lock:
                cursor = self.connection.cursor()
                cursor.execute("""
                    SELECT * FROM system_logs 
                    ORDER BY timestamp DESC 
                    LIMIT ?
                """, (limit,))
                rows = cursor.fetchall()
                return [dict(row) for row in rows]
        except Exception as e:
            logger.error(f"[SLAMDb] Error retrieving logs: {e}")
            return []

    def close(self):
        """Close database connection"""
        if self.connection:
            self.connection.close()
            logger.info("[SLAMDb] Database connection closed")


# Singleton instance
_db_instance: Optional[MissionDatabase] = None


def get_database(db_path: str = None) -> MissionDatabase:
    """Get or create database singleton"""
    global _db_instance
    if _db_instance is None:
        _db_instance = MissionDatabase(db_path)
    return _db_instance
