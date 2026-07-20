"""
Centralized debug utility for the SLAM simulator.

Toggle the DEBUG flag below to enable/disable all debug output across the program.
When DEBUG is True, debug messages are printed to the console.
When DEBUG is False, all debug calls become no-ops (zero overhead).
"""

# ==========================================
# MASTER TOGGLE - Change this to True/False
# ==========================================
DEBUG = True

# Optional: Also write debug output to a file
DEBUG_TO_FILE = False
DEBUG_LOG_FILE = "debug.log"

# Optional: Show a visual debug overlay in the pygame window
# (only takes effect if DEBUG is True)
DEBUG_OVERLAY = True


def _write(msg):
    """Internal helper: prints to console and optionally to file."""
    print(msg)
    if DEBUG_TO_FILE:
        try:
            with open(DEBUG_LOG_FILE, "a") as f:
                f.write(msg + "\n")
        except Exception:
            pass  # Don't crash the program if logging fails


def log(category, msg):
    """Log a debug message with a category tag. No-op when DEBUG is False."""
    if DEBUG:
        _write(f"[DEBUG][{category}] {msg}")


def log_robot(msg):
    log("ROBOT", msg)


def log_map(msg):
    log("MAP", msg)


def log_slam(msg):
    log("SLAM", msg)


def log_pathfinder(msg):
    log("PATHFINDER", msg)


def log_controller(msg):
    log("CONTROLLER", msg)


def log_main(msg):
    log("MAIN", msg)


def log_sensor(msg):
    log("SENSOR", msg)
