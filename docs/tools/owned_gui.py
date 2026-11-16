"""Close only Gazebo and RViz windows belonging to one generated experiment partition."""
import ctypes
import os
from pathlib import Path
import re
import subprocess
import time


def close_gazebo_gui(partition):
    if not re.fullmatch(r"(teaching_|mini_slam_accept_)[0-9a-f]{10}", partition):
        raise ValueError("An exact generated experiment partition is required")
    if not os.environ.get("DISPLAY"):
        return 0
    try:
        tree = subprocess.check_output(["xwininfo", "-root", "-tree"],
                                       stderr=subprocess.DEVNULL, text=True)
    except (OSError, subprocess.CalledProcessError):
        return 0
    windows = []
    for line in tree.splitlines():
        is_gazebo = '"Gazebo Sim"' in line and '"Gazebo GUI"' in line
        is_rviz = ' - RViz"' in line
        if not (is_gazebo or is_rviz):
            continue
        identifier = re.search(r"0x[0-9a-f]+", line)
        if not identifier:
            continue
        try:
            prop = subprocess.check_output(
                ["xprop", "-id", identifier.group(), "_NET_WM_PID"],
                stderr=subprocess.DEVNULL, text=True)
            pid = int(re.search(r"=\s*(\d+)", prop).group(1))
            environment = Path(f"/proc/{pid}/environ").read_bytes().split(b"\0")
            if ("GZ_PARTITION=" + partition).encode() in environment:
                windows.append(int(identifier.group(), 16))
        except (OSError, subprocess.CalledProcessError, AttributeError):
            continue
    if not windows:
        return 0
    x11 = ctypes.CDLL("libX11.so.6")
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XOpenDisplay.restype = ctypes.c_void_p
    display = x11.XOpenDisplay(None)
    if not display:
        return 0
    class Data(ctypes.Union):
        _fields_ = [("l", ctypes.c_long * 5)]
    class ClientMessage(ctypes.Structure):
        _fields_ = [("type", ctypes.c_int), ("serial", ctypes.c_ulong),
                    ("send_event", ctypes.c_int), ("display", ctypes.c_void_p),
                    ("window", ctypes.c_ulong), ("message_type", ctypes.c_ulong),
                    ("format", ctypes.c_int), ("data", Data)]
    class Event(ctypes.Union):
        _fields_ = [("client", ClientMessage), ("padding", ctypes.c_long * 24)]
    x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    x11.XInternAtom.restype = ctypes.c_ulong
    x11.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int,
                              ctypes.c_long, ctypes.POINTER(Event)]
    x11.XFlush.argtypes = [ctypes.c_void_p]
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    try:
        protocols = x11.XInternAtom(display, b"WM_PROTOCOLS", 0)
        delete = x11.XInternAtom(display, b"WM_DELETE_WINDOW", 0)
        for window in windows:
            event = Event()
            event.client.type = 33
            event.client.send_event = 1
            event.client.display = display
            event.client.window = window
            event.client.message_type = protocols
            event.client.format = 32
            event.client.data.l[0] = delete
            x11.XSendEvent(display, window, 0, 0, ctypes.byref(event))
        x11.XFlush(display)
        time.sleep(0.3)
    finally:
        x11.XCloseDisplay(display)
    return len(windows)
