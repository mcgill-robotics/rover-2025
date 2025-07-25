import asyncio
import subprocess
from aiohttp import web
from dotenv import load_dotenv
import os
import re

load_dotenv()

# Tracks active GStreamer processes keyed by device
process_map = {}

@web.middleware
async def cors_middleware(request, handler):
    response = await handler(request)
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
    return response

async def handle_options(request):
    return web.Response(status=200)

def build_gst_pipeline(device_path):
    return [
        "gst-launch-1.0",
        "v4l2src", f"device={device_path}",
        "!", "video/x-raw,width=640,height=480,framerate=20/1",
        "!", "nvvidconv",
        "!", "x264enc", "bitrate=512", "tune=zerolatency",
        "!", "h264parse",
        "!", "rtph264pay", "config-interval=1", "pt=96",
        "!", "udpsink", f"host={os.getenv('HOST')}", f"port={os.getenv('PORT')}"
    ]

def resolve_device_path_from_name(camera_name):
    """Returns the first device path associated with a given camera name."""
    try:
        result = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True)
        output = result.stdout.strip().splitlines()

        current_name = None
        for line in output:
            if not line.startswith("\t"):
                current_name = re.sub(r"\s(.):?$", "", line.strip())
            elif current_name == camera_name and "/dev/video" in line:
                return line.strip()
    except Exception as e:
        print(f"[ERROR] Failed to resolve device: {e}")
    return None

async def start_stream(request):
    data = await request.json()
    camera_name = data.get("camera")
    if not camera_name:
        return web.json_response({"error": "Missing 'camera'"}, status=400)

    print(f'Name {camera_name}')
    device_path = resolve_device_path_from_name(camera_name)
    print(f'Path: {device_path}')
    if not device_path:
        return web.json_response({"error": f"Camera '{camera_name}' not found"}, status=404)

    # Kill existing process for this device if running
    if device_path in process_map:
        process_map[device_path].terminate()
        await process_map[device_path].wait()

    pipeline = build_gst_pipeline(device_path)
    print(f"[JETSON] Starting stream: {' '.join(pipeline)}")
    proc = await asyncio.create_subprocess_exec(*pipeline)
    process_map[device_path] = proc

    return web.json_response({"status": "started", "device": device_path})

async def stop_all_streams(request):
    for proc in process_map.values():
        proc.terminate()
    await asyncio.gather(*(p.wait() for p in process_map.values()))
    process_map.clear()
    return web.json_response({"status": "all streams stopped"})

async def list_devices(request):
    """Returns all camera names and associated device paths."""
    try:
        result = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True)
        output = result.stdout.strip().splitlines()

        devices = []
        current = None

        for line in output:
            if not line.startswith("\t") and line.strip():  # Name line
                if current:
                    devices.append(current)
                name = re.sub(r"\s(.):?$", "", line.strip())
                current = {"name": name, "devices": []}
            elif line.startswith("\t") and current:  # Device path line
                dev_path = line.strip()
                if dev_path.startswith("/dev/video"):
                    current["devices"].append(dev_path)

        if current:
            devices.append(current)

        return web.json_response({"devices": devices})

    except Exception as e:
        return web.json_response({"error": f"Failed to list devices: {e}"}, status=500)

if __name__ == "__main__":
    app = web.Application(middlewares=[cors_middleware])
    app.router.add_post("/start-stream", start_stream)
    app.router.add_post("/stop-all", stop_all_streams)
    app.router.add_get("/list-devices", list_devices)
    app.router.add_options("/start-stream", handle_options)
    app.router.add_options("/stop-all", handle_options)
    app.router.add_options("/list-devices", handle_options)
    web.run_app(app, host="0.0.0.0", port=8000)
