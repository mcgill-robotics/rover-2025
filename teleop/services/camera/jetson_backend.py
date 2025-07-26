import os
import re
import asyncio
import subprocess
import yaml

from aiohttp import web
from dotenv import load_dotenv

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

load_dotenv()
Gst.init(None)

HOST = os.getenv("HOST", "127.0.0.1")
PORT = int(os.getenv("PORT", "5000"))

# Tracks pipelines keyed by device path
pipeline_map = {}

@web.middleware
async def cors_middleware(request, handler):
    response = await handler(request)
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
    return response

async def handle_options(request):
    return web.Response(status=200)

def resolve_device_path_from_name(camera_name):
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

def load_camera_config(camera_name):
    path = os.path.join(os.path.dirname(__file__), "configs", f"{camera_name}.yaml")
    if not os.path.exists(path):
        raise FileNotFoundError(f"Missing config file for camera '{camera_name}' at {path}")
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def build_pipeline(config, device_path):
    # Optional format inclusion
    format_str = f"format={config['format']}," if config.get("format") else ""

    video_caps = (
        f"video/x-raw,{format_str}"
        f"width={config['width']},height={config['height']},"
        f"framerate={config['framerate']}/1"
    )

    pipeline_str = f"""
        v4l2src device={device_path} !
        {video_caps} !
        nvvidconv !
        {config['encoder']} name=encoder bitrate={config['bitrate']} tune={config['tune']} speed-preset={config['speed_preset']} !
        h264parse !
        rtph264pay config-interval=1 pt=96 !
        udpsink host={HOST} port={PORT}
    """

    return Gst.parse_launch(pipeline_str)


async def start_stream(request):
    data = await request.json()
    camera_name = data.get("camera")
    if not camera_name:
        return web.json_response({"error": "Missing 'camera'"}, status=400)

    device_path = resolve_device_path_from_name(camera_name)
    if not device_path:
        return web.json_response({"error": f"Camera '{camera_name}' not found"}, status=404)

    if device_path in pipeline_map:
        pipeline_map[device_path].set_state(Gst.State.NULL)
        del pipeline_map[device_path]

    try:
        config = load_camera_config(camera_name)
        pipeline = build_pipeline(config, device_path)
        pipeline.set_state(Gst.State.PLAYING)
        pipeline_map[device_path] = pipeline
        return web.json_response({"status": "started", "device": device_path})
    except Exception as e:
        return web.json_response({"error": str(e)}, status=500)

async def stop_all_streams(request):
    for pipeline in pipeline_map.values():
        pipeline.set_state(Gst.State.NULL)
    pipeline_map.clear()
    return web.json_response({"status": "all streams stopped"})

async def list_devices(request):
    try:
        result = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True)
        output = result.stdout.strip().splitlines()
        devices = []
        current = None

        for line in output:
            if not line.startswith("\t") and line.strip():
                if current:
                    devices.append(current)
                name = re.sub(r"\s(.):?$", "", line.strip())
                current = {"name": name, "devices": []}
            elif line.startswith("\t") and current:
                dev_path = line.strip()
                if dev_path.startswith("/dev/video"):
                    current["devices"].append(dev_path)

        if current:
            devices.append(current)

        return web.json_response({"devices": devices})
    except Exception as e:
        return web.json_response({"error": f"Failed to list devices: {e}"}, status=500)

async def set_bitrate(request):
    data = await request.json()
    camera_name = data.get("camera")
    new_bitrate = data.get("bitrate")

    if not camera_name or new_bitrate is None:
        return web.json_response({"error": "Missing 'camera' or 'bitrate'"}, status=400)

    device_path = resolve_device_path_from_name(camera_name)
    if not device_path or device_path not in pipeline_map:
        return web.json_response({"error": f"No active stream for camera '{camera_name}'"}, status=404)

    pipeline = pipeline_map[device_path]
    encoder = pipeline.get_by_name("encoder")

    if encoder:
        encoder.set_property("bitrate", int(new_bitrate))
        return web.json_response({"status": f"bitrate set to {new_bitrate} kbps"})
    else:
        return web.json_response({"error": "Encoder not found in pipeline"}, status=500)

if __name__ == "__main__":
    app = web.Application(middlewares=[cors_middleware])
    app.router.add_post("/start-stream", start_stream)
    app.router.add_post("/stop-all", stop_all_streams)
    app.router.add_post("/set-bitrate", set_bitrate)
    app.router.add_get("/list-devices", list_devices)

    # CORS preflight support
    for path in ["/start-stream", "/stop-all", "/set-bitrate", "/list-devices"]:
        app.router.add_options(path, handle_options)

    web.run_app(app, host="0.0.0.0", port=8000)
