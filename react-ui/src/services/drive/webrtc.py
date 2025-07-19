import asyncio
import json
import re
import time
import traceback
import subprocess

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aruco_detector import ArucoVideoTrack

pcs = set()
previous_stats = {}
last_stats_time = 0
STATS_INTERVAL = 2  # seconds

MAX_RETRIES = 3
RETRY_BACKOFF = [1, 2, 4]  # seconds

# ------------------------- Middleware -------------------------

@web.middleware
async def cors_middleware(request, handler):
    response = await handler(request)
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
    return response

# ------------------------- Camera Force-Kill -------------------------

async def force_kill_device_users(device_path):
    try:
        result = subprocess.run(["fuser", "-v", device_path], capture_output=True, text=True)
        output = result.stdout
        pids = []

        for line in output.splitlines():
            parts = line.strip().split()
            if parts and parts[0] == device_path:
                pids = [int(p) for p in parts[1:] if p.isdigit()]

        if not pids:
            print(f"[FORCE-KILL] No processes found using {device_path}")
            return

        for pid in pids:
            print(f"[FORCE-KILL] Killing PID {pid} using {device_path}")
            subprocess.run(["kill", "-9", str(pid)])

    except Exception as e:
        print(f"[FORCE-KILL] Failed to kill process using {device_path}: {e}")

# ------------------------- /offer -------------------------

async def offer(request):
    device_path = request.rel_url.query.get("id")
    if not device_path:
        return web.json_response({"error": "Missing 'id' query param for video device"}, status=400)

    print(f"[OFFER] Requested v4l2 device: {device_path}")

    try:
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    except Exception as e:
        return web.json_response({"error": f"Invalid SDP: {str(e)}"}, status=400)

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        print("[ICE] State:", pc.iceConnectionState)
        if pc.iceConnectionState in ["failed", "disconnected", "closed"]:
            await pc.close()
            pcs.discard(pc)
            print("[ICE] Cleaned up PC.")

    track = None
    for attempt in range(MAX_RETRIES):
        try:
            track = ArucoVideoTrack(f'/dev/video{device_path}')
            break
        except Exception as e:
            print(f"[ERROR] ArucoVideoTrack attempt {attempt+1} failed: {e}")
            if "Device or resource busy" in str(e) and attempt == 0:
                print(f"[BUSY] Attempting to force-kill blocker on {device_path}")
                await force_kill_device_users(device_path)

            if attempt < MAX_RETRIES - 1:
                await asyncio.sleep(RETRY_BACKOFF[attempt])
            else:
                traceback.print_exc()
                return web.json_response({"error": f"Could not open device: {str(e)}"}, status=500)

    try:
        await pc.setRemoteDescription(offer)
        print("[WebRTC] Remote description set.")

        for t in pc.getTransceivers():
            if t.kind == "video":
                pc.addTrack(track)
                print("[WebRTC] Video track added.")

        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        print("[WebRTC] Answer created.")

        return web.json_response({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        })

    except Exception as e:
        traceback.print_exc()
        await pc.close()
        pcs.discard(pc)
        if track:
            await track.stop()
        return web.json_response({"error": f"WebRTC setup failed: {str(e)}"}, status=500)

# ------------------------- /bandwidth-stats -------------------------

async def get_bandwidth_stats(request):
    global previous_stats, last_stats_time
    now = time.time()

    if now - last_stats_time < STATS_INTERVAL:
        return web.json_response({"bandwidth_stats": [], "ping_ms": None})

    start_time = time.perf_counter()
    stats_list = []

    for pc in pcs:
        stats = await pc.getStats()
        bitrate_kbps = None
        rtt_ms = None

        for report in stats.values():
            if report.type == "outbound-rtp" and getattr(report, "kind", None) == "video":
                if hasattr(report, "bytesSent") and hasattr(report, "timestamp"):
                    timestamp_ms = report.timestamp.timestamp() * 1000
                    prev = previous_stats.get(pc)
                    if prev:
                        time_diff = (timestamp_ms - prev["timestamp"]) / 1000
                        byte_diff = report.bytesSent - prev["bytesSent"]
                        if time_diff > 0:
                            bitrate_kbps = (byte_diff * 8) / time_diff / 1000
                    previous_stats[pc] = {
                        "bytesSent": report.bytesSent,
                        "timestamp": timestamp_ms
                    }

            if report.type == "candidate-pair" and getattr(report, "state", "") == "succeeded":
                if getattr(report, "nominated", False) and hasattr(report, "currentRoundTripTime"):
                    rtt_ms = report.currentRoundTripTime * 1000
                    break

        stats_list.append({
            "bitrate_kbps": round(bitrate_kbps, 1) if bitrate_kbps else None,
            "rtt_ms": round(rtt_ms, 1) if rtt_ms else None
        })

    end_time = time.perf_counter()
    ping_ms = round((end_time - start_time) * 1000, 1)
    last_stats_time = now

    return web.json_response({
        "bandwidth_stats": stats_list,
        "ping_ms": ping_ms
    })

# ------------------------- /video-devices -------------------------

async def list_video_devices(request):
    try:
        result = await asyncio.create_subprocess_shell(
            "v4l2-ctl --list-devices",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        stdout, stderr = await result.communicate()

        if result.returncode != 0:
            return web.json_response({"error": stderr.decode()}, status=500)

        lines = stdout.decode().splitlines()
        devices = []
        current = None

        for line in lines:
            if line.strip() == "":
                continue
            if not line.startswith("\t"):
                if current:
                    devices.append(current)
                name = re.sub(r"\s*\(.*\):?$", "", line.strip())
                current = {"name": name, "devices": []}
            else:
                if current:
                    current["devices"].append(line.strip())

        if current:
            devices.append(current)

        return web.json_response({"devices": devices})

    except Exception as e:
        return web.json_response({"error": str(e)}, status=500)

# ------------------------- OPTIONS -------------------------

async def handle_options(request):
    requested_headers = request.headers.get('Access-Control-Request-Headers', '')
    return web.Response(
        status=200,
        headers={
            'Access-Control-Allow-Origin': '*',
            'Access-Control-Allow-Methods': 'POST, OPTIONS, GET',
            'Access-Control-Allow-Headers': requested_headers,
        },
    )

# ------------------------- Shutdown -------------------------

async def on_shutdown(app):
    print("[SHUTDOWN] Closing all peer connections...")
    await asyncio.gather(*[pc.close() for pc in pcs])
    pcs.clear()

# ------------------------- Main -------------------------

if __name__ == "__main__":
    app = web.Application(middlewares=[cors_middleware])
    app.on_shutdown.append(on_shutdown)

    app.router.add_options("/offer", handle_options)
    app.router.add_post("/offer", offer)
    app.router.add_get("/video-devices", list_video_devices)
    app.router.add_get("/bandwidth-stats", get_bandwidth_stats)

    print("[SERVER] WebRTC server running on port 8081")
    web.run_app(app, host="0.0.0.0", port=8081, ssl_context=None)
