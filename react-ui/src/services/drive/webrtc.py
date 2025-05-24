import asyncio
import json
import re
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer

pcs = set()
previous_stats = {}

@web.middleware
async def cors_middleware(request, handler):
    response = await handler(request)
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
    return response

async def offer(request):
    device_path = request.rel_url.query["id"]
    print(f"[OFFER] Using v4l2 device: {device_path}")

    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        print("ICE connection state is", pc.iceConnectionState)
        if pc.iceConnectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    try:
        player = MediaPlayer(
            device_path,
            format="v4l2",
            options={
                "framerate": "30",
                "video_size": "640x480",
                "input_format": "yuyv422"
            }
        )
        print("[DEBUG] player.video:", player.video)
    except Exception as e:
        error_message = f"[OFFER] MediaPlayer creation failed: {e}"
        print(error_message)
        return web.json_response({"error": f"Failed to open {device_path}. {str(e)}"}, status=500)

    await pc.setRemoteDescription(offer)
    print("[DEBUG] Transceivers:", pc.getTransceivers())

    for t in pc.getTransceivers():
        if t.kind == "video" and player.video:
            pc.addTrack(player.video)

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.json_response({
        "sdp": pc.localDescription.sdp,
        "type": pc.localDescription.type
    })

async def get_bandwidth_stats(request):
    global previous_stats
    stats_list = []
    for pc in pcs:
        stats = await pc.getStats()
        for report in stats.values():
            if report.type == "outbound-rtp" and getattr(report, "kind", None) == "video":
                if hasattr(report, "bytesSent") and hasattr(report, "timestamp"):
                    timestamp_ms = report.timestamp.timestamp() * 1000
                    prev_report = previous_stats.get(pc, None)
                    if prev_report:
                        time_diff = (timestamp_ms - prev_report["timestamp"]) / 1000
                        byte_diff = report.bytesSent - prev_report["bytesSent"]
                        if time_diff > 0:
                            bitrate_kbps = (byte_diff * 8) / time_diff / 1000
                            stats_list.append({
                                "bitrate_kbps": bitrate_kbps,
                                "timestamp": timestamp_ms
                            })
                            print(f"[STATS] Bitrate: {bitrate_kbps:.1f} kbps")
                    previous_stats[pc] = {
                        "bytesSent": report.bytesSent,
                        "timestamp": timestamp_ms
                    }
    return web.json_response({"bandwidth_stats": stats_list})

async def list_video_devices(request):
    try:
        result = await asyncio.create_subprocess_shell(
            "v4l2-ctl --list-devices",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        stdout, stderr = await result.communicate()

        if result.returncode != 0:
            print("[VIDEO] v4l2-ctl failed:", stderr.decode())
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
        print("[VIDEO] list_video_devices failed:", e)
        return web.json_response({"error": str(e)}, status=500)

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

async def on_shutdown(app):
    print("[SHUTDOWN] Closing peer connections...")
    await asyncio.gather(*[pc.close() for pc in pcs])
    pcs.clear()

if __name__ == "__main__":
    app = web.Application(middlewares=[cors_middleware])
    app.on_shutdown.append(on_shutdown)

    app.router.add_options("/offer", handle_options)
    app.router.add_post("/offer", offer)
    app.router.add_get("/video-devices", list_video_devices)
    app.router.add_get("/bandwidth-stats", get_bandwidth_stats)

    print("[SERVER] Starting WebRTC server on port 8081")
    web.run_app(app, host="0.0.0.0", port=8081, ssl_context=None)
