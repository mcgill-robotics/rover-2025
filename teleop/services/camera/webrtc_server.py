import asyncio
import json
import re
import time
import traceback
import subprocess

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from video_processor import ArucoProcessorTrack

pcs = set()
previous_stats = {}
last_stats_time = 0
STATS_INTERVAL = 2  # seconds

@web.middleware
async def cors_middleware(request, handler):
    response = await handler(request)
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
    return response

# ---- /offer ----
async def offer(request):
    try:
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    except Exception as e:
        return web.json_response({"error": f"Invalid SDP: {str(e)}"}, status=400)

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        if pc.iceConnectionState in ["failed", "disconnected", "closed"]:
            await pc.close()
            pcs.discard(pc)

    try:
        await pc.setRemoteDescription(offer)
        pc.addTrack(ArucoProcessorTrack())
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.json_response({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        })

    except Exception as e:
        traceback.print_exc()
        await pc.close()
        pcs.discard(pc)
        return web.json_response({"error": f"WebRTC setup failed: {str(e)}"}, status=500)

# ---- /bandwidth-stats ----
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

# ---- OPTIONS ----
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

# ---- Shutdown ----
async def on_shutdown(app):
    await asyncio.gather(*[pc.close() for pc in pcs])
    pcs.clear()

# ---- Main ----
if __name__ == "__main__":
    app = web.Application(middlewares=[cors_middleware])
    app.on_shutdown.append(on_shutdown)

    app.router.add_options("/offer", handle_options)
    app.router.add_post("/offer", offer)
    app.router.add_get("/bandwidth-stats", get_bandwidth_stats)

    print("[SERVER] WebRTC server running on port 8081")
    web.run_app(app, host="0.0.0.0", port=8081, ssl_context=None)
