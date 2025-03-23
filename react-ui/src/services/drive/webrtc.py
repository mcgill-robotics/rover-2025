import asyncio
import json
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer

pcs = set()  # Set of all peer connections
previous_stats = {}  # List of stats

# CORS middleware to handle preflight and allow all origins
@web.middleware
async def cors_middleware(request, handler):
    response = await handler(request)
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
    return response

async def offer(request):
    print(request)
    id = request.rel_url.query["id"]

    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        print("ICE connection state is %s" % pc.iceConnectionState)
        if pc.iceConnectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    # Normal encoding
    # options = {"framerate": "20", "video_size": "640x480"}
    # player = MediaPlayer(f'/dev/video{id}', format="v4l2", options=options)

    # GStreamer pipeline for H.265 encoding
    gst_pipeline = (
        f"v4l2src device=/dev/video{id} ! "
        "video/x-raw,width=640,height=480,framerate=20/1 ! "
        "nvvidconv ! "                                  # Convert to a compatible format for the encoder
        "video/x-raw(memory:NVMM),format=NV12 ! "       # Use NVMM (NVIDIA Memory) for hardware acceleration
        "nvh265enc ! "                                  # NVIDIA H.265 encoder
        "video/x-h265,stream-format=byte-stream ! "     # Output H.265 stream
        "appsink"                                       # Send the stream to the application
    )

    # Use the GStreamer pipeline with MediaPlayer
    player = MediaPlayer(gst_pipeline, format="h265")

    await pc.setRemoteDescription(offer)
    for t in pc.getTransceivers():
        if t.kind == "video" and player.video:
            pc.addTrack(player.video)

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    response = web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )

    return response

async def get_bandwidth_stats(request):
    global previous_stats
    stats_list = []
    for pc in pcs:
        stats = await pc.getStats()
        for report in stats.values():
            if report.type == "outbound-rtp" and hasattr(report, "kind") and report.kind == "video":
                if hasattr(report, "bytesSent") and hasattr(report, "timestamp"):
                    timestamp_ms = report.timestamp.timestamp() * 1000  # Convert to ms

                    prev_report = previous_stats.get(pc, None)
                    if prev_report:
                        prev_bytes = prev_report["bytesSent"]
                        prev_timestamp_ms = prev_report["timestamp"]

                        time_diff_s = (timestamp_ms - prev_timestamp_ms) / 1000
                        byte_diff = report.bytesSent - prev_bytes

                        if time_diff_s > 0:
                            # Calculate bitrate in kbps
                            bitrate_kbps = (byte_diff * 8) / time_diff_s / 1000
                            stats_list.append({"bitrate_kbps": bitrate_kbps, "timestamp": timestamp_ms})

                            print(f"Outbound Bandwidth: {bitrate_kbps:.2f} kbps over last {time_diff_s:.2f} seconds")

                    previous_stats[pc] = {
                        "bytesSent": report.bytesSent,
                        "timestamp": timestamp_ms
                    }

    return web.json_response({"bandwidth_stats": stats_list})

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

async def list_video_devices(request):
    try:
        # Run "ls /dev/video*" asynchronously
        result = await asyncio.create_subprocess_shell(
            'ls /dev/video*',
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )

        stdout, stderr = await result.communicate()

        if result.returncode != 0:
            response = web.json_response({"error": stderr.decode()}, status=500)
        else:
            # Parse devices and return JSON
            devices = stdout.decode().strip().split("\n")
            response = web.json_response({"devices": devices})

        return response

    except Exception as e:
        response = web.json_response({"error": str(e)}, status=500)
        return response

async def on_shutdown(app):
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

if __name__ == "__main__":
    app = web.Application(middlewares=[cors_middleware])
    app.on_shutdown.append(on_shutdown)

    # Camera Feed endpoint
    app.router.add_options("/offer", handle_options)
    app.router.add_post("/offer", offer)

    # Available Devices endpoint
    app.router.add_get("/video-devices", list_video_devices)

    # Bandwidth monitoring endpoint
    app.router.add_get("/bandwidth-stats", get_bandwidth_stats)

    web.run_app(app, host="0.0.0.0", port=8081, ssl_context=None)