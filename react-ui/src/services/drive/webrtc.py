import asyncio
import json
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer

pcs = set()  # Set of all peer connections

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

    options = {"framerate": "20", "video_size": "640x480"}
    player = MediaPlayer(f'/dev/video{id}', format="v4l2", options=options)

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

    web.run_app(app, host="0.0.0.0", port=8081, ssl_context=None)