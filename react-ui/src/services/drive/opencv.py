from fastapi import FastAPI, Response
from fastapi.middleware.cors import CORSMiddleware
import cv2
import threading

app = FastAPI()

# Allow frontend requests (adjust origins as needed)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Change this to your frontend domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Dictionary to map feed IDs to camera indexes
CAMERA_MAP = {
    "feed1": 0,
    "feed2": 1,
    "feed3": 2,
    "feed4": 3,
}

# Global capture cache to prevent reinitializing camera
capture_cache = {}

def generate_frames(camera_index):
    if camera_index not in capture_cache:
        cap = cv2.VideoCapture(camera_index)
        capture_cache[camera_index] = cap
    else:
        cap = capture_cache[camera_index]

    while True:
        success, frame = cap.read()
        if not success:
            break
        _, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.get("/video_feed/{feed_id}")
def video_feed(feed_id: str):
    if feed_id not in CAMERA_MAP:
        return Response(status_code=404)
    camera_index = CAMERA_MAP[feed_id]
    return Response(generate_frames(camera_index), media_type='multipart/x-mixed-replace; boundary=frame')
