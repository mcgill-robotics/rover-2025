import subprocess
from flask import Flask, Response
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

@app.route('/start-ros')
def start_ros():
    def generate():
        # Source ROS 2 workspace
        ros_setup = '/home/exoprisma/Desktop/Robotics/rover-2025/install/setup.bash'

        # Command to run the ROS 2 node
        command = f"source {ros_setup} && ros2 run cpp_pubsub talker"

        # Start the ROS 2 node
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            shell=True,  # Required for sourcing
            executable="/bin/bash"  # Ensures using Bash for 'source'
        )

        # Stream the ROS 2 output
        for line in process.stdout:
            yield f"data: {line}\n\n"

        process.wait()
        yield f"data: Process exited with code {process.returncode}\n\n"

    return Response(generate(), content_type='text/event-stream')

if __name__ == '__main__':
    app.run(debug=True, port=5000)