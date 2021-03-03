# Team: Wayfinder
# Project: OWL
# Notes: This is the mock image server for exercising the frontend for OWL.
# Run as: $ python -m owl.hmi.api.mock_image_server

# Standard library imports
import time
import os

# Third party imports
from dotenv import load_dotenv
from flask import Flask, Response

# Wayfinder imports
# (None)

# Environment variables
HMI_IMAGE_PORT = "HMI_IMAGE_PORT"  # Port to listen on for mock image server
HMI_BIND_ADDRESS = "HMI_BIND_ADDRESS"  # Addresses to listen for connections on
HMI_IMAGE_PATH = "HMI_IMAGE_PATH"  # Stores mock camera image data

# Other constants
HMI_IMAGE_PATTERN = "{:07d}.jpeg"  # Filename pattern for mock camera image data

# Datarates
FREQ_HZ = 1


class MockCamera:
    def __init__(self):
        img_path = os.path.join(os.getenv(HMI_IMAGE_PATH), HMI_IMAGE_PATTERN)
        print("Loading mock camera images...")
        try:
            self.frames = [open(img_path.format(frame), "rb").read() for frame in range(1200, 2000)]
        except FileNotFoundError:
            print("No images found")
            self.frames = []
        print("Done.")
        self.i = 0
        self.num_frames = len(self.frames)

    def get_frame(self):
        self.i = (self.i + 1) % self.num_frames
        return self.frames[self.i]

    def frame_generator(self):
        while True:
            frame = self.get_frame()
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(1 / FREQ_HZ)


# ------------------------------------------------------------------------------------


# Load the environment
file_dir = os.path.abspath(os.path.dirname(__file__))
hmi_environment = os.getenv("HMI_ENVIRONMENT")
if hmi_environment:
    load_dotenv(os.path.join(file_dir, ".env.{}".format(hmi_environment.lower())))
else:
    load_dotenv(os.path.join(file_dir, ".env"))

# Make mocks
mock_camera = MockCamera()

# Make the Flask server
image_port = int(os.getenv(HMI_IMAGE_PORT))
bind_address = os.getenv(HMI_BIND_ADDRESS)
print("Starting camera server on {}:{}".format(bind_address, image_port))
image_server = Flask(__name__)


@image_server.route("/MockCamera_stream")
def flir5_stream():
    return Response(mock_camera.frame_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    # Run the Flask server. Host is specified for external clients; uncommon port to avoid conflict
    image_server.run(host=bind_address, port=image_port)
