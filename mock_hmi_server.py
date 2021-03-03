# Team: Wayfinder
# Project: OWL
# Notes: This is the mock HMI server for exercising the frontend for OWL.
# Requires Python 3.7+ due to use of Python-native asyncio.
# Run as: $ python -m owl.hmi.api.mock_hmi_server

# Standard library imports
import asyncio
from datetime import datetime
import json
from math import sin
import os
import random
import threading
import xml.etree.ElementTree as ET

# Third party imports
from dotenv import load_dotenv
from quart import Quart, request, websocket, copy_current_websocket_context
from scipy.constants import degree

# Wayfinder imports
import owl

# Environment variables
HMI_IMAGE_PORT = "HMI_IMAGE_PORT"  # Port to listen on for mock image server
HMI_API_PORT = "HMI_API_PORT"  # Port to listen on for everything else
HMI_BIND_ADDRESS = "HMI_BIND_ADDRESS"  # Addresses to listen for connections on

# Datarates
TELEMETRY_FREQ_HZ = 20
NLS_FREQ_HZ = 20
SYSTEM_HEALTH_MONITOR_FREQ_HZ = 1
CAMERA_DATA_FREQ_HZ = 1
OWL_STATUS_FREQ_HZ = 1
FLIR5_FREQ_HZ = 20
RUNWAY_INFO_FREQ_HZ = 1
RELEASE_INFO_FREQ_HZ = 1

# OWL modes XML file
CONFIG_DIRS = ["/etc/owl/config", list(owl.__path__)[0] + "/config"]
OWL_MODES_DOC = "baked/software/apps/owl_manager.xml"

# Image resolutions
FLIR5_RES_PX = [2448, 2048]

# GPS
GPS_STATUS_LIST = ["VALID", "INVALID"]
GPS_POSITION_TYPE_LIST = [
    "NO SOLUTION",
    "UNKNOWN",
    "SINGLE",
    "PSEUDORANGE DIFFERENTIAL",
    "SBAS",
    "OMNISTAR",
    "FLOAT RTK",
    "INTEGER RTK",
    "FLOAT PPP",
    "INTEGER PPP",
    "FIXED",
    "3D"
]

# Camera
EXPOSURE_GAIN_MODES = ["Once", "Continuous", "Off"]


class MockInsGnss:
    def __init__(self):
        self.attitude_rpy_rad = [0, 0, 0]
        self.position_lla_deg_m = [0, 0, 0]
        self.velocity_ned_mps = [0, 0, 0]
        self.acceleration_xyz_mpss = [0, 0, 0]
        self.i = 0

        self.lock = threading.RLock()
        self.wiggle()

    def wiggle(self):
        threading.Timer(1 / TELEMETRY_FREQ_HZ, self.wiggle).start()
        with self.lock:
            i = self.i

            self.attitude_rpy_rad[0] = sin(i / 60) * 30 * degree  # Roll
            self.attitude_rpy_rad[1] = sin(i / 140) * 30 * degree  # Pitch
            self.attitude_rpy_rad[2] = (sin(i / 80) * 35 + 330) * degree  # Yaw

            self.velocity_ned_mps[0] = (sin(i / 400) + 1) * 40
            self.velocity_ned_mps[1] = (sin(i / 500) + 1) * 20
            self.velocity_ned_mps[2] = sin(i / 600) * 10

            self.position_lla_deg_m[0] = 37.460976 + (sin(i / 500) + 1)  # Latitude
            self.position_lla_deg_m[1] = -122.114922 + (sin(i / 400) + 1)  # Longitude
            self.position_lla_deg_m[2] = (sin(i / 800) + 1) * 2500  # Altitude (m)

            self.acceleration_xyz_mpss[0] = sin(i / 100)  # Longitudinal acceleration
            self.acceleration_xyz_mpss[1] = -sin(i / 60) * 3  # Lateral acceleration, up to +/- 3 m/s (~1/3 G)
            self.acceleration_xyz_mpss[2] = sin(i / 50) * 0.1  # Vertical acceleration

            self.i = (self.i + 1) % 1e9

    def get_sample(self):
        with self.lock:
            state = {
                "insGnss": {
                    "attitudeRpy_rad": self.attitude_rpy_rad,
                    "positionLla_deg_m": self.position_lla_deg_m,
                    "velocityNed_mps": self.velocity_ned_mps,
                    "accelerationXyz_mpss": self.acceleration_xyz_mpss,
                }
            }
            return json.dumps(state)


class MockCameraIns:
    def __init__(self):
        self.attitude_rpy_rad = [0, 0, 0]
        self.i = 0

        self.lock = threading.RLock()
        self.wiggle()

    def wiggle(self):
        threading.Timer(1 / CAMERA_DATA_FREQ_HZ, self.wiggle).start()
        with self.lock:
            i = self.i
            self.attitude_rpy_rad[0] = sin(i / 60) * 25 * degree  # Roll
            self.attitude_rpy_rad[1] = sin(i / 140) * 25 * degree  # Pitch
            self.attitude_rpy_rad[2] = (sin(i / 80) * 30 + 330) * degree  # Yaw

        self.i = (self.i + 1) % 1e9

    def get_sample(self):
        state = {"cameraIns": {"cameraAttitudeRpy_rad": self.attitude_rpy_rad}}
        return json.dumps(state)


class MockNls:
    def __init__(self):
        self.localizer_deflection = [0]
        self.glideslope_deflection = [0]
        self.atrk_m = [0]
        self.has_detection = False
        self.runway_polygon = []
        self.i = 0

        self.lock = threading.RLock()
        self.wiggle()

    def wiggle(self):
        threading.Timer(1 / NLS_FREQ_HZ, self.wiggle).start()
        with self.lock:
            i = self.i
            self.localizer_deflection = sin(i / 40)
            self.glideslope_deflection = sin(i / 30)
            self.atrk_m = (10 * i) % 6000
            self.has_detection = (self.i // 50) % 4 != 0
            if self.has_detection:
                self.runway_polygon = [
                    {"x": 1100 + 20 * sin(i / 40), "y": 900 + 20 * sin(i / 45)},
                    {"x": 1300 + 20 * sin(i / 42), "y": 900 + 20 * sin(i / 47)},
                    {"x": 1400 + 50 * sin(i / 44), "y": 1400 + 50 * sin(i / 49)},
                    {"x": 1000 + 50 * sin(i / 46), "y": 1400 + 50 * sin(i / 51)},
                ]
            else:
                self.runway_polygon = []
            self.i = (self.i + 1) % 1e9

    def get_sample(self):
        with self.lock:
            state = {
                "nlsOutput": {
                    "hasDetection": self.has_detection,
                    "dtrk_rad": 310 * degree,
                    "atrk_m": self.atrk_m,
                    "xtrk_m": 0,  # Not physically accurate, but ok as not used for now
                    "heightAboveRunway_m": 0,  # Also not currently used
                    "localizerDeflection": self.localizer_deflection,
                    "glideslopeDeflection": self.glideslope_deflection,
                    "runwayPolygon": self.runway_polygon,
                    "firstStageConfidence": 1,
                    "secondStageConfidence": 1,
                    "pnpLoss": 0.1,
                }
            }
            return json.dumps(state)


class MockSystemHealthMonitor:
    def __init__(self):
        self.alerts = [
            {
                "title": "The framastat is broken.",
                "severity": 2,
                "detail": "It's, like, really broken, guys. Leaking coolant and everything.",
            },
            {"title": "I'm not superstitious", "severity": 2, "detail": "...but I am a little stitious."},
            {
                "title": "Who let the dogs out?",
                "severity": 1,
                "detail": '"Whooo! Whooo!" said the OWL. Ha. Haha. Hahahahaha. Errr.',
            },
            {
                "title": "Did you remember to turn off the stove?",
                "severity": 1,
                "detail": "Because here you are, presumably a few thousand feet up in the air, and now you're "
                "wondering if your house is on fire. Which it probably is.",
            },
            {
                "title": "Hello world.",
                "severity": 0,
                "detail": "How are you doing? (Everybody is always concerned about what is going on in the world "
                "but no one ever asks the world how it feels. Rude.)",
            },
            {"title": "I feel the need...", "severity": 0, "detail": "The need for speed!"},
            {
                "title": "Romain, on berries",
                "severity": 0,
                "detail": "Good berries: cherries. Bad berries: Gene Rottenberry.",
            },
            {"title": "Don't Panic.", "severity": 0, "detail": "Time is an illusion. Lunchtime doubly so."},
            {"title": "Eddies in the space-time continuum.", "severity": 0, "detail": "Ah. Is he. Is he."},
            {
                "title": "I hope you're pleased with yourselves.",
                "severity": 0,
                "detail": "We could all have been killed - or worse, expelled.",
            },
        ]
        self.system_health = {
            "cpu_cores": [15, 14, 4, 22, 5, 6, 6, 3, 5, 4, 4, 5],
            "gpuUtilization_pct": 0,
            "gpuMemory_used_pct": 0,
            "gpuMemory_used_KiB": 115212288,
            "gpuTemperature_degc": 60,
            "ethernet": [{"name": "Flir5Forward", "rx_Bps": 5041650, "tx_Bps": 1598}],
            "diskUsage": [
                {
                    "name": "/",
                    "device": "/dev/mapper/ubuntu--vg-root",
                    "size_kB": 950445764,
                    "used_kB": 269034044,
                    "used_pct": 30.5,
                    "read_Bps": 0,
                    "write_Bps": 0,
                }
            ],
            "temperatures": [
                {"name": "Package id 0", "temperature_degc": 36},
                {"name": "Core 0", "temperature_degc": 35},
                {"name": "Core 1", "temperature_degc": 36},
                {"name": "Core 2", "temperature_degc": 35},
                {"name": "Core 3", "temperature_degc": 35},
                {"name": "Core 4", "temperature_degc": 34},
                {"name": "Core 5", "temperature_degc": 35},
            ],
            "memoryUsed_pct": 1,
            "cpuUtilization_pct": 48,
        }

        self.currentTime = {"hour": 0, "minute": 0, "second": 0}

        self.gpsPosition = {
            "gpsPositionStatus": "",
            "gpsPositionType": "",
            "numSatellites": 0,
            "latitudeAccuracy_m": 0,
            "longitudeAccuracy_m": 0,
        }
        self.gpsHeading = {
            "gpsHeadingStatus": "",
            "gpsTrueHeadingAccuracy_deg": 0,
        }

        self.active_alerts = []
        self.i = 0

        self.lock = threading.RLock()
        self.wiggle()

    def wiggle(self):
        threading.Timer(1 / SYSTEM_HEALTH_MONITOR_FREQ_HZ, self.wiggle).start()
        with self.lock:
            self.active_alerts = [self.alerts[x] for x in sorted(random.sample(range(len(self.alerts)), self.i))]
            self.i = (self.i + 1) % (len(self.alerts) + 1)
            self.system_health["gpuTemperature_degc"] = random.randint(0, 120)
            currentTime = datetime.utcnow()
            self.currentTime = {
                "clockStatus": int(round(currentTime.second * 0.05)),  # Change clock status every 10 sec
                "hour": currentTime.hour,
                "minute": currentTime.minute,
                "second": currentTime.second,
            }
            self.gpsPosition = {
                "gpsPositionStatus": GPS_STATUS_LIST[random.randint(0, 1)],
                "gpsPositionType": GPS_POSITION_TYPE_LIST[random.randint(0, 11)],
                "numSatellites": random.randint(0, 20),
                "latitudeAccuracy_m": random.uniform(0, 100),
                "longitudeAccuracy_m": random.uniform(0, 100),
            }
            self.gpsHeading = {
                "gpsHeadingStatus": GPS_STATUS_LIST[random.randint(0, 1)],
                "gpsTrueHeadingAccuracy_deg": random.uniform(0, 360),
            }

    def get_sample(self):
        with self.lock:
            state = {
                "systemHealthMonitor": {
                    "alerts": self.active_alerts,
                    "health": self.system_health,
                    "systemTime": self.currentTime,
                    "gpsPosition": self.gpsPosition,
                    "gpsHeading": self.gpsHeading,
                }
            }
            return json.dumps(state)


class MockCalibration:
    def __init__(self):
        self.i = 0
        self.seconds_to_next_capture = 0
        self.corners = []
        self.sectors_per_edge = 20
        self.checkerboards_collected = 0
        self.checkerboards_required = 30
        self.coverage_grids_covered = 0
        self.coverage_grids_required = self.sectors_per_edge * self.sectors_per_edge
        self.coverage = [[0] * self.sectors_per_edge for i in range(self.sectors_per_edge)]
        self.calibration_complete = False
        self.reprojection_error_threshold_px = 1.0
        self.last_reprojection_error_px = -1
        self.last_distortion = [-1, -1, -1, -1, -1]
        self.last_optical_center_px = [-1, -1]
        self.last_focal_px = [-1, -1]
        self.coverage_for_completion_percent = 100.0
        self.last_checkerboard_detection_time_ms = 0.5
        self.last_checkerboard_detected = True

        self.lock = threading.RLock()
        self.wiggle()

    def wiggle(self):
        threading.Timer(1 / CAMERA_DATA_FREQ_HZ, self.wiggle).start()
        with self.lock:
            i = self.i
            self.seconds_to_next_capture = 5 - (i % 5) + 0.01  # Just to make sure the frontend float handling works
            num_corners = 64
            u_corners = random.sample(range(0, FLIR5_RES_PX[0]), num_corners)
            v_corners = random.sample(range(0, FLIR5_RES_PX[1]), num_corners)
            self.corners = list(zip(u_corners, v_corners))
            self.checkerboards_collected = i % (self.checkerboards_required + 10)
            h = int(i // self.sectors_per_edge % self.sectors_per_edge)
            v = int(i % self.sectors_per_edge)
            self.coverage_grids_covered += 1 % self.coverage_grids_required
            self.coverage[h][v] = 0 if self.coverage[h][v] == 1 else 1
            self.calibration_complete = True if self.checkerboards_collected >= self.checkerboards_required else False
            self.last_reprojection_error_px = 0.5 if self.calibration_complete else -1
            self.last_distortion = [0.1, 0.2, 0, 0, 0.3] if self.calibration_complete else [-1, -1, -1, -1, -1]
            self.last_optical_center_px = [1024, 1224] if self.calibration_complete else [-1, -1]
            self.last_focal_px = [3478.2608695652175, 3478.2608695652175] if self.calibration_complete else [-1, -1]
            self.i = (self.i + 1) % 1e9
            self.last_checkerboard_detection_time_ms = random.random() * 1000
            self.last_checkerboard_detected = not self.last_checkerboard_detected

    def get_sample(self):
        with self.lock:
            state = {
                "intrinsicCalibrations": {
                    "MockCamera": {
                        "imageFrameID": self.i,
                        "secondsToNextCapture": self.seconds_to_next_capture,
                        "corners": self.corners,
                        "checkerboardsCollected": self.checkerboards_collected,
                        "checkerboardsRequired": self.checkerboards_required,
                        "lastCheckerboardDetectionTime_ms": self.last_checkerboard_detection_time_ms,
                        "lastCheckerboardDetected": self.last_checkerboard_detected,
                        "coverageGridsCovered": self.coverage_grids_covered,
                        "coverageGridsRequired": self.coverage_grids_required,
                        "coverage": self.coverage,
                        "collectionComplete": self.calibration_complete,
                        "calibrationComplete": self.calibration_complete,
                        "imageWidth_px": FLIR5_RES_PX[0],
                        "imageHeight_px": FLIR5_RES_PX[1],
                        "reprojectionErrorThresholdPx": self.reprojection_error_threshold_px,
                        "lastReprojectionErrorPx": self.last_reprojection_error_px,
                        "lastDistortion": self.last_distortion,
                        "lastOpticalCenterPx": self.last_optical_center_px,
                        "lastFocalPx": self.last_focal_px,
                        "coverageForCompletionPercent": self.coverage_for_completion_percent,
                    }
                }
            }
            return json.dumps(state)


class MockOwlManager:
    def __init__(self):
        self.mode = 0
        self.i = 0
        self.is_recording = False

        self.lock = threading.RLock()
        self.wiggle()
        self.owl_modes = self.get_owl_modes()

    def set_mode(self, mode):
        with self.lock:
            self.mode = mode

    def wiggle(self):
        threading.Timer(1 / OWL_STATUS_FREQ_HZ, self.wiggle).start()
        with self.lock:
            if round(self.i / 5) % 2 == 0:
                self.is_recording = False
            else:
                self.is_recording = True
            self.i = (self.i + 1) % 1e9

    def get_owl_modes(self):
        """
        Returns OWL modes information
        """
        for d in CONFIG_DIRS:
            path = os.path.join(d, OWL_MODES_DOC)
            modes_conf = {}
            if os.path.exists(path):
                print("Using owl manager file: {}".format(path))
                doc = ET.parse(path)
                root = doc.getroot()
                modes = root.find("modes")

                for m in modes.findall("mode"):
                    name = " ".join([w.capitalize() for w in m.attrib["name"].split("_")])
                    value = int(m.attrib["value"])

                    modes_conf[name] = value
                return modes_conf

        raise FileNotFoundError("ERROR: could not find owl manager file")

    def get_owl_status_sample(self):
        with self.lock:
            state = {"owlStatus": {"mode": self.mode, "isRecording": self.is_recording}}
            return json.dumps(state)

    def get_runway_info_sample(self):
        state = {
            "runwayInfo": {
                "airportIcao": "MWMW",
                "runwayDesignation": "36R",
                "refptLla_deg_m": [37.458503, -122.112469, 2],
            }
        }
        return json.dumps(state)

    def get_release_info(self):
        state = {
            "releaseInfo": {
                "releaseBranch": "my_dev_branch",
                "buildTimeUtc": "20200721184030",
                "releaseCommitHash": "b64a5064921b100fac7205ebe7b576f537aec63f",
                "jenkinsBuild": "NA",
                "mlModelsVersion": "2020_05_03",
                "nlsVersion": "0.8.1",
            }
        }
        return json.dumps(state)


class MockCamera:
    def __init__(self):
        self.i = 0

        self.camerasTelemetry = {
            "camerasTelemetry": {
                "MockCamera": {
                    "temperature_degc": 0,
                    "sensorRole": "MockCamera",
                    "serialNumber": "12345",
                    "ethernetInterface": "enp8s0",
                    "stream_url": "MockCamera_stream",
                    "server_port": os.getenv(HMI_IMAGE_PORT, "10875"),
                    "vendor": "Flir",
                    "model": "Flir5",
                    "deviceVersionMajor": 3,
                    "deviceVersionMinor": 1,
                    "systemFps": 0,
                    "desiredCameraFps": 20,
                    "frameId": 0,
                    "exposureAuto": EXPOSURE_GAIN_MODES[0],
                    "exposureTimeUs": 0,
                    "gainAuto": EXPOSURE_GAIN_MODES[0],
                    "gain": 0,
                    "maxExposureTimeUs": 0,
                    "maxGainDB": 0,
                    "maxGrayValue": 0,
                    "cameraSettingMode": 0
                }
            }
        }

        self.lock = threading.RLock()
        self.wiggle()

    def wiggle(self):
        threading.Timer(1 / CAMERA_DATA_FREQ_HZ, self.wiggle).start()
        with self.lock:
            self.i += 1
            self.camerasTelemetry = {
                "camerasTelemetry": {
                    "MockCamera": {
                        "temperature_degc": random.randint(0, 120),
                        "sensorRole": "MockCamera",
                        "serialNumber": "12345",
                        "ethernetInterface": "enp8s0",
                        "stream_url": "MockCamera_stream",
                        "server_port": os.getenv(HMI_IMAGE_PORT, "10875"),
                        "vendor": "Flir",
                        "model": "Flir5",
                        "deviceVersionMajor": 3,
                        "deviceVersionMinor": 1,
                        "systemFps": random.randint(15, 25),
                        "desiredCameraFps": 20,
                        "frameId": self.i,
                        "exposureAuto": EXPOSURE_GAIN_MODES[random.randint(0, 2)],
                        "exposureTimeUs": random.random(),
                        "gainAuto": EXPOSURE_GAIN_MODES[random.randint(0, 2)],
                        "gain": random.random(),
                        "maxExposureTimeUs": 30000 + (0.1 * random.random()),
                        "maxGainDB": 47 + (0.1 * random.random()),
                        "maxGrayValue": 93 + (0.1 * random.random()),
                        "cameraSettingMode": 0
                    }
                }
            }

    def get_camera_telemetry_sample(self):
        with self.lock:
            return json.dumps(self.camerasTelemetry)


# ------------------------------------------------------------------------------------


# Load the environment
file_dir = os.path.abspath(os.path.dirname(__file__))
hmi_environment = os.getenv("HMI_ENVIRONMENT")
if hmi_environment:
    load_dotenv(os.path.join(file_dir, ".env.{}".format(hmi_environment.lower())))
else:
    load_dotenv(os.path.join(file_dir, ".env"))

# Make mocks
mock_ins_gnss = MockInsGnss()
mock_camera_ins = MockCameraIns()
mock_nls = MockNls()
mock_system_health_monitor = MockSystemHealthMonitor()
mock_calibration = MockCalibration()
mock_owl_manager = MockOwlManager()
mock_camera = MockCamera()

# Make the Quart server
api_port = int(os.getenv(HMI_API_PORT))
bind_address = os.getenv(HMI_BIND_ADDRESS)
print("Starting API server on {}:{}".format(bind_address, api_port))
hmi_server = Quart(__name__, static_url_path="")


@hmi_server.after_request
async def cors(response):  # noqa: E999
    """
    Add CORS headers to all responses
    """
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["*Access-Control-Allow-Methods"] = "*"
    return response


async def send_ins_gnss():
    while True:
        await websocket.send(mock_ins_gnss.get_sample())
        await asyncio.sleep(1 / TELEMETRY_FREQ_HZ)


async def send_camera_ins():
    while True:
        await websocket.send(mock_camera_ins.get_sample())
        await asyncio.sleep(1 / TELEMETRY_FREQ_HZ)


async def send_nls():
    while True:
        await websocket.send(mock_nls.get_sample())
        await asyncio.sleep(1 / NLS_FREQ_HZ)


async def send_system_health():
    while True:
        await websocket.send(mock_system_health_monitor.get_sample())
        await asyncio.sleep(1 / SYSTEM_HEALTH_MONITOR_FREQ_HZ)


async def send_calibration_data():
    while True:
        await websocket.send(mock_calibration.get_sample())
        await asyncio.sleep(1 / CAMERA_DATA_FREQ_HZ)


async def send_camera_telemetry():
    while True:
        await websocket.send(mock_camera.get_camera_telemetry_sample())
        await asyncio.sleep(1 / CAMERA_DATA_FREQ_HZ)


async def send_owl_status():
    while True:
        await websocket.send(mock_owl_manager.get_owl_status_sample())
        await asyncio.sleep(1 / OWL_STATUS_FREQ_HZ)


async def send_runway_info():
    while True:
        await websocket.send(mock_owl_manager.get_runway_info_sample())
        await asyncio.sleep(1 / RUNWAY_INFO_FREQ_HZ)


async def send_release_info():
    while True:
        await websocket.send(mock_owl_manager.get_release_info())
        await asyncio.sleep(1 / RELEASE_INFO_FREQ_HZ)


@hmi_server.route("/runway")
async def process_runway_request():
    runway_designation = request.args.get("runway_designation")
    airport_icao = request.args.get("airport_icao")
    print("Airport: {} Runway: {}".format(airport_icao, runway_designation))
    return json.dumps({"airportIcao": airport_icao, "runwayDesignation": runway_designation})


@hmi_server.route("/airports", methods=["GET"])
async def publish_available_airports():
    available_airports = {
        "KPAO": ["31", "13"],
        "KWVI": ["2", "20", "9", "27"],
        "KLVK": ["7L", "25R", "7R", "25L"],
        "KMRY": ["10R", "28L", "10L", "28R"],
        "LFBO": ["14L", "32R", "14R", "32L"],
    }
    return json.dumps(available_airports)


@hmi_server.route("/modeStr2Int")
async def publish_mode_str_to_int():
    return json.dumps(mock_owl_manager.owl_modes)


@hmi_server.route("/modeInt2Str")
async def publish_mode_int_to_str():
    d = {v: k for k, v in mock_owl_manager.owl_modes.items()}
    return json.dumps(d)


@hmi_server.route("/mode", methods=["GET"])
def set_mode():
    mode = int(request.args.get("mode_enum"))
    print("Received mode {}".format(mode))
    mock_owl_manager.set_mode(mode)
    return str(mode)


@hmi_server.route("/software_version")
async def process_software_version():
    return mock_owl_manager.get_release_info()


@hmi_server.route("/")
async def root():
    return await hmi_server.send_static_file("index.html")


@hmi_server.route("/power", methods=["GET"])
async def process_power_command():
    command = request.args.get("power")
    print("Received power request: {}".format(command))
    return "Done"


@hmi_server.route("/camera_commands", methods=["GET"])
async def process_camera_command():
    sensor_role = request.args.get("sensor_role")
    command = request.args.get("command")
    print("Received command {} for sensor {}".format(command, sensor_role))
    return command


@hmi_server.route("/fte_note", methods=["POST"])
async def process_fte_note():
    data = await request.data
    note = json.loads(data)
    print("Received new note: {} \n at {}".format(note["note"], note["timestamp_ms"]))
    return "Done"


@hmi_server.route("/camera_settings", methods=["POST"])
async def process_camera_settings():
    """
    The POST request should have the following keys:
        - auto_exposure_upper_limit_us
        - auto_gain_upper_limit_db
        - auto_exposure_grey_value_upper_limit
        - camera_settings_mode
    """
    data = await request.data
    camera_settings = json.loads(data)
    print("Received new camera settings: {}".format(camera_settings))
    return "Done"


@hmi_server.route("/update_recording_metadata", methods=["POST"])
async def process_update_recording_metadata():
    data = await request.data
    metadata = json.loads(data)
    print("Received new recording metadata: {}".format(metadata))
    return "Done"


@hmi_server.websocket("/telemetry")
async def send_telemetry():
    send_ins_gnss_task = asyncio.ensure_future(copy_current_websocket_context(send_ins_gnss)())
    send_camera_ins_task = asyncio.ensure_future(copy_current_websocket_context(send_camera_ins)())
    send_owl_status_task = asyncio.ensure_future(copy_current_websocket_context(send_owl_status)())
    send_nls_task = asyncio.ensure_future(copy_current_websocket_context(send_nls)())
    send_system_health_task = asyncio.ensure_future(copy_current_websocket_context(send_system_health)())
    send_calibration_task = asyncio.ensure_future(copy_current_websocket_context(send_calibration_data)())
    send_camera_telemetry_task = asyncio.ensure_future(copy_current_websocket_context(send_camera_telemetry)())
    send_runway_info_task = asyncio.ensure_future(copy_current_websocket_context(send_runway_info)())
    send_release_info_task = asyncio.ensure_future(copy_current_websocket_context(send_release_info)())
    try:
        await asyncio.gather(
            send_ins_gnss_task,
            send_camera_ins_task,
            send_owl_status_task,
            send_nls_task,
            send_system_health_task,
            send_calibration_task,
            send_camera_telemetry_task,
            send_runway_info_task,
            send_release_info_task,
        )
    finally:
        send_ins_gnss_task.cancel()
        send_camera_ins_task.cancel()
        send_owl_status_task.cancel()
        send_nls_task.cancel()
        send_system_health_task.cancel()
        send_calibration_task.cancel()
        send_camera_telemetry_task.cancel()
        send_runway_info_task.cancel()
        send_release_info_task.cancel()


@hmi_server.route("/restart_camera", methods=["GET"])
async def restart_camera():
    print("Received camera restart request")
    return "Done"

if __name__ == "__main__":
    # Run the Quart server. Host is specified for external clients; uncommon port to avoid conflict
    hmi_server.run(host=bind_address, port=api_port)
