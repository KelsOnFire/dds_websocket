# Team: Wayfinder
# Project: OWL
# Notes: This is the HMI server for interacting with owl via DDS.
# Requires Python 3.7+ due to use of Python-native asyncio.
# Run as: $ python -m owl.hmi.api.hmi_server

import asyncio
import json
import numpy as np
from quart import Quart, websocket, copy_current_websocket_context, request
import rticonnextdds_connector as rti
import os
from scipy.constants import degree
import sys
import threading
import yaml
import xml.etree.ElementTree as ET

import owl
from owl.dvt.gige_camera.enable_poe import EnablePoe
from owl.dvt.lib.dds_interface import DDS

AIRPORTS_DATABASE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "../../config/baked/data/real_airports_labels.yaml",
)
if not os.path.exists(AIRPORTS_DATABASE_PATH):
    AIRPORTS_DATABASE_PATH = "/etc/owl/config/baked/data/real_airports_labels.yaml"

HMI_API_PORT = 10874

CAMERA_DATA_FREQ_HZ = 1
CAMERA_CALIBRATION_DATA_FREQ_HZ = 10
NLS_FREQ_HZ = 20
OWL_MODE_FREQ_HZ = 1
SYSTEM_HEALTH_MONITOR_FREQ_HZ = 1
TELEMETRY_FREQ_HZ = 20
RUNWAY_INFO_FREQ_HZ = 1
RELEASE_INFO_FREQ_HZ = 1

NOMINAL_GLIDESLOPE_DEG = 3
GLIDESLOPE_RANGE_DEG = 1  # Localizer varies from 2 to 4 deg
LOCALIZER_RANGE_DEG = 3  # Localizer varies from -3 to 3 deg

TIMEOUT_MS = 50

CONFIG_DIRS = ["/etc/owl/config", list(owl.__path__)[0] + "/config"]
CONFIG_FILE = "baked/software/dds_core/qos.xml"
OWL_MODES_DOC = "baked/software/apps/owl_manager.xml"

# GPS Status enums
GPS_HEADING_VALID = 1 << 0
GPS_TILT_VALID = 1 << 2,
GPS_VELOCITY_VALID = 1 << 3,
GPS_DUAL_ANTENNA_ACTIVE = 1 << 4

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

CAMERA_SETTINGS_KEYS = {
    "auto_exposure_upper_limit_us": "AutoExposureMaxTimeValue",
    "auto_gain_upper_limit_db": "AutoGainMaxValue",
    "auto_exposure_grey_value_upper_limit": "AutoExposureMaxGrayValue",
    "contrast": "Contrast",
}


class DDSConnector(object):
    def __init__(self, debug=False):
        config_path = None
        for dir in CONFIG_DIRS:
            path = os.path.join(dir, CONFIG_FILE)
            print("Config", path)
            if os.path.exists(path):
                if debug:
                    print("Using config {}".format(path))
                config_path = path
                break
        if config_path is None:
            print("ERROR: could not find a config file")
            sys.exit(1)

        # Download airports DB
        self.airports_db = {}
        try:
            with open(AIRPORTS_DATABASE_PATH) as f:
                airports_db = yaml.safe_load(f)
                # Avoid issue with integers runway designations (eg. LEZL 09)
                self.airports_db = json.loads(json.dumps(airports_db))
        except EnvironmentError:
            print("Airport database not available")

        # Owl modes
        self.owl_modes = self.get_owl_modes()

        # Init states
        self.ins_gnss_state = {
            "insGnss": {
                "attitudeRpy_rad": [0, 0, 0],
                "positionLla_deg_m": [0, 0, 0],
                "velocityNed_mps": [0, 0, 0],
                "accelerationXyz_mpss": [0, 0, 0],
            }
        }

        self.nls_output = {
            "nlsOutput": {
                "hasDetection": False,
                "dtrk_rad": 0,
                "atrk_m": 0,
                "xtrk_m": 0,
                "heightAboveRunway_m": 0,
                "runwayPolygon": [],
                "localizerDeflection": 0,
                "glideslopeDeflection": 0,
                "firstStageConfidence": 0,
                "secondStageConfidence": 0,
                "pnpLoss": 0,
            }
        }

        self.system_health = {
            "systemHealthMonitor": {
                "alerts": [],
                "health": {
                    "cpu_cores": [],
                    "ethernet": [],
                    "diskUsage": [],
                    "temperatures": [],
                    "memoryUsed_pct": 0,
                    "cpuUtilization_pct": 0,
                    "gpuUtilization_pct": 0,
                    "gpuMemory_used_pct": 0,
                    "gpuMemory_used_KiB": 0,
                    "gpuTemperature_degc": 0,
                },
                "systemTime": {"clockStatus": "", "hour": 0, "minute": 0, "second": 0},
                "gpsPosition": {
                    "gpsPositionStatus": "",
                    "gpsPositionType": "",
                    "numSatellites": 0,
                    "latitudeAccuracy_m": 0,
                    "longitudeAccuracy_m": 0,
                },
                "gpsHeading": {"gpsHeadingStatus": "", "gpsTrueHeadingAccuracy_deg": 0},
            }
        }

        self.calibration_status = {
            "intrinsicCalibrations": {}
        }

        self.camera_telemetry = {
            "camerasTelemetry": {}
        }

        self.camera_ins_state = {"cameraIns": {"cameraAttitudeRpy_rad": [0, 0, 0]}}

        self.owl_status = {"owlStatus": {"mode": 0, "isRecording": False}}

        self.runway_info = {"runwayInfo": {"airportIcao": "", "runwayDesignation": ""}}

        self.release_info = {
            "releaseInfo": {
                "releaseBranch": "Unknown",
                "buildTimeUtc": "Unknown",
                "releaseCommitHash": "Unknown",
                "jenkinsBuild": "Unknown",
                "mlModelsVersion": "Unknown",
                "nlsVersion": "Unknown",
            }
        }

        self.dds = DDS()

        self.lock = threading.RLock()

        connector = rti.Connector("OwlParticipantLibrary::OwlParticipant", config_path)

        self.dds.set_callback("InsGnssSubscriber::InsGnssEkfReader", self.callback_ins_gnss_ekf)
        self.dds.set_callback("InsGnssSubscriber::InsGnssImuDataReader", self.callback_ins_gnss_imu)
        self.dds.set_callback("InsGnssSubscriber::InsGnssGpsTimeReader", self.callback_ins_gnss_time)
        self.dds.set_callback("InsGnssSubscriber::InsGnssGpsReader", self.callback_ins_gnss_gps)

        self.dds.set_callback("NlsSubscriber::NlsOutputReader", self.callback_nls_output)

        self.dds.set_callback("HealthMonitorSubscriber::HealthMonitorAlertReader", self.callback_health_monitor_alerts)
        self.dds.set_callback("HealthMonitorSubscriber::HealthMonitorHealthReader", self.callback_health_monitor_status)

        self.dds.set_callback(
            "CameraSubscriber::CameraCalibrationHealthReader", self.callback_camera_calibration_health)
        self.dds.set_callback(
            "CameraSubscriber::CameraCalibrationStatusReader", self.callback_camera_calibration_status)
        self.dds.set_callback("CameraSubscriber::CameraCalibrationReader", self.callback_camera_calibration)
        self.dds.set_callback("CameraSubscriber::CameraHealthReader", self.callback_camera_health)
        self.dds.set_callback("CameraInsSubscriber::CameraInsEkfReader", self.callback_camera_ins_ekf)

        self.dds.set_callback(
            "OwlManagerSubscriber::OwlManagerHealthMonitorReader", self.callback_owl_manager_health_monitor)
        self.dds.set_callback("OwlManagerSubscriber::ReleaseInfoReader", self.callback_release_info)
        self.dds.set_callback("OwlManagerSubscriber::RunwayInfoReader", self.callback_runway_info)

        self.poe_command = EnablePoe()

        self.owl_mode_request_writer = connector.getOutput("HmiServerPublisher::OwlModeRequestWriter")
        self.runway_request_writer = connector.getOutput("HmiServerPublisher::RunwayRequestWriter")
        self.computer_shutdown_writer = connector.getOutput("HmiServerPublisher::ComputerShutdownRequestWriter")
        self.fte_note_writer = connector.getOutput("HmiServerPublisher::FteNoteWriter")
        self.set_parameter_request_writer = connector.getOutput("CameraPublisher::SetParameterRequestWriter")
        self.set_camera_settings_mode_writer = connector.getOutput("CameraPublisher::SetCameraSettingsModeWriter")
        self.camera_command_writer = connector.getOutput("HmiServerPublisher::CameraCommandRequestWriter")
        self.update_recording_metadata_writer = connector.getOutput("HmiServerPublisher::UpdateRecordingMetadataWriter")
        self.poe_request_writer = connector.getOutput("OwlManagerPublisher::EnablePoeRequestWriter")

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

    def callback_ins_gnss_ekf(self, msg):
        with self.lock:
            self.ins_gnss_state["insGnss"]["attitudeRpy_rad"] = [
                msg["roll_rad"],
                msg["pitch_rad"],
                msg["yaw_rad"],
            ]
            self.ins_gnss_state["insGnss"]["velocityNed_mps"] = [
                msg["velocity_n_mps"],
                msg["velocity_e_mps"],
                msg["velocity_d_mps"],
            ]
            self.ins_gnss_state["insGnss"]["positionLla_deg_m"] = [
                msg["latitude_deg"],
                msg["longitude_deg"],
                msg["altitude_msl_m"],
            ]

    def callback_ins_gnss_imu(self, msg):
        with self.lock:
            self.ins_gnss_state["insGnss"]["accelerationXyz_mpss"] = [
                msg["accel_x_mpss"],
                msg["accel_y_mpss"],
                msg["accel_z_mpss"],
            ]

    def callback_ins_gnss_time(self, msg):
        with self.lock:
            self.system_health["systemHealthMonitor"]["systemTime"] = {
                "clockStatus": msg["clock_status"],
                "hour": msg["hour"],
                "minute": msg["minute"],
                "second": msg["second"],
            }

    def callback_ins_gnss_gps(self, msg):
        with self.lock:
            self.system_health["systemHealthMonitor"]["gpsPosition"] = {
                "gpsPositionStatus": "VALID" if (GPS_POSITION_TYPE_LIST[msg["gps_fix_type"]] != "NO_SOLUTION")
                                     else "INVALID",
                "gpsPositionType": GPS_POSITION_TYPE_LIST[msg["gps_fix_type"]],
                "numSatellites": msg["num_satellites"],
                "latitudeAccuracy_m": msg["latitude_accuracy_m"],
                "longitudeAccuracy_m": msg["longitude_accuracy_m"],
            }
            self.system_health["systemHealthMonitor"]["gpsHeading"] = {
                "gpsHeadingStatus": "VALID" if (msg["status_bits"] & GPS_HEADING_VALID) else "INVALID",
                "gpsTrueHeadingAccuracy_deg": msg["gps_true_heading_accuracy_rad"] / degree,
            }

    def get_ins_gnss_state(self):
        with self.lock:
            return json.dumps(self.ins_gnss_state)

    def callback_nls_output(self, msg):
        localizer_deflection = (
            np.clip(msg["lpath_deg"], -LOCALIZER_RANGE_DEG, LOCALIZER_RANGE_DEG) / LOCALIZER_RANGE_DEG
        )
        glideslope_deflection = (
            np.clip(msg["vpath_deg"] - NOMINAL_GLIDESLOPE_DEG, -GLIDESLOPE_RANGE_DEG, GLIDESLOPE_RANGE_DEG)
            / GLIDESLOPE_RANGE_DEG
        )
        with self.lock:
            self.nls_output["nlsOutput"] = {
                "hasDetection": msg["has_detection"],
                "dtrk_rad": msg["runway_heading_deg"] * degree,
                "atrk_m": msg["atrk_m"],
                "xtrk_m": msg["xtrk_m"],
                "heightAboveRunway_m": msg["height_above_rwy_m"],
                "localizerDeflection": localizer_deflection,
                "glideslopeDeflection": glideslope_deflection,
                "runwayPolygon": msg["runway_polygon"],  # Clockwise starting at the top left
                "firstStageConfidence": msg["first_stage_confidence"],  # Float between 0 and 1
                "secondStageConfidence": msg["second_stage_confidence"],  # Float between 0 and 1
                "pnpLoss": msg["pnp_loss"],  # Float >0
            }

    def get_nls_output(self):
        with self.lock:
            return json.dumps(self.nls_output)

    def callback_health_monitor_alerts(self, msg):
        with self.lock:
            self.system_health["systemHealthMonitor"]["alerts"] = msg["alerts"]

    def callback_health_monitor_status(self, msg):
        health = {
            "cpu_cores": [],
            "gpu": [],
            "ethernet": [],
            "diskUsage": [],
            "temperatures": msg["temperatures"],
            "memoryUsed_pct": 100 - msg["memory_available_pct"],
            "cpuUtilization_pct": 100 - msg["cpu_total"]["idle_pct"],
        }
        for cpu_core in msg["cpu_cores"]:
            health["cpu_cores"].append(100 - cpu_core["idle_pct"])
        for interface in msg["ethernet"]:
            if interface["sensor_role"] != "":
                health["ethernet"].append(
                    {"name": interface["sensor_role"], "tx_Bps": interface["tx_Bps"], "rx_Bps": interface["rx_Bps"]}
                )
        for disk in msg["disk_usage"]:
            if disk["name"] in ["/", "/root", "/owl/data"]:
                disk["name"] = "/"
                health["diskUsage"].append(disk)
        if len(msg["gpu"]) > 0:
            health["gpuUtilization_pct"] = msg["gpu"][0]["utilization_pct"]
            health["gpuMemory_used_pct"] = msg["gpu"][0]["memory_used_pct"]
            health["gpuMemory_used_KiB"] = msg["gpu"][0]["memory_used_KiB"]
            health["gpuTemperature_degc"] = msg["gpu"][0]["temperature_degc"]

            health["gpu"].append(msg["gpu"][0])
        with self.lock:
            self.system_health["systemHealthMonitor"]["health"] = health

    def get_system_health(self):
        with self.lock:
            return json.dumps(self.system_health)

    def callback_camera_calibration_health(self, msg):
        with self.lock:
            for camera_calibrator in msg['camera_calibrators']:
                if camera_calibrator["sensor_role"] not in self.calibration_status["intrinsicCalibrations"]:
                    self.calibration_status["intrinsicCalibrations"][camera_calibrator["sensor_role"]] = {}
                self.calibration_status['intrinsicCalibrations'][camera_calibrator["sensor_role"]]['currentState'] = \
                    camera_calibrator['current_state']

    def callback_camera_calibration_status(self, msg):
        coverage = []
        for row in msg["coverage"]:
            coverage.append(row["values"])

        corners = []
        for corner in msg["corners"]:
            corners.append([corner["x"], corner["y"]])

        with self.lock:
            if msg["sensor_role"] not in self.calibration_status["intrinsicCalibrations"]:
                self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]] = {}
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["imageFrameID"] = msg["image_frame_id"]
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["secondsToNextCapture"] = \
                msg["time_to_next_s"]
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["corners"] = corners
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["checkerboardsCollected"] = \
                msg["num_checkerboards_collected"]
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["checkerboardsRequired"] = \
                msg["num_checkerboards_required"]
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["lastCheckerboardDetectionTime_ms"] = \
                msg["last_checkerboard_detection_time_ms"]
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["lastCheckerboardDetected"] = \
                msg["last_checkerboard_detected"]
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["coverage"] = coverage
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["coverageGridsRequired"] = \
                msg["num_coverage_grids_required"]
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["coverageGridsCovered"] = \
                msg["num_coverage_grids_covered"]
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["collectionComplete"] = \
                msg["collection_complete"]
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["calibrationComplete"] = \
                msg["calibration_complete"]
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["imageWidth_px"] = \
                msg["image_width_px"]
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["imageHeight_px"] = \
                msg["image_height_px"]
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["reprojectionErrorThresholdPx"] = \
                msg["reprojection_error_threshold_px"]
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["coverageForCompletionPercent"] = \
                msg["coverage_for_completion_percent"]

    def callback_camera_calibration(self, msg):
        with self.lock:
            if msg["sensor_role"] not in self.calibration_status["intrinsicCalibrations"]:
                self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]] = {}
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["lastReprojectionErrorPx"] = \
                msg["error"]
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["lastDistortion"] = \
                msg["distortion"]
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["lastOpticalCenterPx"] = \
                [msg["intrinsics"][0]["values"][2], msg["intrinsics"][1]["values"][2]]
            self.calibration_status["intrinsicCalibrations"][msg["sensor_role"]]["lastFocalPx"] = \
                [msg["intrinsics"][0]["values"][0], msg["intrinsics"][1]["values"][1]]

    def get_calibration_status(self):
        with self.lock:
            return json.dumps(self.calibration_status)

    def fillFlirTelemetry(self, msg):
        camera_telemetry = {
            "exposureAuto": msg["visible_telemetry"]["exposure_auto"],
            "exposureTimeUs": msg["visible_telemetry"]["exposure_time_us"],
            "gainAuto": msg["visible_telemetry"]["gain_auto"],
            "gain": msg["visible_telemetry"]["gain"],
            "maxExposureTimeUs": msg["visible_telemetry"]["exposure_time_auto_upper_limit"],
            "maxGainDB": msg["visible_telemetry"]["gain_auto_upper_limit"],
            "maxGrayValue": msg["visible_telemetry"]["auto_gray_value_upper_limit"],
        }
        return camera_telemetry

    def fillTeledyneTelemetry(self, msg):
        camera_telemetry = {
            "integrationTimeUs": msg["infrared_telemetry"]["integration_time_us"],
            "contrast": msg["infrared_telemetry"]["contrast"],
        }
        return camera_telemetry

    def callback_camera_health(self, msg):
        for camera_health in msg["cameras"]:
            sensor_role = camera_health["sensor_role"]
            if sensor_role not in self.camera_telemetry["camerasTelemetry"]:
                self.camera_telemetry["camerasTelemetry"][sensor_role] = {}

            update_dict = {
                "vendor": camera_health["vendor"],
                "model": camera_health["model"],
                "deviceVersionMajor": camera_health["device_version_major"],
                "deviceVersionMinor": camera_health["device_version_minor"],
                "systemFps": camera_health["system_fps"],
                "desiredCameraFps": camera_health["desired_camera_fps"],
                "stream_url": sensor_role + "_stream",
                "server_port": camera_health["server_port"],
                "temperature_degc": camera_health["device_temperature_C"],
                "serialNumber": camera_health["serial_number"],
                "ethernetInterface": camera_health["ethernet_interface"],
                "frameId": camera_health["frame_id"],
                "cameraSettingMode": camera_health["camera_settings_mode"],
                "resolution": [camera_health["width_px"], camera_health["height_px"]],
            }

            # Camera specific telemetry
            camera_telemetry = {}
            if camera_health["vendor"] == "Teledyne DALSA":
                camera_telemetry = self.fillTeledyneTelemetry(camera_health)
            elif camera_health["vendor"] == "FLIR":
                camera_telemetry = self.fillFlirTelemetry(camera_health)
            update_dict.update(camera_telemetry)
            with self.lock:
                self.camera_telemetry["camerasTelemetry"][sensor_role].update(update_dict)

    def get_camera_telemetry(self):
        with self.lock:
            return json.dumps(self.camera_telemetry)

    def callback_camera_ins_ekf(self, msg):
        with self.lock:
            self.camera_ins_state["cameraIns"]["cameraAttitudeRpy_rad"] = [
                msg["roll_rad"],
                msg["pitch_rad"],
                msg["yaw_rad"],
            ]

    def get_camera_ins_state(self):
        with self.lock:
            return json.dumps(self.camera_ins_state)

    def process_camera_command(self, device_name, command):
        """
        Sends camera commands
        Args:
            - device_name: (str) <name of the device>
            - command: (str) 'ON', 'OFF' or 'RESTART', 'FORCE_CALIBRATION', 'RESET_CALIBRATION'

        Output:
            (bool): True if the command was successful, False otherwise
        """

        if command == "ON" or command == "OFF" or command == "RESTART":
            success = self.poe_command.enable_device(device_name, command)
        else:
            self.camera_command_writer.instance.setDictionary({"device_name": device_name, "command": command})
            self.camera_command_writer.write()
            success = True
        return success

    def callback_owl_manager_health_monitor(self, msg):
        with self.lock:
            self.owl_status["owlStatus"] = {"mode": msg["owl_mode"]["mode"], "isRecording": msg["is_recording"]}

    def callback_release_info(self, msg):
        with self.lock:
            self.release_info["releaseInfo"]["releaseBranch"] = msg["release_branch"]
            self.release_info["releaseInfo"]["buildTimeUtc"] = msg["build_time_utc"]
            self.release_info["releaseInfo"]["releaseCommitHash"] = msg["release_commit_hash"]
            self.release_info["releaseInfo"]["jenkinsBuild"] = msg["jenkins_build"]
            self.release_info["releaseInfo"]["mlModelsVersion"] = msg["ml_models_version"]
            self.release_info["releaseInfo"]["nlsVersion"] = msg["nls_version"]

    def get_release_info(self):
        with self.lock:
            return json.dumps(self.release_info)

    def get_owl_status(self):
        with self.lock:
            return json.dumps(self.owl_status)

    def callback_runway_info(self, msg):
        runway_refpt_lla_deg_m = None
        try:
            runway_refpt_lla_deg_m = self.airports_db[msg["airport_icao"]]["runways"][msg["runway_designation"]][
                "rwy_refpt"
            ]
        except KeyError as e:
            print("Key error getting runway reference point position: {}".format(e))

        with self.lock:
            self.runway_info["runwayInfo"] = {
                "airportIcao": msg["airport_icao"],
                "runwayDesignation": msg["runway_designation"],
                "refptLla_deg_m": runway_refpt_lla_deg_m,
            }

    def get_runway_info(self):
        with self.lock:
            return json.dumps(self.runway_info)

    def send_owl_mode_request(self, owl_mode):
        self.owl_mode_request_writer.instance.setDictionary({"mode": owl_mode})
        self.owl_mode_request_writer.write()

    def publish_runway_request(self, airport_icao, runway_designation):
        """
        Publish runway request to the OWL manager
        """
        request = {
            "airport_icao": airport_icao,
            "runway_designation": runway_designation,
        }
        self.runway_request_writer.instance.setDictionary(request)
        self.runway_request_writer.write()

    def publish_computer_shutdown_request(self):
        """
        Publish computer shutdown request to the Owl Manager
        """
        self.computer_shutdown_writer.instance.setDictionary({"shutdown": True})
        self.computer_shutdown_writer.write()

    def publish_fte_note(self, note):
        """
        Publishes the FTE note over DDS
        """
        self.fte_note_writer.instance.setDictionary(
            {"note": note["note"], "timestamp_note_started": {"seconds_s": int(note["timestamp_ms"] / 1000)}}
        )
        self.fte_note_writer.write()

    def publish_camera_settings(self, camera_settings):
        """
        Publishes the camera settings over DDS
        """
        camera_name = camera_settings.pop("camera_name")
        for key in camera_settings:
            if key == "camera_settings_mode":
                self.set_camera_settings_mode_writer.instance.setDictionary(
                    {"sensor_role": camera_name,
                     "mode": camera_settings["camera_settings_mode"]}
                )
                self.set_camera_settings_mode_writer.write()
            else:
                self.set_parameter_request_writer.instance.setDictionary(
                    {"sensor_role": camera_name,
                     "key": CAMERA_SETTINGS_KEYS[key],
                     "value": str(camera_settings[key])}
                )
                self.set_parameter_request_writer.write()

    def publish_update_recording_metadata(self, metadata):
        """
        Publishes the recording metadata over DDS
        """
        self.update_recording_metadata_writer.instance.setDictionary(
            {
                "test_type": metadata["test_type"],
                "test_name": metadata["test_name"]
            }
        )
        self.update_recording_metadata_writer.write()

    def publish_camera_restart(self, camera_restart_request):
        self.poe_request_writer.instance.setDictionary({"device_name": camera_restart_request["camera_name"],
                                                        "command": EnablePoe.commands["RESTART"]})
        self.poe_request_writer.write()


if __name__ == "__main__":

    hmi_server = Quart(__name__, static_url_path="")

    @hmi_server.after_request
    async def cors(response):  # noqa E999
        """
        Add CORS headers to all responses
        """
        response.headers["Access-Control-Allow-Origin"] = "*"
        response.headers["*Access-Control-Allow-Methods"] = "*"
        return response

    dds_connector = DDSConnector()

    # Send
    async def send_ins_gnss_state():
        while True:
            await websocket.send(dds_connector.get_ins_gnss_state())
            await asyncio.sleep(1 / TELEMETRY_FREQ_HZ)

    async def send_nls_output():
        while True:
            await websocket.send(dds_connector.get_nls_output())
            await asyncio.sleep(1 / NLS_FREQ_HZ)

    async def send_system_health():
        while True:
            await websocket.send(dds_connector.get_system_health())
            await asyncio.sleep(1 / SYSTEM_HEALTH_MONITOR_FREQ_HZ)

    async def send_calibration_status():
        while True:
            await websocket.send(dds_connector.get_calibration_status())
            await asyncio.sleep(1 / CAMERA_CALIBRATION_DATA_FREQ_HZ)

    async def send_camera_telemetry():
        while True:
            await websocket.send(dds_connector.get_camera_telemetry())
            await asyncio.sleep(1 / CAMERA_DATA_FREQ_HZ)

    async def send_camera_ins_state():
        while True:
            await websocket.send(dds_connector.get_camera_ins_state())
            await asyncio.sleep(1 / TELEMETRY_FREQ_HZ)

    async def send_owl_status():
        while True:
            await websocket.send(dds_connector.get_owl_status())
            await asyncio.sleep(1 / OWL_MODE_FREQ_HZ)

    async def send_runway_info():
        while True:
            await websocket.send(dds_connector.get_runway_info())
            await asyncio.sleep(1 / RUNWAY_INFO_FREQ_HZ)

    async def send_release_info():
        while True:
            await websocket.send(dds_connector.get_release_info())
            await asyncio.sleep(1 / RELEASE_INFO_FREQ_HZ)

    @hmi_server.websocket("/telemetry")
    async def send_telemetry():
        send_ins_gnss_task = asyncio.ensure_future(copy_current_websocket_context(send_ins_gnss_state)())
        send_nls_task = asyncio.ensure_future(copy_current_websocket_context(send_nls_output)())
        send_alerts_task = asyncio.ensure_future(copy_current_websocket_context(send_system_health)())
        send_calibration_task = asyncio.ensure_future(copy_current_websocket_context(send_calibration_status)())
        send_camera_telemetry_task = asyncio.ensure_future(copy_current_websocket_context(send_camera_telemetry)())
        send_camera_ins_task = asyncio.ensure_future(copy_current_websocket_context(send_camera_ins_state)())
        send_owl_status_task = asyncio.ensure_future(copy_current_websocket_context(send_owl_status)())
        send_runway_info_task = asyncio.ensure_future(copy_current_websocket_context(send_runway_info)())
        send_release_info_task = asyncio.ensure_future(copy_current_websocket_context(send_release_info)())
        try:
            await asyncio.gather(
                send_ins_gnss_task,
                send_nls_task,
                send_alerts_task,
                send_calibration_task,
                send_camera_telemetry_task,
                send_camera_ins_task,
                send_owl_status_task,
                send_runway_info_task,
                send_release_info_task,
            )
        finally:
            send_ins_gnss_task.cancel()
            send_nls_task.cancel()
            send_alerts_task.cancel()
            send_calibration_task.cancel()
            send_camera_telemetry_task.cancel()
            send_camera_ins_task.cancel()
            send_owl_status_task.cancel()
            send_runway_info_task.cancel()
            send_release_info_task.cancel()

    @hmi_server.route("/camera_commands", methods=["GET"])
    async def process_camera_poe_command():
        device_name = request.args.get("sensor_role")
        command = request.args.get("command")
        success = dds_connector.process_camera_command(device_name, command)
        return str(success)

    @hmi_server.route("/modeStr2Int")
    async def publish_mode_str_to_int():
        return json.dumps(dds_connector.owl_modes)

    @hmi_server.route("/modeInt2Str")
    async def publish_mode_int_to_str():
        d = {v: k for k, v in dds_connector.owl_modes.items()}
        return json.dumps(d)

    @hmi_server.route("/mode", methods=["GET"])
    async def process_owl_mode():
        mode = request.args.get("mode_enum")
        dds_connector.send_owl_mode_request(mode)
        return str(mode)

    @hmi_server.route("/runway")
    async def process_runway_request():
        runway_designation = request.args.get("runway_designation")
        airport_icao = request.args.get("airport_icao")
        dds_connector.publish_runway_request(airport_icao, runway_designation)
        return json.dumps({"airportIcao": airport_icao, "runwayDesignation": runway_designation})

    @hmi_server.route("/airports", methods=["GET"])
    async def publish_available_airports():
        available_airports = {}
        for key in dds_connector.airports_db.keys():
            available_airports[key] = [str(key) for key in dds_connector.airports_db[key]["runways"].keys()]

        return json.dumps(available_airports)

    @hmi_server.route("/software_version")
    async def process_software_version():
        return dds_connector.get_release_info()

    @hmi_server.route("/")
    async def root():
        return await hmi_server.send_static_file("index.html")

    @hmi_server.route("/power", methods=["GET"])
    async def process_power_command():
        dds_connector.publish_computer_shutdown_request()
        return "Done"

    @hmi_server.route("/fte_note", methods=["POST"])
    async def process_fte_note():
        data = await request.data
        note = json.loads(data)
        dds_connector.publish_fte_note(note)
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
        dds_connector.publish_camera_settings(camera_settings)
        return "Done"

    @hmi_server.route("/update_recording_metadata", methods=["POST"])
    async def process_update_recording_metadata():
        data = await request.data
        metadata = json.loads(data)
        dds_connector.publish_update_recording_metadata(metadata)
        return "Done"

    @hmi_server.route("/restart_camera", methods=["POST"])
    async def restart_camera():
        data = await request.data
        camera_restart_request = json.loads(data)
        dds_connector.publish_camera_restart(camera_restart_request)
        return "Done"

    hmi_server.run(port=HMI_API_PORT, host="0.0.0.0")
