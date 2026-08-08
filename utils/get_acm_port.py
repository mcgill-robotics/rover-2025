#!/usr/bin/python3

#
# This script attempts to match USB cdc devices to their corresponding
# /dev/ttyACMx device path to autoconfigure services based on the needed
# embedded subsystem.
#
# When used as a module, the get_ACM_port function is used to 
# match a subsystem to the device path.
#
# This script can also be used from the CLI to print the device path to stdout.
# 
# Limitations of this script:
# The script relies on USB Product ID and Vendor ID and serial numbers to
# identify devices. If two devices of the same type (same PID and VID) have the
# same serial number it will not be able to differentiate them and will throw 
# an error (eg. If Arm and Drive CANables have the same serial number 
# in firmware). However, the use of VID and PID in the matching criteria, 
# in theory, means that different types of devices in theory should not be 
# mistaken for one another.

from enum import Enum
import os
import subprocess
import json
import sys

DEV_PATH = "/dev" # Constant path to /dev to list all devices

class DeviceInfo():
	"""Data class to hold usb device information
	"""

	def __init__(self, vid, pid, serial, display_name=""):
		self.vid = vid # USB Vendor ID
		self.pid = pid # USB Product ID
		self.serial = serial # Device Serial number
		self.display_name = display_name # Optional display name for logging
	
	# Provide an equals for easy comparisons
	def __eq__(self, other):
		if isinstance(other, DeviceInfo):
			return self.vid.lower() == other.vid.lower() \
				and self.pid.lower() == other.pid.lower() \
				and self.serial.lower() == other.serial.lower()

		return False

class Subsystem(Enum):
	"""Known subsystems that can be matched
	"""

	# Add known subsystems here...
	DRIVE = DeviceInfo(
		vid="16d0",
		pid="117e",
		serial="2087338C3630",
		display_name = "Drive"
	)
	ARM = DeviceInfo(
		vid="0000",
		pid="0000",
		serial="00000000000000000000",
		display_name = "Arm"
	)
	# TODO: change to actual info
	ARM_BRUSHED = DeviceInfo(
		vid="cafe",
		pid="7001",
		serial="20003E001950453055373020",
		display_name = "Arm Brushed"
	)
	GPS = DeviceInfo(
		vid="cafe",
		pid="6001",
		serial="28003E001950453055373020",
		display_name = "GPS"
	)
	SCIENCE = DeviceInfo(
		vid="cafe",
		pid = "5001",
		serial=20003E001950453055373020,
		display_name="Science"
	)
	


class UdevFetchException(Exception):
	"""Exception for udevadm errors
	"""

	def __init__(self, message:str, path:str, stderr:str):
		self.message = message
		self.device_path = path
		self.stderr = stderr
		super().__init__(f"{self.message}\n{stderr}")

class DeviceMatchingException(Exception):
	"""Exception for failed subsystem to device path matching
	"""

	def __init__(self, message, matched_paths):
		self.message = message
		self.paths = matched_paths
		super().__init__(self.message)

def fetch_device_info(device_path:str) -> DeviceInfo | None:
	"""Gets information from udevadm to identify a device

	Args:
		device_path (str): Path to device

	Raises:
		UdevFetchException: Thrown if udevadm exists with non-zero exit code

	Returns:
		DeviceInfo: PID,VID and serial for the device
	"""

	# Query device information using udevadm
	result = subprocess.run(
		["udevadm", "info", "-n", f"{DEV_PATH}/{device_path}"], 
		capture_output=True, 
		text=True)

	if result.returncode != 0:
		raise UdevFetchException("Could not fetch device info", device_path, result.stderr)

	lines = result.stdout.split("\n")
	properties = {}
	for line in lines:
		colon_location = line.find(":")
		if colon_location != -1:
			if colon_location+1 < len(line):
				kv_pair = line[colon_location+1:]
				eq_location = kv_pair.find("=")
				if eq_location != 1:
					if eq_location+1 < len(kv_pair):
						key = kv_pair[:eq_location].strip().upper()
						value = kv_pair[eq_location+1:].strip()
						properties[key] = value
	# properties = json.loads(result.stdout)

	if "ID_VENDOR_ID" in properties \
		and "ID_MODEL_ID" in properties \
		and "ID_SERIAL_SHORT" in properties:
		usb_vid = properties["ID_VENDOR_ID"]
		usb_pid = properties["ID_MODEL_ID"]
		serial = properties["ID_SERIAL_SHORT"]
		return DeviceInfo(usb_vid, usb_pid, serial)

	return None
	
def get_ACM_port(subsystem:Subsystem=Subsystem.DRIVE) -> str:
	"""Gets the path to a ACM device for a subsystem

	Args:
		subsystem (Subsystem, optional): Subsystem for which to get the device.
		 Defaults to Subsystem.DRIVE.

	Raises:
		DeviceMatchingException: Thrown if no devices match
		DeviceMatchingException: Thrown if multiple devices match

	Returns:
		str: Path to the device
	"""

	# Get all ACM ports on system
	devices =  os.listdir(DEV_PATH)
	acm_ports = filter(lambda x:x.startswith("ttyACM"), devices)

	# Get display name for error case
	display_name = subsystem.value.display_name.strip()
	display_name = str(subsystem) if display_name  == "" else display_name
	matches = []
	# Get matches for target subsystem
	for port in acm_ports:
		dev_info = fetch_device_info(port)
		if subsystem.value == dev_info:
			port_path = f"{DEV_PATH}/{port}"
			matches.append(port_path)
			
	match_count = len(matches)

	if match_count == 1:
		return matches[0]

	elif match_count == 0:	
		raise DeviceMatchingException(f"No ACM port could be detected for {display_name}", matches)

	else:
		raise DeviceMatchingException(f"Multiple matching devices found for {display_name}", matches)



def help_message():
	"""help message for CLI mode
	"""

	print("USAGE: python3 acm_detect.py [--help| -d subsystem]", file=sys.stderr)
	print(file=sys.stderr)
	print("Get the ACM port for a subsystem on the rover", file=sys.stderr)
	print(file=sys.stderr)
	print("-h --help:\tprints this message", file=sys.stderr)
	print("-d --device:\tchoose a subsystem to identify (case insensitive), if omitted the program will default to drive", file=sys.stderr)

def arg_to_enum(arg:str)  -> Subsystem | None:
	"""match device argument in CLI mode to Subsystem enum

	Args:
		arg (str): String from argument

	Returns:
		Subsystem | None: Matched subsystem or None if unmatched
	"""

	arg_cleaned = arg.strip().lower()
	# match argument to enum 
	for subsystem in Subsystem:
		if subsystem.name.lower() == arg_cleaned:
			return subsystem
		elif subsystem.value.display_name == arg_cleaned:
			return subsystem
	return None

if __name__ == "__main__":
	subsystem_to_query = None
	if len(sys.argv) < 2:
		print("WARNING: No argument given, defaulting to drive subsystem", file=sys.stderr)
		print("")
	else:
		argument = sys.argv[1]

		if argument == "--help" or argument == "-h":
			help_message()
			exit(0)

		elif argument == "-d" or argument == "--device":
			if len(sys.argv) < 3:
				print("ERROR: A subsystem must be provided", file=sys.stderr)
				print()
				help_message()
				exit(1)

			else:
				matched_enum = arg_to_enum(sys.argv[2])
				if matched_enum is None:
					print("This device is not in the known devices", file=sys.stderr)
					print(f"Known devices: {', '.join([subsystem.name.lower() for subsystem in Subsystem])}", file=sys.stderr)
					exit(1)	

				subsystem_to_query = matched_enum

		else:
			help_message()
			exit(1)

	try:
		if subsystem_to_query is None:
			path = get_ACM_port()
		else:
			path = get_ACM_port(subsystem_to_query)
		print(path)
	except UdevFetchException as udev_error:
		print("An error occurred fetching info for a device", file=sys.stderr)
		print(f"failed device: {DEV_PATH}/{udev_error.device_path}", file=sys.stderr)
		print(f"udevadm output: {udev_error.stderr}", file=sys.stderr)
		exit(1)
	except DeviceMatchingException as dev_match_error:
		print(dev_match_error.message)
		exit(1)