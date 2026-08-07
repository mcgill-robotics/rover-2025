import sys
import math
import re

def deg_to_dec(degrees, minutes, seconds, direction):
    """
    Convert degrees, minutes, seconds latitude and longitude to decimal format.
    """
    dd = float(degrees) + float(minutes)/60 + float(seconds)/(60*60)
    if direction == 'E' or direction == 'S':
        dd *= -1
    return dd

def parse_deg(deg_data):
    parts = re.split('[°\'"]+', deg_data)
    return deg_to_dec(parts[0], parts[1], parts[2], parts[3])

if __name__ == "__main__":
    # args = sys.argv[1:]
    # lat = parse_deg(str(args[0]))
    # lon = parse_deg(str(args[1]))
    print("Input latitude in degree format:")
    lat = input()
    print("Input lon in degree format:")
    lon = input()
    lat = parse_deg(lat)
    lon = parse_deg(lon)
    print(f"{lat}, {lon}")