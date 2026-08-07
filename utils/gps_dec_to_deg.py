import sys
import math

def dec_to_deg(dec_lat, dec_lon):
    """
    Convert decimal latitude and longitude to degrees, minutes, seconds format.
    """
    if dec_lat>0:
        lat_dir = "N"
    else:
        lat_dir = "S"


    if dec_lon>0:
        long_dir = "E"
    else:
        long_dir = "W"
        
    lat_deg = math.floor(abs(dec_lat))
    lat_min = math.floor((abs(dec_lat)-lat_deg)*60)
    lat_sec = (((abs(dec_lat)-lat_deg)*60)-lat_min)*60
    long_deg = math.floor(abs(dec_lon))
    long_min = math.floor((abs(dec_lon)-long_deg)*60)
    long_sec = (((abs(dec_lon)-long_deg)*60)-long_min)*60
    converted = f"{lat_deg}°{lat_min}\'{lat_sec}\"{lat_dir} {long_deg}°{long_min}\'{long_sec}\"{long_dir}"

    print(converted)

if __name__ == "__main__":
    try:
        args = sys.argv[1:]
        lat = float(args[0])
        lon = float(args[1])
        dec_to_deg(lat, lon)
    except:
        print("Invalid or nonexistent command line args, please provide a latitude and longitude as floats.")