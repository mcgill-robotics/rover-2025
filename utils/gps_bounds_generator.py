import sys

def generate_bounds(lat, lon):
    """
    Generate bounds for the gps image with the following format:
    west: -112.75490,
    east: -112.74859, 
    north: 51.47228,
    south: 51.46956,
    """

    # These values are subject to change and are only hard-coded given the image dimensions
    north_south = abs(51.47228 - 51.46956) / 2
    west_east = abs(-112.75490 - (-112.74859)) / 2

    north = lat + north_south
    south = lat - north_south
    west = lon - west_east
    east = lon + west_east

    print(f"west: {west}, \n"  \
        f"east: {east}, \n" \
        f"nort: {north}, \n" \
        f"south: {south}")


if __name__ == "__main__":
    try:
        args = sys.argv[1:]
        lat = float(args[0])
        lon = float(args[1])
        generate_bounds(lat, lon)
    except:
        print("Invalid or nonexistent command line args, please provide a latitude and longitude as floats.")



