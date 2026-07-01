import math
import requests
import numpy as np
import matplotlib.pyplot as plt
from io import StringIO, BytesIO
from PIL import Image

def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
    return (xtile, ytile)

def num2deg(xtile, ytile, zoom):
    n = 2.0 ** zoom
    lon_deg = xtile / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
    lat_deg = math.degrees(lat_rad)
    return (lat_deg, lon_deg)

def get_image_cluster(lat_deg, lon_deg, delta_lat,  delta_long, zoom):
    user_agent = {"User-agent": "McGill_Robotics Rover"}
    
    smurl = r"https://tile.openstreetmap.org/{0}/{1}/{2}.png"
    xmin, ymax = deg2num(lat_deg, lon_deg, zoom)
    xmax, ymin = deg2num(lat_deg + delta_lat, lon_deg + delta_long, zoom)
    
    cluster = Image.new('RGB',((xmax-xmin+1)*256-1,(ymax-ymin+1)*256-1) ) 
    for xtile in range(xmin, xmax+1):
        for ytile in range(ymin, ymax+1):
            try:
                imgurl = smurl.format(zoom, xtile, ytile)
                print("Opening: " + imgurl)
                imgstr = requests.get(imgurl, headers=user_agent).content
                print("Done with request.")

                tile = Image.open(BytesIO(imgstr))
                print("Generate Tile")

                cluster.paste(tile, box=((xtile-xmin)*256 ,  (ytile-ymin)*255))
                print("Pasted cluster")
            except Exception as e: 
                print(e) 
                print("Couldn't download image")
                tile = None

    return cluster
    
   
if __name__ == '__main__':
    a = get_image_cluster(45.5058, -73.5762, 0.0002, 0.0005, 17)
    fig = plt.figure()
    fig.patch.set_facecolor('none')
    plt.imshow(np.asarray(a))
    plt.axis('off')
    #plt.savefig('map.png', transparent=True, bbox_inches='tight', dpi=300)
    plt.show()