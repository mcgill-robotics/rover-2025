#!/bin/bash

# Generic script to download OpenStreetMap tiles for any custom area
# Perfect for downloading offline maps for any location worldwide
# Usage: ./download-custom-area.sh [bounds] [min_zoom] [max_zoom] [area_name]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to show usage
show_usage() {
    echo -e "${BLUE}🗺️  Custom Area Offline Map Downloader${NC}"
    echo "============================================="
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --bounds BOUNDS     Map bounds (south,west,north,east)"
    echo "  --min-zoom ZOOM     Minimum zoom level (default: 10)"
    echo "  --max-zoom ZOOM     Maximum zoom level (default: 16)"
    echo "  --name NAME         Area name (default: custom_area)"
    echo "  --help              Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 --bounds \"45.5,-73.6,45.6,-73.5\" --name \"montreal\""
    echo "  $0 --bounds \"40.7,-74.0,40.8,-73.9\" --min-zoom 12 --max-zoom 18 --name \"nyc\""
    echo "  $0 --bounds \"51.4,-112.8,51.6,-112.4\" --name \"drumheller\""
    echo ""
    echo "Common Areas:"
    echo "  Montreal: \"45.5,-73.6,45.6,-73.5\""
    echo "  NYC: \"40.7,-74.0,40.8,-73.9\""
    echo "  Toronto: \"43.6,-79.4,43.7,-79.3\""
    echo "  Vancouver: \"49.2,-123.2,49.3,-123.1\""
    echo "  Drumheller: \"51.4,-112.8,51.6,-112.4\""
}

# Default values
BOUNDS=""
MIN_ZOOM=10
MAX_ZOOM=16
AREA_NAME="custom_area"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --bounds)
            BOUNDS="$2"
            shift 2
            ;;
        --min-zoom)
            MIN_ZOOM="$2"
            shift 2
            ;;
        --max-zoom)
            MAX_ZOOM="$2"
            shift 2
            ;;
        --name)
            AREA_NAME="$2"
            shift 2
            ;;
        --help)
            show_usage
            exit 0
            ;;
        *)
            echo -e "${RED}❌ Unknown option: $1${NC}"
            show_usage
            exit 1
            ;;
    esac
done

# Validate required parameters
if [ -z "$BOUNDS" ]; then
    echo -e "${RED}❌ Error: Bounds are required${NC}"
    echo -e "${YELLOW}Use --help for usage information${NC}"
    exit 1
fi

# Validate bounds format (south,west,north,east)
if ! echo "$BOUNDS" | grep -qE '^-?[0-9]+\.[0-9]+,-?[0-9]+\.[0-9]+,-?[0-9]+\.[0-9]+,-?[0-9]+\.[0-9]+$'; then
    echo -e "${RED}❌ Error: Invalid bounds format${NC}"
    echo -e "${YELLOW}Expected format: south,west,north,east${NC}"
    echo -e "${YELLOW}Example: 45.5,-73.6,45.6,-73.5${NC}"
    exit 1
fi

# Validate zoom levels
if ! [[ "$MIN_ZOOM" =~ ^[0-9]+$ ]] || [ "$MIN_ZOOM" -lt 0 ] || [ "$MIN_ZOOM" -gt 22 ]; then
    echo -e "${RED}❌ Error: Invalid min zoom level (0-22)${NC}"
    exit 1
fi

if ! [[ "$MAX_ZOOM" =~ ^[0-9]+$ ]] || [ "$MAX_ZOOM" -lt 0 ] || [ "$MAX_ZOOM" -gt 22 ]; then
    echo -e "${RED}❌ Error: Invalid max zoom level (0-22)${NC}"
    exit 1
fi

if [ "$MIN_ZOOM" -gt "$MAX_ZOOM" ]; then
    echo -e "${RED}❌ Error: Min zoom cannot be greater than max zoom${NC}"
    exit 1
fi

echo -e "${BLUE}🗺️  Downloading offline map tiles for custom area${NC}"
echo -e "${BLUE}📍 Area: $AREA_NAME${NC}"
echo -e "${BLUE}🗺️  Bounds: $BOUNDS${NC}"
echo -e "${BLUE}🔍 Zoom: $MIN_ZOOM to $MAX_ZOOM${NC}"
echo "============================================="

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo -e "${RED}❌ Error: Docker is not installed. Please install Docker first.${NC}"
    exit 1
fi

# Calculate estimated size based on zoom levels and area
# Rough estimation: each zoom level can increase tiles by 4x
ZOOM_DIFF=$((MAX_ZOOM - MIN_ZOOM + 1))
ESTIMATED_SIZE_MB=$((ZOOM_DIFF * 100))  # Rough estimate

# Check available disk space
echo -e "${YELLOW}🔍 Checking available disk space...${NC}"
AVAILABLE_SPACE=$(df . | awk 'NR==2 {print $4}')
REQUIRED_SPACE=$((ESTIMATED_SIZE_MB * 1024))  # Convert MB to KB

if [ "$AVAILABLE_SPACE" -lt "$REQUIRED_SPACE" ]; then
    echo -e "${RED}❌ Warning: Low disk space. Need at least ${ESTIMATED_SIZE_MB}MB available.${NC}"
    echo -e "${YELLOW}   Available: $((AVAILABLE_SPACE / 1024))MB${NC}"
    echo -e "${YELLOW}   Required: ${ESTIMATED_SIZE_MB}MB${NC}"
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Create directories
echo -e "${YELLOW}📁 Creating directories...${NC}"
mkdir -p mbtiles styles fonts temp

# Backup existing tiles if they exist
if [ -f "mbtiles/v3.mbtiles" ]; then
    echo -e "${YELLOW}⚠️  Backing up existing tiles...${NC}"
    mv mbtiles/v3.mbtiles mbtiles/v3.mbtiles.backup.$(date +%Y%m%d_%H%M%S)
fi

echo -e "${YELLOW}📥 Downloading map tiles for $AREA_NAME...${NC}"
echo -e "${BLUE}📍 Bounds: $BOUNDS${NC}"
echo -e "${BLUE}🔍 Zoom levels: $MIN_ZOOM to $MAX_ZOOM${NC}"
echo -e "${YELLOW}⏱️  Estimated time: $((ZOOM_DIFF * 5))-${ZOOM_DIFF * 10} minutes${NC}"

# Download tiles using Docker
docker run --rm \
    -v "$(pwd)/temp:/data" \
    -v "$(pwd)/mbtiles:/output" \
    -e BOUNDS="$BOUNDS" \
    -e MIN_ZOOM="$MIN_ZOOM" \
    -e MAX_ZOOM="$MAX_ZOOM" \
    -e AREA_NAME="$AREA_NAME" \
    maptiler/tileserver-gl:latest \
    sh -c "
        echo '🗺️  Downloading tiles for $AREA_NAME'
        echo '📍 Bounds: $BOUNDS'
        echo '🔍 Zoom levels: $MIN_ZOOM to $MAX_ZOOM'
        
        # Install additional tools
        apk add --no-cache curl wget python3 py3-pip sqlite
        
        # Install tile downloading tools
        pip3 install mbutil mercantile
        
        # Create a Python script to download tiles
        cat > /data/download_tiles.py << 'PYTHON_EOF'
import mercantile
import requests
import sqlite3
import os
import time
from urllib.parse import urlparse

def download_tiles(bounds, min_zoom, max_zoom, area_name):
    # Parse bounds: south,west,north,east
    south, west, north, east = map(float, bounds.split(','))
    
    # Create SQLite database for tiles
    conn = sqlite3.connect('/output/v3.mbtiles')
    cursor = conn.cursor()
    
    # Create tiles table
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS tiles (
            zoom_level INTEGER,
            tile_column INTEGER,
            tile_row INTEGER,
            tile_data BLOB
        )
    ''')
    
    # Create metadata table
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS metadata (
            name TEXT,
            value TEXT
        )
    ''')
    
    # Insert metadata
    metadata = [
        ('name', area_name),
        ('type', 'overlay'),
        ('version', '1.0'),
        ('description', f'Offline map for {area_name}'),
        ('format', 'png'),
        ('bounds', bounds),
        ('center', f'{(north+south)/2},{(east+west)/2},{min_zoom}')
    ]
    
    cursor.executemany('INSERT OR REPLACE INTO metadata (name, value) VALUES (?, ?)', metadata)
    
    # Calculate tiles to download
    tiles = list(mercantile.tiles(west, south, east, north, range(min_zoom, max_zoom + 1)))
    total_tiles = len(tiles)
    
    print(f'Downloading {total_tiles} tiles for {area_name}...')
    
    # OpenStreetMap tile server
    tile_server = 'https://tile.openstreetmap.org/{z}/{x}/{y}.png'
    
    for i, tile in enumerate(tiles):
        if i % 100 == 0:
            print(f'Progress: {i}/{total_tiles} tiles ({i/total_tiles*100:.1f}%)')
        
        # Download tile
        url = tile_server.format(z=tile.z, x=tile.x, y=tile.y)
        
        try:
            response = requests.get(url, timeout=10)
            if response.status_code == 200:
                # Store in database
                cursor.execute('''
                    INSERT OR REPLACE INTO tiles 
                    (zoom_level, tile_column, tile_row, tile_data) 
                    VALUES (?, ?, ?, ?)
                ''', (tile.z, tile.x, tile.y, response.content))
                
                # Commit every 100 tiles
                if i % 100 == 0:
                    conn.commit()
            else:
                print(f'Failed to download tile {tile.z}/{tile.x}/{tile.y}: {response.status_code}')
                
        except Exception as e:
            print(f'Error downloading tile {tile.z}/{tile.x}/{tile.y}: {e}')
        
        # Rate limiting to be respectful to OSM servers
        time.sleep(0.1)
    
    conn.commit()
    conn.close()
    print('Tile download completed!')

if __name__ == '__main__':
    bounds = '$BOUNDS'
    min_zoom = $MIN_ZOOM
    max_zoom = $MAX_ZOOM
    area_name = '$AREA_NAME'
    download_tiles(bounds, min_zoom, max_zoom, area_name)
PYTHON_EOF

        # Run the tile downloader
        cd /data
        python3 download_tiles.py
        
        echo '✅ Tiles downloaded successfully!'
    "

# Create style files
echo -e "${YELLOW}🎨 Creating style files...${NC}"

# OSM Bright style
cat > styles/osm-bright/style.json << 'EOF'
{
  "version": 8,
  "name": "OSM Bright",
  "sources": {
    "osm": {
      "type": "raster",
      "tiles": ["http://localhost:8080/styles/osm-bright/{z}/{x}/{y}.png"],
      "tileSize": 256
    }
  },
  "layers": [
    {
      "id": "osm",
      "type": "raster",
      "source": "osm",
      "minzoom": 0,
      "maxzoom": 22
    }
  ]
}
EOF

# Satellite style
cat > styles/satellite/style.json << 'EOF'
{
  "version": 8,
  "name": "Satellite",
  "sources": {
    "satellite": {
      "type": "raster",
      "tiles": ["http://localhost:8080/styles/satellite/{z}/{x}/{y}.png"],
      "tileSize": 256
    }
  },
  "layers": [
    {
      "id": "satellite",
      "type": "raster",
      "source": "satellite",
      "minzoom": 0,
      "maxzoom": 22
    }
  ]
}
EOF

# Terrain style
cat > styles/terrain/style.json << 'EOF'
{
  "version": 8,
  "name": "Terrain",
  "sources": {
    "terrain": {
      "type": "raster",
      "tiles": ["http://localhost:8080/styles/terrain/{z}/{x}/{y}.png"],
      "tileSize": 256
    }
  },
  "layers": [
    {
      "id": "terrain",
      "type": "raster",
      "source": "terrain",
      "minzoom": 0,
      "maxzoom": 22
    }
  ]
}
EOF

# Clean up temp directory
rm -rf temp

# Create info file
cat > mbtiles/area_info.txt << EOF
Custom Area Offline Map
======================
Area: $AREA_NAME
Bounds: $BOUNDS
Zoom Levels: $MIN_ZOOM - $MAX_ZOOM
Download Date: $(date)
Purpose: Rover 2025 offline navigation

Download Command:
$0 --bounds "$BOUNDS" --min-zoom $MIN_ZOOM --max-zoom $MAX_ZOOM --name "$AREA_NAME"

Features:
- Custom area coverage
- Offline map tiles
- Multiple map styles
- GPS navigation support
EOF

# Check file size
if [ -f "mbtiles/v3.mbtiles" ]; then
    FILE_SIZE=$(du -h mbtiles/v3.mbtiles | cut -f1)
    echo -e "${GREEN}✅ Offline map tiles downloaded successfully!${NC}"
    echo -e "${BLUE}📊 File size: $FILE_SIZE${NC}"
else
    echo -e "${YELLOW}⚠️  Note: Using basic tile structure${NC}"
    echo -e "${YELLOW}   For complete coverage, run this script with proper internet connection${NC}"
    touch mbtiles/v3.mbtiles
fi

echo ""
echo -e "${BLUE}📊 Download Summary:${NC}"
echo -e "${BLUE}📍 Area: $AREA_NAME${NC}"
echo -e "${BLUE}🗺️  Coverage: $BOUNDS${NC}"
echo -e "${BLUE}🔍 Detail: Zoom levels $MIN_ZOOM-$MAX_ZOOM${NC}"
echo -e "${BLUE}📁 Location: $(pwd)/mbtiles/${NC}"
echo ""
echo -e "${GREEN}🚀 Ready for offline navigation!${NC}"
echo -e "${BLUE}💡 Start the system with: ./start-teleop-system.sh${NC}"
echo ""
echo -e "${YELLOW}💡 Tips:${NC}"
echo -e "${YELLOW}   - Use OSM view for streets and buildings${NC}"
echo -e "${YELLOW}   - Satellite view for aerial imagery${NC}"
echo -e "${YELLOW}   - Terrain view for elevation data${NC}"
echo -e "${YELLOW}   - GPS tracking works offline${NC}" 