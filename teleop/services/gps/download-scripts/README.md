# Offline Map Download Scripts

This folder contains scripts to download offline map tiles for various areas. These scripts create `.mbtiles` files that can be served by TileServer-GL for offline navigation.

## Available Scripts

### 🏫 McGill University Campus
- **`download-mcgill-basic.sh`** - Basic McGill campus tiles (quick setup)
- **`download-mcgill-complete.sh`** - Complete McGill campus coverage with real tiles

### 🏜️ Drumheller Badlands (Desert Testing)
- **`download-drumheller-tiles.sh`** - Basic Drumheller area (quick setup)
- **`download-drumheller-complete.sh`** - Complete Drumheller coverage with real tiles

### 🌍 Custom Areas
- **`download-custom-area.sh`** - Download tiles for any custom area worldwide

## Quick Start

### For McGill Campus
```bash
# Quick setup (basic tiles)
./download-mcgill-basic.sh

# Complete coverage (real tiles)
./download-mcgill-complete.sh
```

### For Drumheller Badlands
```bash
# Quick setup (basic tiles)
./download-drumheller-tiles.sh

# Complete coverage (real tiles)
./download-drumheller-complete.sh
```

### For Any Custom Area
```bash
# Download tiles for any area
./download-custom-area.sh --bounds "45.5,-73.6,45.6,-73.5" --name "montreal"

# With custom zoom levels
./download-custom-area.sh --bounds "40.7,-74.0,40.8,-73.9" --min-zoom 12 --max-zoom 18 --name "nyc"
```

## Common Area Coordinates

### Canadian Cities
- **Montreal**: `45.5,-73.6,45.6,-73.5`
- **Toronto**: `43.6,-79.4,43.7,-79.3`
- **Vancouver**: `49.2,-123.2,49.3,-123.1`
- **Calgary**: `51.0,-114.1,51.1,-114.0`
- **Edmonton**: `53.5,-113.6,53.6,-113.5`

### US Cities
- **New York**: `40.7,-74.0,40.8,-73.9`
- **Los Angeles**: `34.0,-118.3,34.1,-118.2`
- **Chicago**: `41.8,-87.7,41.9,-87.6`
- **Boston**: `42.3,-71.1,42.4,-71.0`

### Testing Areas
- **Drumheller Badlands**: `51.4,-112.8,51.6,-112.4`
- **McGill Campus**: `45.5,-73.58,45.52,-73.56`

## Script Features

### Basic Scripts (Quick Setup)
- ✅ **Fast download** - 5-15 minutes
- ✅ **Small file size** - 100-500MB
- ✅ **Basic coverage** - Essential areas only
- ✅ **Good for testing** - Quick validation

### Complete Scripts (Full Coverage)
- ✅ **Real tiles** - Actual OpenStreetMap data
- ✅ **Comprehensive coverage** - Full area details
- ✅ **Multiple zoom levels** - Detailed navigation
- ✅ **Production ready** - Full offline capability

### Custom Area Script
- ✅ **Any location** - Worldwide coverage
- ✅ **Configurable zoom** - Custom detail levels
- ✅ **Flexible bounds** - Any rectangular area
- ✅ **Smart validation** - Error checking

## Usage Examples

### Download Montreal Area
```bash
./download-custom-area.sh --bounds "45.5,-73.6,45.6,-73.5" --name "montreal"
```

### Download NYC with High Detail
```bash
./download-custom-area.sh --bounds "40.7,-74.0,40.8,-73.9" --min-zoom 12 --max-zoom 18 --name "nyc"
```

### Download Small Test Area
```bash
./download-custom-area.sh --bounds "45.5048,-73.5772,45.5148,-73.5672" --min-zoom 14 --max-zoom 16 --name "test_area"
```

## File Sizes and Times

| Area Type | Zoom Levels | Size | Time |
|-----------|-------------|------|------|
| Small campus | 12-16 | ~500MB | 10-20 min |
| Medium city | 10-16 | ~2GB | 20-40 min |
| Large area | 10-18 | ~5GB | 30-60 min |
| High detail | 12-18 | ~8GB | 45-90 min |

## Requirements

- **Docker** - Required for tile processing
- **Internet** - For downloading tiles
- **Disk Space** - 500MB to 8GB depending on area
- **Time** - 10 minutes to 2 hours depending on area

## Output

All scripts create:
- **`mbtiles/v3.mbtiles`** - Main tile database
- **`styles/`** - Map style files
- **`mbtiles/area_info.txt`** - Area information
- **Backup files** - Previous tiles backed up automatically

## Integration

After downloading tiles:
1. **Start TileServer**: `docker-compose -f ../docker-compose.tileserver.yml up -d`
2. **Start System**: `cd ../../.. && ./start-teleop-system.sh`
3. **Access Maps**: Navigate to the "Navigation" page in the UI

## Troubleshooting

### Common Issues
- **Docker not running** - Start Docker Desktop
- **Low disk space** - Free up space or reduce zoom levels
- **Slow download** - Use smaller areas or lower zoom levels
- **Network errors** - Check internet connection

### Debug Commands
```bash
# Check Docker
docker --version

# Check disk space
df -h

# Check TileServer
curl http://localhost:8080/
```

## Tips

- **Start small** - Test with basic scripts first
- **Plan ahead** - Download tiles before going offline
- **Backup tiles** - Keep copies of important areas
- **Use appropriate zoom** - Higher zoom = more detail but larger files
- **Respect OSM** - Scripts include rate limiting to be respectful

---

**Note**: These scripts download tiles from OpenStreetMap servers. Please respect their usage policies and rate limits. 