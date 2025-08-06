# inpx2xodr

A Python tool for converting Vissim INPX files to OpenDRIVE (.xodr) format with elevation data support.

Originally forked from [JHMeusener's osm2xodr](https://github.com/JHMeusener/osm2xodr), this tool has been enhanced to support both OSM and INPX input formats with special focus on preserving z-offset elevation data from Vissim networks.

## Features

- **INPX Support**: Direct parsing of Vissim INPX files
- **OSM Support**: Original OSM file parsing (maintained)
- **Elevation Preservation**: Z-offset data from INPX files is preserved throughout the conversion
- **Coordinate Transformation**: Automatic conversion from Mercator projection to WGS84
- **OpenDRIVE Generation**: Creates valid OpenDRIVE XML files with roads, junctions, and elevation profiles

## Installation

1. Clone this repository:
```bash
git clone https://github.com/your-username/inpx2xodr.git
cd inpx2xodr
```

2. Install required dependencies:
```bash
pip install -r requirements.txt
```

Note: If you encounter issues with `osmread` installation, you can still use INPX functionality. The OSM features require the osmread package.

## Usage

### INPX to OpenDRIVE Conversion

```bash
python main.py input_file.inpx -o output.xodr
```

### OSM to OpenDRIVE Conversion

```bash
python main.py input_file.osm -o output.xodr
```

### Advanced Options

```bash
python main.py input_file.inpx -o output.xodr --min-height 0.0 --max-height 100.0 --curve-radius 15.0
```

**Parameters:**
- `--min-height`: Minimum height for elevation mapping (default: 0.0)
- `--max-height`: Maximum height for elevation mapping (default: 100.0)
- `--curve-radius`: Default curve radius for road geometry (default: 12.0)

## INPX File Format

The tool expects INPX files in the standard Vissim JSON format with the following structure:

```json
{
  "sections": {
    "netPara": {
      "children": {
        "refPoint": {"attributes": {"x": "0.0", "y": "0.0"}},
        "refPointNet": {"attributes": {"x": "0.0", "y": "0.0"}}
      }
    },
    "links": {
      "children": {
        "link": [
          {
            "attributes": {"no": "1", "name": "Road Name"},
            "children": {
              "geometry": {
                "children": {
                  "linkPolyPts": {
                    "children": {
                      "linkPolyPoint": [
                        {"attributes": {"x": "1000.0", "y": "2000.0", "zOffset": "0.0"}},
                        {"attributes": {"x": "1100.0", "y": "2000.0", "zOffset": "1.5"}}
                      ]
                    }
                  }
                }
              }
            }
          }
        ]
      }
    }
  }
}
```

## Key Features for INPX Processing

1. **Z-Offset Preservation**: The tool extracts and preserves `zOffset` values from INPX link points
2. **Coordinate System Handling**: Converts from Mercator projection (EPSG:3857) used in Vissim to WGS84 (EPSG:4326)
3. **Road Network Topology**: Creates proper OpenDRIVE road networks from Vissim link structures
4. **Elevation Profiles**: Integrates elevation data into OpenDRIVE elevation profiles

## Testing

Test the INPX parsing functionality:

```bash
python test_inpx_only.py
```

This will create a sample INPX file and verify that:
- INPX files are parsed correctly
- Coordinate transformation works
- Z-offset data is preserved
- Nodes and ways are created properly

## File Structure

- `main.py`: Main entry point with command-line interface
- `OSMParser/inpxParsing.py`: INPX file parser and coordinate transformation
- `OSMParser/osmParsing.py`: Core conversion logic (enhanced for INPX support)
- `OSMParser/xodrWriting.py`: OpenDRIVE XML generation
- `OSMParser/utils.py`: Utility functions for coordinate and geometric operations

## Dependencies

- `pyproj`: Coordinate system transformations
- `numpy`: Numerical operations
- `Pillow`: Image processing (for topographic data)
- `osmread`: OSM file parsing (optional, only needed for OSM files)

## Output

The tool generates OpenDRIVE (.xodr) files with:
- Road network geometry
- Junction definitions
- Lane information
- Elevation profiles (when z-offset data is available)
- Proper OpenDRIVE XML structure

## License

This project maintains the original license from the forked repository.