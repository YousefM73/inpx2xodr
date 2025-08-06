#!/usr/bin/env python3
"""INPX to OpenDRIVE Converter"""

import argparse
import sys
import os
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from INPXParser.inpxParsing import parseInpxAll
from INPXParser.xodrWriting import writeXODR

def serialize_element(element):
    """Convert XML element to dict structure"""
    result = {}
    
    if element.attrib:
        result['attributes'] = element.attrib
    
    if element.text and element.text.strip():
        result['text'] = element.text.strip()
    
    children = {}
    for child in element:
        child_name = child.tag
        child_data = serialize_element(child)
        
        if child_name in children:
            if not isinstance(children[child_name], list):
                children[child_name] = [children[child_name]]
            children[child_name].append(child_data)
        else:
            children[child_name] = child_data
    
    if children:
        result['children'] = children
    
    return result

def parse_inpx_to_json(inpx_file):
    """Parse INPX XML file and convert to JSON structure"""
    try:
        tree = ET.parse(inpx_file)
        root = tree.getroot()
        
        sections = {}
        for child in root:
            section_name = child.tag
            section_data = serialize_element(child)
            
            if section_name in sections:
                if not isinstance(sections[section_name], list):
                    sections[section_name] = [sections[section_name]]
                sections[section_name].append(section_data)
            else:
                sections[section_name] = section_data
        
        return {'sections': sections}
        
    except ET.ParseError as e:
        raise ValueError(f"Error parsing INPX XML: {e}")
    except FileNotFoundError:
        raise FileNotFoundError(f"INPX file not found: {inpx_file}")

def main():
    parser = argparse.ArgumentParser(description='Convert INPX files to OpenDRIVE format')
    parser.add_argument('input', help='Input INPX file path')
    parser.add_argument('-o', '--output', help='Output XODR file path')
    parser.add_argument('--radius', type=float, default=12.0, help='Curve radius (default: 12.0)')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.input):
        print(f"Error: File not found: {args.input}")
        return 1
    
    if not args.input.lower().endswith('.inpx'):
        print(f"Error: Input must be .inpx file")
        return 1
    
    output_file = args.output or str(Path(args.input).with_suffix('.xodr'))
    
    try:
        print(f"Converting {args.input} to {output_file}...")
        
        json_data = parse_inpx_to_json(args.input)
        
        temp_json = args.input + '.temp.json'
        with open(temp_json, 'w') as f:
            json.dump(json_data, f)
        
        parseInpxAll(temp_json, curveRadius=args.radius)
        writeXODR(output_file)
        
        # Keep temp file for debugging
        # os.remove(temp_json)
        
        if os.path.exists(output_file):
            size = os.path.getsize(output_file)
            print(f"✓ Success! Created {output_file} ({size} bytes)")
        else:
            print("✗ Error: Output file not created")
            return 1
            
    except Exception as e:
        print(f"Error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

