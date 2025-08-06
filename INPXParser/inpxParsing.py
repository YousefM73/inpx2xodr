"""
INPX to OpenDRIVE conversion core module.
Handles parsing INPX files and converting to internal road network representation.
"""

__all__ = ['rNode', 'InpxWay', 'InpxWayEndcap', 'parseInpxAll', 'JunctionRoad']

import json
import pyproj
from pathlib import Path
from math import pi, sin, cos, sqrt, atan2, degrees, radians
import numpy as np
from .utils import *
from .arcCurves import *

# Global variable to store Vissim connector information
vissim_connectors = {}

class InpxNode:
    """Node class for INPX data compatible with OSM format"""
    def __init__(self, id, lat, lon, tags=None):
        self.id = id
        self.lat = lat
        self.lon = lon
        self.tags = tags if tags else {}

class rNode:
    """Node class for internal road network representation"""
    allNodes = {}
    giveNextElementIDVar = 0
    
    @staticmethod
    def giveNextElementID():
        rNode.giveNextElementIDVar += 1
        return rNode.giveNextElementIDVar
    
    def __init__(self, id, longitude, latitude, tags=None):
        self.id = id
        self.longitude = longitude
        self.latitude = latitude
        self.connections = []
        self.Tags = tags if tags else {}
        self.elementID = rNode.giveNextElementID()
        rNode.allNodes[self.id] = self
        
        if 'z_offset' in self.Tags:
            try:
                self.z_offset = float(self.Tags['z_offset'])
            except (ValueError, TypeError):
                self.z_offset = 0.0
        else:
            self.z_offset = 0.0

class InpxPreWay:
    """Intermediate way representation for processing INPX ways"""
    allWays = {}
    
    @staticmethod
    def reset():
        InpxPreWay.allWays = {}
    
    def __init__(self, id, tags, rNodes):
        self.id = id
        self.tags = tags
        self.rNodes = rNodes
        self.type = 'normal'
        
        if len(self.rNodes) >= 2:
            if self.id in InpxPreWay.allWays:
                InpxPreWay.allWays[self.id].append(self)
            else:
                InpxPreWay.allWays[self.id] = [self]
        else:
            InpxPreWay.allWays[self.id] = self
    
    def createInpxWays(self):
        """Convert to final InpxWay objects"""
        # For Vissim links, don't split at junction nodes - keep as single road
        # The junctions will be handled separately through conflict areas
        InpxWay(self.id, self.tags, self.rNodes, self.rNodes[0], self.rNodes[-1])

class InpxWay:
    """Final way representation compatible with OpenDRIVE generation"""
    allWays = {}
    giveNextElementIDVar = 0
    
    @staticmethod
    def giveNextElementID():
        InpxWay.giveNextElementIDVar += 1
        return InpxWay.giveNextElementIDVar
    
    @staticmethod
    def reset():
        InpxWay.allWays = {}
        InpxWay.giveNextElementIDVar = 0
    
    def __init__(self, id, tags, rNodes, startNode, endNode):
        self.id = str(id)
        self.tags = tags
        self.rNodes = rNodes
        self.startNode = startNode
        self.endNode = endNode
        self.XODRNodeList = []
        self.roadElements = []
        self.elevationElements = []
        self.elementID = InpxWay.giveNextElementID()
        self.xodrID = str(id)  # Use original Vissim link ID directly
        
        # Lane information
        self.laneNumberOpposite = 0  # Default: no opposite lanes
        self.laneNumberCurrent = 1
        self.laneNumberDirection = 1
        
        # Junction information
        self.startJunction = -1
        self.endJunction = -1
        
        # Parse lane information from tags - only for non-sidewalks
        if tags.get('is_sidewalk', 'no') == 'no':
            if 'lanes' in tags:
                try:
                    total_lanes = int(tags['lanes'])
                    # Vissim links are unidirectional - all lanes go in one direction
                    self.laneNumberCurrent = total_lanes
                    self.laneNumberDirection = total_lanes
                    self.laneNumberOpposite = 0
                except ValueError:
                    pass
        else:
            # Sidewalks are always single "lane"
            self.laneNumberCurrent = 1
            self.laneNumberDirection = 1
            self.laneNumberOpposite = 0
        
        # Store lane width if available
        self.laneWidth = 3.5  # Default for roads
        if tags.get('is_sidewalk', 'no') == 'yes':
            self.laneWidth = 2.0  # Default for sidewalks
        
        if 'lane_width' in tags:
            try:
                self.laneWidth = float(tags['lane_width'])
            except ValueError:
                pass
        
        self.InpxNodes = [node.id for node in rNodes]
        InpxWay.allWays[self.elementID] = self
        
    def getLaneWidth(self, lane_number=1):
        """Get width for a specific lane number. For now, all lanes have same width."""
        return self.laneWidth
        
    @staticmethod
    def getLaneWidthFromRef(lane_ref):
        """Parse lane reference like '1 1' and return width of that specific lane"""
        try:
            parts = lane_ref.split()
            if len(parts) == 2:
                link_id = parts[0]
                lane_num = int(parts[1])
                
                # Try both string and int keys
                road = InpxWay.allWays.get(link_id)
                if not road:
                    road = InpxWay.allWays.get(int(link_id))
                
                if road:
                    width = road.getLaneWidth(lane_num)
                    print(f"  DEBUG getLaneWidthFromRef: {lane_ref} -> link {link_id}, lane {lane_num} = {width}m")
                    return width
                else:
                    print(f"  DEBUG getLaneWidthFromRef: Road {link_id} not found in allWays")
                    
        except (ValueError, AttributeError) as e:
            print(f"  DEBUG getLaneWidthFromRef: Error parsing {lane_ref}: {e}")
            pass
        return 3.5  # Default fallback
        
        for node in rNodes:
            if self.elementID not in node.connections:
                node.connections.append(self.elementID)

class InpxWayEndcap:
    """End cap representation for ways"""
    allCaps = {}
    
    @staticmethod
    def reset():
        InpxWayEndcap.allCaps = {}
    
    def __init__(self, id, inpxway, endNode, endSide):
        self.id = id
        self.inpxway = inpxway
        self.endNode = endNode
        self.endSide = endSide
        self.XODRNodeList = []
        InpxWayEndcap.allCaps[self.id] = self

class JunctionRoad:
    """Junction road representation"""
    junctionDict = {}
    junctionNodes = {}
    giveNextElementIDVar = 0
    
    @staticmethod
    def giveNextElementID():
        JunctionRoad.giveNextElementIDVar += 1
        return JunctionRoad.giveNextElementIDVar
    
    @staticmethod
    def reset():
        JunctionRoad.junctionDict = {}
        JunctionRoad.junctionNodes = {}
        JunctionRoad.giveNextElementIDVar = 0
    
    @staticmethod
    def giveJunctionDict(junctionNode):
        return JunctionRoad.junctionDict.get(junctionNode.id, {})
    
    @staticmethod
    def createJunctionRoadsBetweenLanes(predecessorway, successorway, junctionNode, from_lane, to_lane, maxerror=0.1):
        """Create junction roads between specific lanes of two ways"""
        contactPointPredecessor = "end"
        contactPointSuccessor = "start"
        roadElements, elevationElements = createInpxJunctionRoadLine(predecessorway, successorway, junctionNode, maxerror=maxerror)
        
        # Create single lane connection based on Vissim connector
        road = JunctionRoad(predecessorway, successorway, from_lane, to_lane, junctionNode, contactPointPredecessor, contactPointSuccessor, roadElements, elevationElements)
        return [road]
    
    @staticmethod
    def createJunctionRoadsBetween(predecessorway, successorway, junctionNode, maxerror=0.1):
        """Create junction roads between two ways"""
        contactPointPredecessor = "start" if predecessorway.InpxNodes[0] == junctionNode.id else "end"
        contactPointSuccessor = "start" if successorway.InpxNodes[0] == junctionNode.id else "end"
        roadElements, elevationElements = createInpxJunctionRoadLine(predecessorway, successorway, junctionNode, maxerror=maxerror)
        
        predecessor2successorLaneConnections = [[1, 1]]
        
        roads = []
        for connection in predecessor2successorLaneConnections:
            roads.append(JunctionRoad(predecessorway, successorway, connection[0], connection[1], junctionNode, contactPointPredecessor, contactPointSuccessor, roadElements, elevationElements))
        return roads

    def __init__(self, predecessorway, successorway, startlane, endlane, junctionNode, contactPointPredecessor, contactPointSuccessor, roadElements, elevationElements):
        self.id = str(predecessorway.id) + "_to_" + str(successorway.id) + "_lane_" + str(startlane) + "_to_" + str(endlane)
        self.xodrID = str(rNode.giveNextElementID())
        junctionDict = JunctionRoad.giveJunctionDict(junctionNode)
        
        if (str(predecessorway.id) + "_to_" + str(successorway.id)) in junctionDict:
            waydic = junctionDict[str(predecessorway.id) + "_to_" + str(successorway.id)]
        else:
            waydic = {}
            junctionDict[str(predecessorway.id) + "_to_" + str(successorway.id)] = waydic
            
        waydic[str(startlane) + "_to_" + str(endlane)] = self
        JunctionRoad.junctionDict[junctionNode.id] = junctionDict

def mercator_to_latlon(x, y):
    """Convert from Mercator projection to lat/lon"""
    transformer = pyproj.Transformer.from_crs("EPSG:3857", "EPSG:4326", always_xy=True)
    x, y = transformer.transform(x, y)
    return x, y

def parse_inpx_file(inpx_file_path):
    """Parse INPX file and return nodes and ways"""
    print(f"Parsing INPX file: {inpx_file_path}")
    
    with open(inpx_file_path, 'r') as file:
        data = json.load(file)

    net_para = data.get('sections', {}).get('netPara', {})
    ref_point_map = net_para.get('children', {}).get('refPointMap', {})
    ref_point_attrs = ref_point_map.get('attributes', {})
    
    ref_x = float(ref_point_attrs.get('x', 0))
    ref_y = float(ref_point_attrs.get('y', 0))

    # Parse links
    links_section = data.get('sections', {}).get('links')
    if not links_section or 'children' not in links_section:
        print("No links found in INPX file")
        return {}, {}, {}
        
    links = links_section['children'].get('link', [])
    if not isinstance(links, list):
        links = [links]
    
    # Parse conflict areas for junction information
    conflict_section = data.get('sections', {}).get('conflictAreas', {})
    connectors = []
    if conflict_section and 'children' in conflict_section:
        conflict_data = conflict_section['children'].get('conflictArea', [])
        if not isinstance(conflict_data, list):
            conflict_data = [conflict_data]
        connectors = conflict_data
    
    nodes_dict = {}
    ways_dict = {}
    connector_dict = {}
    node_id_counter = 1
    
    print(f"Found {len(links)} links to process")
    
    sidewalk_count = 0
    road_count = 0
    
    for link in links:
        link_no = link.get('attributes', {}).get('no', 'unknown')
        link_attrs = link.get('attributes', {})
        
        # Check if this is a sidewalk (displayType 58)
        display_type = link_attrs.get('displayType', '1')
        is_sidewalk = (display_type == '58') or link_attrs.get('isPedArea', 'false') == 'true'
        
        # Extract lane information from Vissim link - check for lanes section
        geometry = link.get('children', {}).get('geometry', {})
        lanes_section = link.get('children', {}).get('lanes', {})
        
        # Count individual lane elements
        num_lanes = 1  # Default
        lane_width = 3.5  # Default (for roads)
        connector_lane_data = {}  # For storing fromLanes/toLanes data
        if is_sidewalk:
            lane_width = 2.0  # Narrower default for sidewalks
        
        if lanes_section and 'children' in lanes_section:
            lane_elements = lanes_section['children'].get('lane', [])
            if not isinstance(lane_elements, list):
                lane_elements = [lane_elements]
            
            num_lanes = len(lane_elements)
            
            # Get width from first lane (assuming all lanes have same width)
            if lane_elements and 'attributes' in lane_elements[0]:
                lane_width = float(lane_elements[0]['attributes'].get('width', 3.5))
                
                # In Vissim, lane width is typically per individual lane
                # Validate that the width is reasonable for a single lane
                if lane_width > 10.0:  # Likely total width, divide by number of lanes
                    lane_width = lane_width / num_lanes
                elif lane_width < 0.5:  # Too narrow, use default
                    lane_width = 3.5 if not is_sidewalk else 2.0
            
            # For connector links, parse fromLanes/toLanes connection data
            connector_lane_data = {}
            for lane_element in lane_elements:
                lane_attrs = lane_element.get('attributes', {})
                lane_children = lane_element.get('children', {})
                
                # Parse fromLanes
                from_lanes = lane_children.get('fromLanes', {})
                if from_lanes:
                    from_refs = from_lanes.get('children', {}).get('objectRef', [])
                    if not isinstance(from_refs, list):
                        from_refs = [from_refs]
                    from_lane_refs = [ref.get('attributes', {}).get('key', '') for ref in from_refs]
                else:
                    from_lane_refs = []
                
                # Parse toLanes
                to_lanes = lane_children.get('toLanes', {})
                if to_lanes:
                    to_refs = to_lanes.get('children', {}).get('objectRef', [])
                    if not isinstance(to_refs, list):
                        to_refs = [to_refs]
                    to_lane_refs = [ref.get('attributes', {}).get('key', '') for ref in to_refs]
                else:
                    to_lane_refs = []
                
                if from_lane_refs or to_lane_refs:
                    connector_lane_data = {
                        'from_lanes': from_lane_refs,
                        'to_lanes': to_lane_refs
                    }
                    # Debug connector lane parsing
                    if link_no in ['10000', '10001', '10002']:
                        print(f"DEBUG CONNECTOR LANES - Link {link_no}: from={from_lane_refs}, to={to_lane_refs}")
        else:
            # Fallback to old attribute-based parsing
            num_lanes = int(link_attrs.get('numLanes', 1))
            parsed_width = float(link_attrs.get('laneWidth', 3.5))
            
            # Check if this might be total width vs individual lane width
            if num_lanes > 1 and parsed_width > 6.0:  # Likely total width
                lane_width = parsed_width / num_lanes
            else:
                lane_width = parsed_width
        
        # Skip non-sidewalk links with width less than 1 meter
        if not is_sidewalk and lane_width < 1.0:
            continue
        
        link_poly_pts = geometry.get('children', {}).get('linkPolyPts', {})
        link_poly_points = link_poly_pts.get('children', {}).get('linkPolyPoint', [])
        
        if not isinstance(link_poly_points, list):
            link_poly_points = [link_poly_points]
        
        link_node_ids = []
        
        for i, point in enumerate(link_poly_points):
            attributes = point.get('attributes', {})
            x = ref_x + float(attributes.get('x', 0))
            y = ref_y + float(attributes.get('y', 0))
            z_offset = float(attributes.get('zOffset', 0))
            
            lon, lat = mercator_to_latlon(x, y)
            
            node_tags = {
                "vissim_link": link_no,
                "point_index": str(i),
                "z_offset": str(z_offset),
                "highway": "footway" if is_sidewalk else "primary",
                "is_sidewalk": "yes" if is_sidewalk else "no",
                "display_type": display_type
            }
            
            node = InpxNode(node_id_counter, lat, lon, node_tags)
            nodes_dict[node_id_counter] = node
            link_node_ids.append(node_id_counter)
            node_id_counter += 1
        
        if len(link_node_ids) > 1:
            way_tags = {
                "vissim_link": link_no,
                "highway": "footway" if is_sidewalk else "primary",
                "lanes": str(num_lanes),
                "lane_width": str(lane_width),
                "oneway": "yes",  # Vissim links are typically unidirectional
                "is_sidewalk": "yes" if is_sidewalk else "no",
                "display_type": display_type
            }
            
            # Add connector lane data if this is a connector link
            if connector_lane_data:
                way_tags['connector_lanes'] = str(connector_lane_data)
            
            # Debug: Report sidewalks and multi-lane roads during parsing (only show a few)
            if is_sidewalk:
                sidewalk_count += 1
                if sidewalk_count <= 5:
                    print(f"Found sidewalk: Link {link_no} (displayType: {display_type}, width: {lane_width}m)")
            else:
                road_count += 1
                if num_lanes > 1 and road_count <= 10:
                    print(f"Found multi-lane road: Link {link_no} has {num_lanes} lanes (width: {lane_width}m)")
            
            way = InpxNode.__new__(type('InpxWay', (), {}))
            way.id = link_no
            way.nd = link_node_ids
            way.tags = way_tags
            ways_dict[link_no] = way

        progress = int((links.index(link)) / len(links) * 100)
        if progress % 10 == 0:
            print(f'Progress: {progress}%', end='\r')

    # Parse conflict areas for junction information
    print("\nProcessing conflict areas...")
    for conflict in connectors:
        conflict_attrs = conflict.get('attributes', {})
        conflict_no = conflict_attrs.get('no', 'unknown')
        
        # Get the two links that conflict
        link_a = conflict_attrs.get('linkA')
        link_b = conflict_attrs.get('linkB')
        lane_a = conflict_attrs.get('laneA', '1')
        lane_b = conflict_attrs.get('laneB', '1')
        
        if link_a and link_b:
            connector_dict[conflict_no] = {
                'from_link': link_a,
                'to_link': link_b,
                'from_lane': lane_a if lane_a else '1',
                'to_lane': lane_b if lane_b else '1',
                'type': 'conflict'
            }

    print(f'Progress: 100%')
    print(f"Created {len(nodes_dict)} nodes, {len(ways_dict)} ways, {len(connector_dict)} connectors")
    print(f"Detected {sidewalk_count} sidewalks and {road_count} roads")
    
    return nodes_dict, ways_dict, connector_dict

def parseInpxAll(inpxFile, minimumHeight=0.0, maximumHeight=100.0, curveRadius=12.0):
    """Main function to parse INPX file and convert to internal representation"""
    print(f"Starting INPX parsing: {inpxFile}")
    
    # Reset all static data
    rNode.allNodes = {}
    rNode.giveNextElementIDVar = 0
    InpxPreWay.reset()
    InpxWay.reset()
    InpxWayEndcap.reset()
    JunctionRoad.reset()
    
    nodes_dict, ways_dict, connector_dict = parse_inpx_file(inpxFile)
    
    if not nodes_dict:
        print("No nodes found in INPX file")
        return
    
    # Store connector information globally for junction creation
    global vissim_connectors
    vissim_connectors = connector_dict
    
    # Set coordinate reference based on first node
    first_node = list(nodes_dict.values())[0]
    from .utils import referenceLat, referenceLon
    if referenceLat is None or referenceLon is None:
        import INPXParser.utils as utils
        utils.referenceLat = first_node.lat
        utils.referenceLon = first_node.lon
        print(f"Set coordinate reference: lat={first_node.lat:.6f}, lon={first_node.lon:.6f}")
    
    print("Converting nodes...")
    for node_id, inpx_node in nodes_dict.items():
        rNode(node_id, inpx_node.lon, inpx_node.lat, inpx_node.tags)
    
    print("Converting ways...")
    for way_id, inpx_way in ways_dict.items():
        way_rnodes = []
        for node_id in inpx_way.nd:
            if node_id in rNode.allNodes:
                way_rnodes.append(rNode.allNodes[node_id])
        
        if len(way_rnodes) >= 2:
            InpxPreWay(way_id, inpx_way.tags, way_rnodes)
    
    print(f"Created {len(InpxPreWay.allWays)} pre-ways from {len(ways_dict)} input ways")
    
    print("Creating connections from conflict data...")
    createConnectionsFromConflicts()
    
    print("Connecting continuous roads...")
    connectContinuousLinks()
    
    print("Processing way topology...")
    for preways in InpxPreWay.allWays.values():
        if isinstance(preways, list):
            for preway in preways:
                preway.createInpxWays()
        else:
            preways.createInpxWays()
    
    print(f"Final InpxWay count: {len(InpxWay.allWays)} ways")
    
    # Debug: Check road ID mappings 
    print("Road ID mapping verification:")
    road_id_samples = []
    sidewalk_samples = []
    
    for way in InpxWay.allWays.values():
        link_id = way.tags.get('vissim_link', 'unknown')
        is_sidewalk = way.tags.get('is_sidewalk', 'no') == 'yes'
        
        if is_sidewalk:
            if len(sidewalk_samples) < 5:
                sidewalk_samples.append(f"  Sidewalk: Vissim Link {link_id} -> XODR Road {way.xodrID}, width: {way.laneWidth}m")
        else:
            if len(road_id_samples) < 10:
                road_id_samples.append(f"  Road: Vissim Link {link_id} -> XODR Road {way.xodrID}, lanes: {way.laneNumberDirection}")
    
    for sample in road_id_samples:
        print(sample)
    for sample in sidewalk_samples:
        print(sample)
    
    # Check specific problematic links
    for way in InpxWay.allWays.values():
        link_id = way.tags.get('vissim_link', 'unknown')
        if link_id in ['10091', '10211']:
            print(f"  FOUND PROBLEM LINK: Vissim {link_id} -> XODR {way.xodrID}, sidewalk: {way.tags.get('is_sidewalk', 'no')}")
    
    print(f"✓ All roads preserve original Vissim link IDs as XODR road IDs")
    
    # print("Creating junction roads...")
    # createJunctionRoads()
    
    print("Creating end caps...")
    createEndCaps()
    
    print("Generating geometry...")
    for way in InpxWay.allWays.values():
        createInpxWayNodeList2XODRRoadLine(way, curveRadius=curveRadius)
    
    for cap in InpxWayEndcap.allCaps.values():
        createEndCap(cap, curveRadius=curveRadius)
    
    print(f"INPX parsing complete. Created {len(rNode.allNodes)} nodes, {len(InpxWay.allWays)} ways")

    # Create junction roads 
    createJunctionRoads()
    
    junction_count = sum(len(junction_roads) for junction_roads in JunctionRoad.junctionNodes.values())
    print(f"Created {junction_count} junction roads")

def createJunctionRoads():
    """Create junction roads based on Vissim connectors"""
    global vissim_connectors
    
    junction_roads_created = 0
    for conn_id, connector in vissim_connectors.items():
        from_link = connector['from_link']
        to_link = connector['to_link']
        from_lane = int(connector['from_lane'])
        to_lane = int(connector['to_lane'])
        
        # Find the corresponding ways
        from_way = None
        to_way = None
        
        for way in InpxWay.allWays.values():
            if way.tags.get('vissim_link') == from_link:
                from_way = way
            elif way.tags.get('vissim_link') == to_link:
                to_way = way
        
        if from_way and to_way:
            # Create junction road with specific lane connections
            junction_node = from_way.endNode  # Approximate junction location
            JunctionRoad.createJunctionRoadsBetweenLanes(from_way, to_way, junction_node, from_lane, to_lane)
            junction_roads_created += 1
    
    print(f"Created {junction_roads_created} junction roads from {len(vissim_connectors)} connectors")

def createConnectionsFromConflicts():
    """Create connections between ways based on conflict areas"""
    global vissim_connectors
    
    # Create a mapping of link IDs to ways
    link_to_way = {}
    for way_id, preway in InpxPreWay.allWays.items():
        if isinstance(preway, list):
            preway = preway[0]
        link_id = preway.tags.get('vissim_link')
        if link_id:
            link_to_way[link_id] = preway
    
    # Connect ways based on conflict areas
    connected_pairs = set()
    for conn_id, connector in vissim_connectors.items():
        from_link = connector['from_link']
        to_link = connector['to_link']
        
        # Avoid duplicate connections
        pair = tuple(sorted([from_link, to_link]))
        if pair in connected_pairs:
            continue
        connected_pairs.add(pair)
        
        from_way = link_to_way.get(from_link)
        to_way = link_to_way.get(to_link)
        
        if from_way and to_way and from_way != to_way:
            # Connect the end of from_way to the start of to_way
            from_end_node = from_way.rNodes[-1]
            to_start_node = to_way.rNodes[0]
            
            # Add cross-references
            if to_way.id not in from_end_node.connections:
                from_end_node.connections.append(to_way.id)
            if from_way.id not in to_start_node.connections:
                to_start_node.connections.append(from_way.id)

def connectContinuousLinks():
    """Connect links that are geometrically continuous (no conflict areas needed)"""
    # Create a mapping of link IDs to ways
    link_to_way = {}
    for way_id, preway in InpxPreWay.allWays.items():
        if isinstance(preway, list):
            preway = preway[0]
        link_id = preway.tags.get('vissim_link')
        if link_id:
            link_to_way[link_id] = preway
    
    # For each way, find nearby ways that could be connected
    ways_list = list(link_to_way.values())
    connection_threshold = 5.0  # Maximum distance in meters to consider a connection
    continuous_connections = 0
    
    for i, way1 in enumerate(ways_list):
        for j, way2 in enumerate(ways_list):
            if i >= j or way1 == way2:  # Avoid self-connections and duplicates
                continue
            
            # Check if these ways are already connected via conflict areas
            already_connected = False
            for node in way1.rNodes:
                if way2.id in node.connections:
                    already_connected = True
                    break
            
            if already_connected:
                continue
            
            # Check distance between endpoints
            connections_to_make = []
            
            # Check way1 end to way2 start
            end1 = way1.rNodes[-1]
            start2 = way2.rNodes[0]
            distance = calculateDistance(end1.latitude, end1.longitude, start2.latitude, start2.longitude)
            if distance < connection_threshold:
                connections_to_make.append((end1, start2, way1, way2))
            
            # Check way1 start to way2 start
            start1 = way1.rNodes[0]
            distance = calculateDistance(start1.latitude, start1.longitude, start2.latitude, start2.longitude)
            if distance < connection_threshold:
                connections_to_make.append((start1, start2, way1, way2))
            
            # Check way1 end to way2 end
            end2 = way2.rNodes[-1]
            distance = calculateDistance(end1.latitude, end1.longitude, end2.latitude, end2.longitude)
            if distance < connection_threshold:
                connections_to_make.append((end1, end2, way1, way2))
            
            # Check way1 start to way2 end
            distance = calculateDistance(start1.latitude, start1.longitude, end2.latitude, end2.longitude)
            if distance < connection_threshold:
                connections_to_make.append((start1, end2, way1, way2))
            
            # Make the closest connection
            if connections_to_make:
                # Sort by distance and take the closest
                connections_to_make.sort(key=lambda x: calculateDistance(x[0].latitude, x[0].longitude, x[1].latitude, x[1].longitude))
                node1, node2, w1, w2 = connections_to_make[0]
                
                # Add cross-references
                if w2.id not in node1.connections:
                    node1.connections.append(w2.id)
                if w1.id not in node2.connections:
                    node2.connections.append(w1.id)
                continuous_connections += 1
    
    print(f"Created {continuous_connections} continuous road connections")

def calculateDistance(lat1, lon1, lat2, lon2):
    """Calculate distance between two lat/lon points in meters"""
    R = 6371000  # Earth's radius in meters
    lat1_rad = radians(lat1)
    lat2_rad = radians(lat2)
    delta_lat = radians(lat2 - lat1)
    delta_lon = radians(lon2 - lon1)
    
    a = sin(delta_lat/2)**2 + cos(lat1_rad) * cos(lat2_rad) * sin(delta_lon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    distance = R * c
    
    return distance

def createEndCaps():
    """Create end caps for ways that don't connect to junctions"""
    for way in InpxWay.allWays.values():
        if len(way.startNode.connections) <= 1:
            InpxWayEndcap(f"{way.id}_start", way, way.startNode, "start")
        
        if len(way.endNode.connections) <= 1:
            InpxWayEndcap(f"{way.id}_end", way, way.endNode, "end")

def createInpxWayNodeList2XODRRoadLine(inpxWay, curveRadius=12.0):
    """Create XODR geometry for an INPX way"""
    inpxWay.XODRNodeList = createInpxWayNodeList2XODRRoadLineInternal(inpxWay.rNodes, curveRadius)
    
    inpxWay.roadElements = []
    inpxWay.elevationElements = []
    
    if len(inpxWay.XODRNodeList) >= 2:
        for i in range(len(inpxWay.XODRNodeList) - 1):
            curr = inpxWay.XODRNodeList[i]
            next_node = inpxWay.XODRNodeList[i + 1]
            
            dx = next_node['x'] - curr['x']
            dy = next_node['y'] - curr['y']
            length = sqrt(dx*dx + dy*dy)
            
            road_element = {
                "xstart": curr['x'],
                "ystart": curr['y'],
                "heading": curr['heading'],
                "length": length,
                "curvature": 0.0
            }
            inpxWay.roadElements.append(road_element)
            
            elevation_element = {
                "length": length,
                "zstart": curr['z'],
                "steigung": (next_node['z'] - curr['z']) / length if length > 0 else 0.0
            }
            inpxWay.elevationElements.append(elevation_element)

def createInpxWayNodeList2XODRRoadLineInternal(rNodes, curveRadius=12.0):
    """Internal function to create XODR road line from rNodes"""
    if len(rNodes) < 2:
        return []
    
    xodr_nodes = []
    for i, node in enumerate(rNodes):
        x, y = convertLongitudeLatitude(node.longitude, node.latitude)
        
        xodr_node = {
            'x': x,
            'y': y,
            'z': getattr(node, 'z_offset', 0.0),
            'heading': 0.0
        }
        
        xodr_nodes.append(xodr_node)
    
    for i in range(len(xodr_nodes) - 1):
        curr = xodr_nodes[i]
        next_node = xodr_nodes[i + 1]
        
        dx = next_node['x'] - curr['x']
        dy = next_node['y'] - curr['y']
        heading = atan2(dy, dx)
        curr['heading'] = heading
    
    if len(xodr_nodes) > 1:
        xodr_nodes[-1]['heading'] = xodr_nodes[-2]['heading']
    
    return xodr_nodes

def createInpxJunctionRoadLine(predecessorway, successorway, junctionNode, maxerror=0.1):
    """Create junction road geometry between two ways"""
    roadElements = []
    elevationElements = []
    
    # Get the actual endpoints of the roads that need to be connected
    # Find which end of predecessor connects to junction
    pred_start_node = predecessorway.rNodes[0]
    pred_end_node = predecessorway.rNodes[-1]
    
    # Find which end of successor connects to junction  
    succ_start_node = successorway.rNodes[0]
    succ_end_node = successorway.rNodes[-1]
    
    # Determine connection points based on proximity to junction node
    pred_junction_node = pred_end_node  # Assume end connects to junction
    succ_junction_node = succ_start_node  # Assume start connects to junction
    
    # Check if we need to use different endpoints
    pred_start_dist = calculateDistance(pred_start_node.latitude, pred_start_node.longitude, 
                                      junctionNode.latitude, junctionNode.longitude)
    pred_end_dist = calculateDistance(pred_end_node.latitude, pred_end_node.longitude,
                                    junctionNode.latitude, junctionNode.longitude)
    
    if pred_start_dist < pred_end_dist:
        pred_junction_node = pred_start_node
    
    succ_start_dist = calculateDistance(succ_start_node.latitude, succ_start_node.longitude,
                                      junctionNode.latitude, junctionNode.longitude)
    succ_end_dist = calculateDistance(succ_end_node.latitude, succ_end_node.longitude,
                                    junctionNode.latitude, junctionNode.longitude)
    
    if succ_end_dist < succ_start_dist:
        succ_junction_node = succ_end_node
    
    # Create junction road connecting the road endpoints
    pred_x, pred_y = convertLongitudeLatitude(pred_junction_node.longitude, pred_junction_node.latitude)
    succ_x, succ_y = convertLongitudeLatitude(succ_junction_node.longitude, succ_junction_node.latitude)
    
    # Calculate connection geometry
    dx = succ_x - pred_x
    dy = succ_y - pred_y
    length = sqrt(dx*dx + dy*dy)
    heading = atan2(dy, dx)
    
    # Use minimum length to avoid zero-length segments
    if length < 0.1:
        length = 0.1
    
    pred_z = getattr(pred_junction_node, 'z_offset', 0.0)
    succ_z = getattr(succ_junction_node, 'z_offset', 0.0)
    
    roadElements.append({
        'type': 'line',
        'x': pred_x,
        'y': pred_y,
        'hdg': heading,
        'length': length
    })
    
    elevation_slope = (succ_z - pred_z) / length if length > 0 else 0.0
    elevationElements.append({
        'a': pred_z,
        'b': elevation_slope,
        'c': 0.0,
        'd': 0.0
    })
    
    return roadElements, elevationElements

def createEndCap(endcap, curveRadius=12.0):
    """Create end cap geometry"""
    endcap.XODRNodeList = []
    
    node = endcap.endNode
    x, y = convertLongitudeLatitude(node.longitude, node.latitude)
    z = getattr(node, 'z_offset', 0.0)
    
    endcap.XODRNodeList.append({
        'x': x,
        'y': y,
        'z': z,
        'heading': 0.0
    })
