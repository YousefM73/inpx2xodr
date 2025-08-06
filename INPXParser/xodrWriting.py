"""OpenDRIVE XML file generation functions."""

__all__ = ['writeXODR', 'startBasicXODRFile', 'fillNormalRoads', 'fillJunctionRoads']

from math import floor, pi
import numpy as np
from .utils import giveHeading, distance, schnittpunkt, getXYPositionFromLineLength, giveReferences
from .arcCurves import giveHeading, getArcEndposition, distance, schnittpunkt, getArcCurvatureAndLength, getXYPositionFromLineLength, getArcCurvatureAndLength2Point, endTurn2LaneStreet
from .inpxParsing import rNode, InpxWay, JunctionRoad, InpxWayEndcap

def writeXODR(path='output.xodr'):
    """Main function to write complete XODR file"""
    startBasicXODRFile(path)
    fillNormalRoads(path)
    fillJunctionRoads(path)

def startBasicXODRFile(path='Test.xodr'):
    referenceLon, referenceLat, topoParameter = giveReferences()
    xmin, xmax, ymin, ymax = topoParameter
    with open(path,'w',encoding='utf-8') as f:
        f.write('''<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="4" name="" version="1" date="2019-02-18T13:36:12" north="{0}" south="{1}" east="{2}" west="{3}">
    <geoReference><![CDATA[+proj=tmerc +lat_0={4} +lon_0={5} +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs]]></geoReference>
    </header>
    <!-- Roads -->
    <!-- nextRoad -->
    <!-- Junctions -->
    <!-- nextJunction -->
</OpenDRIVE>
    '''.format(ymax-ymin, 0.0, xmax-xmin, 0.0, referenceLat, referenceLon))

def fillNormalRoads(path = 'Test.xodr'):
    fillNormalRoads.connector_debug_count = 0  # Initialize debug counter
    filedata = ""
    with open(path,'r',encoding='utf-8') as file:
          filedata = file.read()
    parts = filedata.split("<!-- nextRoad -->")
    
    # Debug: Check for connector links in allWays
    connector_count = 0
    for road_id, road in InpxWay.allWays.items():
        if hasattr(road, 'tags') and 'connector_lanes' in road.tags:
            connector_count += 1
            if connector_count <= 3:
                print(f"DEBUG: Found connector in allWays - Road {road_id}, lanes data: {road.tags.get('connector_lanes', 'N/A')}")
    
    print(f"DEBUG: Total connectors found in allWays: {connector_count}")
    
    for road in InpxWay.allWays.values():
        # create geometry
        geometry = ""
        lengths = []
        for element in road.roadElements:
            lengths.append(element["length"])
            geometry += '''
            <geometry s="{0}" x="{1}" y="{2}" hdg="{3}" length="{4}">'''.format(sum(lengths[:-1]), element["xstart"],
                                                                               element["ystart"], element["heading"],
                                                                               element["length"])+('''
                <line/>''' if element["curvature"] == 0.0 else '''
                <arc curvature="{0}"/>'''.format(element["curvature"])) + '''
            </geometry>'''
        lengths = []
        elevation = ""
        # create elevation
        for element in road.elevationElements:
            lengths.append(element["length"])
            elevation += '''
            <elevation s="{0}" a="{1}" b="{2}" c="0.0" d="0.0"/>'''.format(sum(lengths[:-1]),element["zstart"], element["steigung"])

        name = "Road "+ str(road.xodrID)
        try: name = road.tags["name"]
        except: pass
        maxspeed = "30"
        try: maxspeed = road.tags["maxspeed"]
        except: pass
        #add road string
        leftlanes = ""
        leftlanenumber = 1
        
        # Determine if this is a sidewalk
        is_sidewalk = False
        if hasattr(road, 'tags'):
            is_sidewalk = road.tags.get('is_sidewalk', 'no') == 'yes'
        
        lane_type = "sidewalk" if is_sidewalk else "driving"
        
        # Use Vissim width data directly - trust the source data
        individual_lane_width = road.laneWidth
        
        # Check if this is a connector road with lane connection data
        is_connector = hasattr(road, 'tags') and 'connector_lanes' in road.tags
        if hasattr(road, 'tags') and road.tags.get('vissim_link') == '10139':
            print(f"DEBUG 10139: is_connector = {is_connector}, has tags = {hasattr(road, 'tags')}")
            if hasattr(road, 'tags'):
                print(f"DEBUG 10139: tags = {road.tags}")
        
        if is_connector:
            try:
                import ast
                connector_data = ast.literal_eval(road.tags['connector_lanes'])
                
                # Get width from fromLanes (start of connector)
                start_width = 3.5
                if 'from_lanes' in connector_data and connector_data['from_lanes']:
                    from_ref = connector_data['from_lanes'][0]  # Use first lane reference
                    start_width = InpxWay.getLaneWidthFromRef(from_ref)
                
                # Get width from toLanes (end of connector)  
                end_width = 3.5
                if 'to_lanes' in connector_data and connector_data['to_lanes']:
                    to_ref = connector_data['to_lanes'][0]  # Use first lane reference
                    end_width = InpxWay.getLaneWidthFromRef(to_ref)
                
                # Use average of start and end widths for connector
                individual_lane_width = (start_width + end_width) / 2.0
                
                # Debug output for connectors connecting to/from wide roads
                if hasattr(road, 'tags'):
                    link_id = road.tags.get('vissim_link', 'unknown')
                    # Debug specific connector 10139 and any wide road connectors
                    if link_id == '10139' or start_width > 5.0 or end_width > 5.0 or start_width != end_width:
                        print(f"CONNECTOR ROAD DEBUG - Link {link_id}: from_width={start_width:.2f}m, to_width={end_width:.2f}m, avg={individual_lane_width:.2f}m")
                        print(f"  From: {connector_data.get('from_lanes', [])}, To: {connector_data.get('to_lanes', [])}")
                        if link_id == '10139':
                            print(f"  DEBUG 10139: connector_data = {connector_data}")
                            print(f"  DEBUG 10139: original road.laneWidth = {road.laneWidth}")
                        
            except Exception as e:
                if hasattr(road, 'tags') and road.tags.get('vissim_link') == '10139':
                    print(f"ERROR processing connector 10139: {e}")
                print(f"Warning: Error processing connector lane data for road {road.xodrID}: {e}")
        
        # Only apply fallbacks for clearly invalid data
        if individual_lane_width <= 0.0:  # Invalid/missing data
            individual_lane_width = 2.0 if is_sidewalk else 3.5
        elif individual_lane_width < 0.5:  # Extremely narrow, likely invalid
            individual_lane_width = 2.0 if is_sidewalk else 3.5
        
        # Only add left lanes if there are any (opposite direction)
        for i in range(road.laneNumberOpposite):
            leftlanes += '''
                        <lane id="{0}" type="{2}" level="false">
                                        <link>
                                        </link>
                                        <width sOffset="0.0" a="{1:.2f}" b="0.0" c="0.00" d="0.00"/>
                        </lane>'''.format(leftlanenumber, individual_lane_width, lane_type)
            leftlanenumber += 1
            
        rightlanes = ""
        rightlanenumber = -1
        # Add right lanes (main direction)
        for i in range(road.laneNumberDirection):
            rightlanes += '''
                        <lane id="{0}" type="{2}" level="false">
                                        <link>
                                        </link>
                                        <width sOffset="0.0" a="{1:.2f}" b="0.0" c="0.00" d="0.00"/>
                        </lane>'''.format(rightlanenumber, individual_lane_width, lane_type)
            rightlanenumber -= 1

        # Calculate lane offset to properly position road when no left lanes exist
        lane_offset = 0.0
        if road.laneNumberOpposite == 0 and road.laneNumberDirection > 0:
            # When no left lanes, offset the centerline to position right lanes correctly
            # Offset by half the total width of right lanes to center the road properly
            total_right_width = road.laneNumberDirection * individual_lane_width
            lane_offset = total_right_width / 2.0

        # Debug output for specific roads
        if hasattr(road, 'tags'):
            link_id = road.tags.get('vissim_link', 'unknown')
            if link_id in ['10091', '10211', '81', '182', '183'] or (road.laneNumberDirection > 1 and int(link_id) <= 20):
                print(f"DEBUG Lane Width - Link {link_id}: original_vissim={road.laneWidth:.2f}m, final={individual_lane_width:.2f}m, lanes={road.laneNumberDirection}, type={lane_type}, offset={lane_offset:.2f}m")
            
            # Special debugging for wider roads to verify they're preserved
            if individual_lane_width > 5.0:
                print(f"WIDE ROAD DEBUG - Road {link_id}: opposite={road.laneNumberOpposite}, direction={road.laneNumberDirection}")
                print(f"  Vissim width: {road.laneWidth:.2f}m, Final width: {individual_lane_width:.2f}m")
                print(f"  Lane offset: {lane_offset:.2f}m")
                print(f"  Left lanes: {len(leftlanes.split('<lane id='))-1}, Right lanes: {len(rightlanes.split('<lane id='))-1}")

        parts[0] +='''
        <road name="{0}" length="{1}" id="{2}" junction="-1">
            <link>
                <predecessor elementType="junction" elementId="{3}"/>
                <successor elementType="junction" elementId="{4}"/>
            </link>'''.format(name, sum(lengths), road.xodrID, road.startJunction, road.endJunction)+'''
        <type s="0.0" type="town">
             <speed max="{0}" unit="mph"/>
        </type>
             <planView>'''.format(maxspeed) + geometry +'''
             </planView>

        <elevationProfile>''' + elevation + '''
        </elevationProfile>
             <lanes>
                <laneOffset s="0.0" a="{0:.3f}" b="0.0" c="0.0" d="0.0"/>
                <laneSection s="0.0">
                    <left>'''.format(lane_offset)+leftlanes+'''
                    </left>
                    <center>
                        <lane id="0" type="none" level="false">
                        </lane>
                    </center>
                    <right>'''+rightlanes+'''
                    </right>
                </laneSection>
            </lanes>
        </road>
        '''
    with open(path,'w',encoding='utf-8') as f:
        f.write("<!-- nextRoad -->".join(parts))


def fillJunctionRoads(path = 'Test.xodr'):
    print(f"DEBUG: Starting fillJunctionRoads, JunctionRoad.junctionNodes keys: {len(JunctionRoad.junctionNodes.keys()) if JunctionRoad.junctionNodes else 0}")
    filedata = ""
    with open(path,'r',encoding='utf-8') as file:
          filedata = file.read()
    parts = filedata.split("<!-- nextRoad -->")
    secondsplits = parts[1].split("<!-- nextJunction -->")
    parts[1] = secondsplits[0]
    parts.append(secondsplits[1])
    
    junction_count = 0
    for junction in JunctionRoad.junctionNodes.keys():
        junction_count += 1
        # create junction start
        parts[1] += '''
        <junction id="{0}" name="{1}">'''.format(str(junction),"junction "+str(junction))
        connectionID = 1
        for roadkey in JunctionRoad.junctionNodes[junction].keys():
            incomingRoad,outgoingRoad = roadkey.split("_to_")
            for lanekey in JunctionRoad.junctionNodes[junction][roadkey].keys():
                    fromLane,toLane = lanekey.split("_to_")
                    road = JunctionRoad.junctionNodes[junction][roadkey][lanekey]
                    
                    # Debug first few connectors
                    if junction_count <= 3:
                        print(f"DEBUG JUNCTION {junction}: Processing connector {roadkey}, lane {lanekey}, road_id={road.xodrID}")
                    #create connection
                    parts[1] += '''
                    <connection id="{0}" incomingRoad="{1}" connectingRoad="{2}" contactPoint="{3}">
                        <laneLink from="{4}" to="{5}"/>
                    </connection>'''.format(connectionID, incomingRoad, road.xodrID, "start",
                                           fromLane, "-1")
                    connectionID +=1

                    #create road
                    geometry = ""
                    lengths = []
                    for element in road.roadElements:
                        lengths.append(element["length"])
                        geometry += '''
                        <geometry s="{0}" x="{1}" y="{2}" hdg="{3}" length="{4}">'''.format(sum(lengths[:-1]), element["xstart"],
                                                                                           element["ystart"], element["heading"],
                                                                                           element["length"])+('''
                            <line/>''' if element["curvature"] == 0.0 else '''
                            <arc curvature="{0}"/>'''.format(element["curvature"])) + '''
                        </geometry>'''
                    lengths = []
                    elevation = ""
                    # create elevation
                    for element in road.elevationElements:
                        lengths.append(element["length"])
                        elevation += '''
                        <elevation s="{0}" a="{1}" b="{2}" c="0.0" d="0.0"/>'''.format(sum(lengths[:-1]),element["zstart"], element["steigung"])

                    # Get connector width from fromLanes/toLanes lane references
                    connector_width = 3.5  # Default fallback
                    start_width = 3.5
                    end_width = 3.5
                    
                    # Check if this road has connector lane data (fromLanes/toLanes)
                    if hasattr(road, 'tags') and 'connector_lanes' in road.tags:
                        try:
                            import ast
                            connector_data = ast.literal_eval(road.tags['connector_lanes'])
                            
                            # Get width from fromLanes (start of connector)
                            if 'from_lanes' in connector_data and connector_data['from_lanes']:
                                from_ref = connector_data['from_lanes'][0]  # Use first lane reference
                                start_width = InpxWay.getLaneWidthFromRef(from_ref)
                            
                            # Get width from toLanes (end of connector)  
                            if 'to_lanes' in connector_data and connector_data['to_lanes']:
                                to_ref = connector_data['to_lanes'][0]  # Use first lane reference
                                end_width = InpxWay.getLaneWidthFromRef(to_ref)
                            
                            # Use average of start and end widths for connector
                            connector_width = (start_width + end_width) / 2.0
                            
                            # Debug output for connectors with lane references
                            if start_width > 5.0 or end_width > 5.0:
                                print(f"CONNECTOR LANE DEBUG - {roadkey}: from_width={start_width:.2f}m, to_width={end_width:.2f}m, avg={connector_width:.2f}m")
                                
                        except Exception as e:
                            print(f"Warning: Error parsing connector lane data for {roadkey}: {e}")
                    
                    # Fallback: try to get width from incoming road if no lane data
                    if connector_width == 3.5:  # Still default, try road-level width
                        try:
                            incoming_road_obj = InpxWay.allWays.get(incomingRoad)
                            if incoming_road_obj:
                                connector_width = incoming_road_obj.laneWidth
                        except Exception as e:
                            print(f"Warning: Error getting width for connector from road {incomingRoad}: {e}")

                    name = "JunctionConnection "+ roadkey + " lane "+lanekey
                    maxspeed = "30"
                    parts[0] +='''
        <road name="{0}" length="{1}" id="{2}" junction="{3}">
            <link>
                <predecessor elementType="road" elementId="{4}" contactPoint="{6}"/>
                <successor elementType="road" elementId="{5}" contactPoint="{7}"/>
            </link>'''.format(name, sum(lengths), road.xodrID, junction, incomingRoad, outgoingRoad,
                             road.contactPointPredecessor, road.contactPointSuccessor)+'''
        <type s="0.0" type="town">
             <speed max="{0}" unit="mph"/>
        </type>
             <planView>'''.format(maxspeed) + geometry +'''
             </planView>

        <elevationProfile>''' + elevation + '''
        </elevationProfile>
             <lanes>
                <laneOffset s="0.0" a="{0}" b="{1}" c="0.0" d="0.0"/>'''.format(road.laneOffsetA, road.laneOffsetB) + '''
                <laneSection s="0.0">
                     <center>
                        <lane id="0" type="none" level="false">
                        </lane>
                    </center>
                    <right>
                        <lane id="-1" type="driving" level="false">
                            <link>
                                <predecessor id="{0}"/>
                                <successor id="{1}"/>
                            </link>
                            <width sOffset="0.0" a="{0:.2f}" b="0.0" c="0.00" d="0.00"/>
                        </lane>
                    </right>
                </laneSection>
            </lanes>
        </road>
        '''.format(fromLane,toLane,connector_width)
        #close junction
        parts[1] += '''
        </junction>
        '''
    parts[0] = "<!-- nextRoad -->".join([parts[0],parts[1]])
    whole = "<!-- nextJunction -->".join([parts[0],parts[2]])
    
    print(f"DEBUG: Processed {junction_count} junctions in fillJunctionRoads")

    with open(path,'w',encoding='utf-8') as f:
            f.write(whole)
