#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions

import time
import math
import datetime
import pyproj

vehicle = connect('/dev/ttyUSB0', wait_ready=True, baud=115200)


def ArmAndTakeoffMeter(aTargetAltitude):
    
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    print(f"Takeoff: {aTargetAltitude}")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt , end="      \r")
        
        CurrentAltitude = vehicle.location.global_relative_frame.alt
        
        if CurrentAltitude >aTargetAltitude-0.1: #Trigger just below target alt.
            print(" Altitude: ", vehicle.location.global_relative_frame.alt ) 
            print("Reached target altitude")
            break
        time.sleep(0.001)


def Land():
    vehicle.mode = VehicleMode("LAND")
    while True:
        ALT = vehicle.location.global_relative_frame.alt
        print(" Altitude: ", ALT , end="      \r")
        if(ALT < 0.1):
            break
    print("Landed Altitude: ", ALT )
        
def ModdedCondition_yaw(heading, relative=False):
    print("---Yaw---")
    degree = vehicle.attitude.yaw* 180 / math.pi
    
    if(degree < 0):
            degree = degree + 360
    
    CurrentDegree=degree
    
    print("Yaw: ",heading)
    
    if(heading>(360-0.000000000001)):
        heading = 0
    
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    
    offest = heading - CurrentDegree
    
    if(offest > 0):
        D = 1
    if(offest < 0):
        D = -1
        
    if(offest ==0):
        D = 0
        
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        D,   # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    
    End=time.time()+10
    while(1):
    # send command to vehicle
        vehicle.send_mavlink(msg)
        time.sleep(0.001)
        
        degree = vehicle.attitude.yaw * 180 / math.pi
        
        if(degree < 0):
            degree = degree + 360
        
        print(degree,end="      \r")
        
        if(heading + 0.1 > degree and degree > heading - 0.1):
            Logger(degree)
            break
        
        if(time.time()>End):
            Logger("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
            Logger("Yaw timeout")
            Logger(degree)
            break
        
    print("---------")

def Get_3DDistance(aLocation1, aLocation2):
    
    R = 6371000  # Radius of Earth in meters
    lon1, lat1, lon2, lat2 = map(math.radians, [aLocation1.lon, aLocation1.lat, aLocation2.lon, aLocation2.lat])

    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

def Modded_goto(dNorth, dEast, Altitude):
    print(f"Modded_goto",dNorth, dEast, Altitude)
    proj = pyproj.Transformer.from_crs(3857, 4326, always_xy=True)
    dLatitude, dLongitude = proj.transform(dNorth ,dEast)
    
    print(f"offset {dLatitude} {dLongitude}")
    
    CurrentLocation = vehicle.location.global_relative_frame
    
    TargetLatitude  = CurrentLocation.lat + dLatitude
    TargetLongitude = CurrentLocation.lon + dLongitude
    TargetAltitude  = CurrentLocation.alt + Altitude
        
    targetLocation = LocationGlobalRelative(TargetLatitude,TargetLongitude, TargetAltitude)
    
    LastTarget[0]=targetLocation
        
    print(f"C_Location:{CurrentLocation}")
    print(f"T_Location:{targetLocation}")
    
    vehicle.mode = VehicleMode("GUIDED")
    
    while(not vehicle.mode.name=="GUIDED"):
        print(f"Not in GUIDED mode {vehicle.mode.name}")
    
    vehicle.simple_goto(targetLocation)
    
    Start=time.time()
    End=Start+10
    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.

        OffsetLAT,OffsetLON,OffsetALT,D = LD()
        
        print("{:f}".format(OffsetLAT),"{:f}".format(OffsetLON),"{:f}".format(OffsetALT) , end="      \r")
        
        if(abs(OffsetLAT)<0.000001 and abs(OffsetLON)<0.000001 and abs(OffsetALT)<0.01):
            Logger("Reached target  ")
            Logger(f"C_Location:{vehicle.location.global_relative_frame}")
            Logger(f"T_Location:{targetLocation}")
            break
        
        if(time.time()>End):
            Logger("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
            Logger("goto timeout")
            Logger(f"C_Location:{vehicle.location.global_relative_frame}")
            Logger(f"T_Location:{targetLocation}")
            break

    Logger(f"Use time {time.time() - Start}")
    if(not vehicle.mode.name=="GUIDED"):    
        print(f"Not in GUIDED mode {vehicle.mode.name}")
    
    print("-----------")
    print(LD(1))
    
def LD(Log=0):

    CurrentLocation = vehicle.location.global_relative_frame
    
    CurrentLatitude  = CurrentLocation.lat 
    CurrentLongitude = CurrentLocation.lon 
    CurrentAltitude  = CurrentLocation.alt 
    
    LastTargetLatitude  = LastTarget[0].lat 
    LastTargetLongitude = LastTarget[0].lon 
    LastTargetAltitude  = LastTarget[0].alt
    
    OLAT=CurrentLatitude-LastTargetLatitude
    OLON=CurrentLongitude-LastTargetLongitude
    OALT=CurrentAltitude-LastTargetAltitude
    
    R = 6371000  # Radius of Earth in meters
    lon1, lat1, lon2, lat2 = map(math.radians, [CurrentLongitude, CurrentLatitude, LastTargetLongitude, LastTargetLatitude])

    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    if Log == 1:
        Logger(f"{OLAT},{OLON},{OALT},{distance}")
    return OLAT,OLON,OALT,distance


def Logger(Text):
    print(Text)
    F=open("LoggerDistance.txt","a")
    F.write(str(datetime.datetime.now())+" "+str(Text)+"\n")
    F.close()


ArmAndTakeoffMeter(5)
LastTarget=[vehicle.location.global_relative_frame]

print("test 0 90 180 270 360 ")
ModdedCondition_yaw(0)
ModdedCondition_yaw(90)
ModdedCondition_yaw(180)
ModdedCondition_yaw(270)
ModdedCondition_yaw(360)

Modded_goto(0, 0, 0)
ModdedCondition_yaw(0)
print(LD(1))

for DM in list([1.5,1,0.5,0.1,0.05,0.04,0.03,0.02,0.01]):
    Logger(f"test {DM} m")
    
    Modded_goto( DM,   0, 0)
    Modded_goto(-DM,   0, 0)
    time.sleep(1)
    
    Modded_goto(   0, DM, 0)
    Modded_goto(   0,-DM, 0)
    time.sleep(1)
    
    Modded_goto(0, 0,  DM)
    Modded_goto(0, 0, -DM)
    time.sleep(1)

print("Landing .....")
Land()

print("Close vehicle object")
vehicle.close()

print("Check 'Logger Distance.txt' ")

