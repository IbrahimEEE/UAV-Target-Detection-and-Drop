# This is the whole mission code used with simulation.
# Look at the mission.pdf documentation.

from mavsdk import System
import numpy as np
import asyncio
import cv2
import haversine as hs
from haversine import Unit
import time

#----------------------------------------------------------------------------
async def run():
    # init the drone
    aldebaran22 = System()
    await aldebaran22.connect(system_address="udp://:14540")
    
    print("connected to aldebaran22\n")
    
    # record a video, take a photo when red is seen
    cam = cv2.VideoCapture(0)
    
    width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

    my_fourcc = cv2.VideoWriter_fourcc(*'XVID')
    writer = cv2.VideoWriter("v6MissionOnSimulation.avi",my_fourcc,50,(width,height))

    lower1 = np.array([0,159,168])      # red color ranges in lab color space
    upper1 = np.array([255,255,255])    
    lower2 = np.array([0,179,148])
    upper2 = np.array([255,255,255])

    print_searching_for_red_circle_count = 20
    first_detection_flag = 0
    while first_detection_flag == 0:
        
        if print_searching_for_red_circle_count % 20 == 0:
            print("searching for Red Circle...")
        
        print_searching_for_red_circle_count += 1
        
        ret, image = cam.read()

        writer.write(image)
        
        imageLab = cv2.cvtColor(image,cv2.COLOR_BGR2Lab)
        
        image_lab_mask1 = cv2.inRange(imageLab,lower1,upper1)   # there are two masks for better capture
        image_lab_mask2 = cv2.inRange(imageLab,lower2,upper2)
        image_lab_mask_total = image_lab_mask1 + image_lab_mask2
        
        # until a circle is detected, we continue to search for
        circles=cv2.HoughCircles(image_lab_mask_total, cv2.HOUGH_GRADIENT, 2,
        image_lab_mask_total.shape[0] / 8, param1=100, param2=40, minRadius=10, maxRadius=60)

        if circles is not None:
            print("\nRed Circle Detected...")
            
            first_detection_flag = 1
            
            circles = np.round(circles[0, :]).astype("int") # we draw the circle we found for
            circle_center=(circles[0, 0], circles[0, 1])    # analysis
            circle_radius=circles[0, 2]

            cv2.circle(image, circle_center,circle_radius, color=(255, 0, 0), thickness=2)

            cv2.imwrite("v6FirstDetection.jpg",image)   # we save the picture that we found the circle
            cv2.imwrite("v6FirstDetectionMask.jpg",image_lab_mask_total)

            # when circle is detected, we go for the task
            asyncio.ensure_future(determine_shooting_range(aldebaran22))
            
        cv2.waitKey(20)

        cv2.imshow("mask",image_lab_mask_total)
        cv2.imshow("cam",image)
        
#----------------------------------------------------------------------------     
async def determine_shooting_range(aldebaran22):
    
    print("\nShooting Range is Determining..")
    
    uav_speed = 0

    uav_altitude_at_target_location = 0 # location of the target 
    target_latitude = 0
    target_longitude = 0
        
    vehicle_altiude = 0 # location of the uav
    vehicle_latitude = 0
    vehicle_longitude = 0
        
    shooting_distance = 0 # required distance to release the load.
    distance_to_target = 0 # current distance to target
        
    gps_info_count = 0 # our uav in simulation gives 50 gps values per second

    activated_servo_count = 0 # we use servo for releasing two loads
                            # by activating servo in two different angles.
                            # there are just prints because this code is for the simulation
    
    async for velocity in aldebaran22.telemetry.velocity_ned():
        
        # this is the uav speed at the target location.
        uav_speed = ((velocity.east_m_s * velocity.east_m_s + velocity.north_m_s * velocity.north_m_s)**0.5)
        
        async for position in aldebaran22.telemetry.position():
            
            if gps_info_count == 0: # If the uav is just flying above the target (first detection),
                                 # assign current gps data to target location parameters.
                                
                uav_altitude_at_target_location = position.relative_altitude_m
                target_latitude = position.latitude_deg
                target_longitude = position.longitude_deg
                
                task_delay = 1 # we give some time to components (ex: servo) for their work
                shooting_distance = uav_speed * ((2*uav_altitude_at_target_location/9.81)**0.5 + task_delay)
                
                print("\nShooting Range is Determined..")
                print("UAV Altitude at Target Location: "+str(uav_altitude_at_target_location))
                print("Target Latitude: "+str(target_latitude))
                print("Target Longitude: "+str(target_longitude))
                print("Shooting Distance: "+str(shooting_distance))
                print("Waiting for 8 seconds... (50 gps info per sec)")
                gps_info_count += 1                   
                continue                        # we must release the load in the next tours.
                                                # so, we does not follow first 400 gps values because
            elif gps_info_count < 401:          # we are also in the shooting range now.
            
                vehicle_altiude = position.relative_altitude_m
                vehicle_latitude = position.latitude_deg
                vehicle_longitude = position.longitude_deg
                
                distance_to_target = hs.haversine((target_latitude,target_longitude),(vehicle_latitude,vehicle_longitude),unit=Unit.METERS)
                
                if gps_info_count == 1 or gps_info_count % 50 == 0: # we print some information for
                    print("\ngpsInfoCount: ",gps_info_count)        # analysis.
                    print("\n"+time.asctime())
                    print("Unused Vehicle Altitude: ",vehicle_altiude)
                    print("Unused Vehicle Latitude: ",vehicle_latitude)
                    print("Unused Vehicle Longitude: ",vehicle_longitude)
                    print("Distance to Target",distance_to_target)
                    print("Shooting Range: ",shooting_distance)
                    print("Remaining Meters: ",(distance_to_target-shooting_distance))
                
                gps_info_count += 1
                continue
            
            else:
                vehicle_altiude = position.relative_altitude_m
                vehicle_latitude = position.latitude_deg
                vehicle_longitude = position.longitude_deg
                
                distance_to_target = hs.haversine((target_latitude,target_longitude),(vehicle_latitude,vehicle_longitude),unit=Unit.METERS)
                
                if gps_info_count == 1 or gps_info_count % 50 == 0:
                    print("\ngpsInfoCount: ",gps_info_count)
                    print("\n"+time.asctime())
                    print("Vehicle Altitude: ",vehicle_altiude)
                    print("Vehicle Latitude: ",vehicle_latitude)
                    print("Vehicle Longitude: ",vehicle_longitude)
                    print("Distance to Target",distance_to_target)
                    print("Shooting Range: ",shooting_distance)
                    print("Remaining Meters: ",(distance_to_target-shooting_distance))
                
                gps_info_count += 1
            
            if distance_to_target <= shooting_distance:       # when we are in the shooting range,
                print("\nThe field has been entered..")       # we activate the servo
                
                if activated_servo_count == 0:
                    print("Shoot !!! : 1")
                    activated_servo_count += 1
                    gps_info_count = 1
                    
                elif activated_servo_count == 1:
                    print("Shoot !!! : 2")
                    print("Mission is done ! Terminating...")
                    exit(0)
                    
#----------------------------------------------------------------------------      
if __name__ == "__main__":
    
    # Start the main function
    asyncio.ensure_future(run())

    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()