'''
Spawn points : Town 03
Initial Spawn Point Location : 97
Route Indices : [97,71,130,263,55,176,166,36,33,156,169,43,119,205,143,139] // Not working
[263,198,86,166,32,154,197,6,4,170,45,116,227,117,199,142,74,138,96,70,133,156]
______________________________________________________________________________
SPAWN POINT:TOWN04 
Initial Spawn Point Location : 93
Route Indices : [93,131,135,333,302,122,110,106,322,13,21,368,301,332,155,184,52,199,320]
roun about = [337,269,131,135,127,297,306,110,292,273]
__________________________________________________________________________________

SPAWN POINT : TOWN05
Bridge:
Initaal_spawn : 235
Route Indices = [235,272,155,255,195,265]
##Half Way Indices = [[237,8,143,47,219,118,251,181]]
1st : [237,8,143,47,219,118,251,181,117,229,82,47,214,229,82,224,193,48,159,269,263,189] // inside Town
2dn : [224,193,48,159,269,263,189] // Move around Bridge 
_______________________________________________________________________________________________   
'''


import glob
from multiprocessing.synchronize import SemLock
import os
import sys
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import time
import numpy as np
import cv2

IM_WIDTH = 640  
IM_HEIGHT = 480

laneInvasion = False
collsionDetection = False

imageArray = []
def process_img(image):
    # print('Frame_Number',image.frame)
    imageArray.append(image.frame)
    i = np.array(image.raw_data)
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    i3 = i2[:, :, :3]
    cv2.imshow("", i3)
    cv2.waitKey(1)
    # print('Vehicle is cruising at {}'.format(vehicle.get_speed_limit()))
    # print("another steerin",vehicle.get_control().steer)
    if vehicle.is_at_traffic_light():
        traffic_light = vehicle.get_traffic_light()
        if traffic_light.get_state() == carla.TrafficLightState.Red:
            traffic_light.set_state(carla.TrafficLightState.Green)
    
        # print('Vehicle is cruising at {}'.format(vehicle.get_speed_limit()))
        # print("another steerin",vehicle.get_control().steer)
    # cv2.imwrite('Town04/%08d.jpg' % (
    #                 image.frame), i3)
    # image.save_to_disk('_out/%08d' % image.frame)
    last_steer =  str(vehicle.get_control().steer)
    frame_no = str(image.frame)
    text_value = frame_no +'.jpg'+ "," + last_steer
    print('Steering Angle :',vehicle.get_control().steer)
    # with open('C:/CARLA_Latest/WindowsNoEditor/PythonAPI/examples/_out/{}.txt'.format(frame_no),'w') as f:
    #      f.write(text_value)
    # with open('T
    # own04/ground_truth.txt','a') as f:
    #      f.write('\n')
    #      f.write(text_value)

    return i3/255.0

def laneInvasionWarning(event):
    print("Crossing Lane Marking:",str(event.crossed_lane_markings))

def collisionWarning(event):
    actor_type = get_actor_display_name(event.other_actor)
    print(('Collision with %r' % actor_type))
    # actor_we_collide_against = event.other_actor
    # impulse = event.normal_impulse
    # intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

actor_list = []
try:
    client = carla.Client('127.0.0.1', 2000)
    client.load_world('Town04')
    print(client.get_available_maps())

    client.set_timeout(0.5)

    world = client.get_world()
    settings = world.get_settings()

    blueprint_library = world.get_blueprint_library()

    bp = blueprint_library.filter('model3')[0]
    print(bp)

    spawn_point = random.choice(world.get_map().get_spawn_points())
    print('spanw',spawn_point)
    dummy = spawn_point.location
    print('dummy',dummy)
    # vehicle = world.spawn_actor(bp, spawn_point)
    traffic_manager = client.get_trafficmanager()

    ###Trajectory Setting
    spectator = world.get_spectator()
    # route_2_indices = [21, 105, 52, 104, 141, 109, 13, 80, 0, 104, 140, 10, 149,302,333,292,110,306]
    #______________________________Indices for Town 04_______________________________________
    route_2_indices = [337,269,131,135,127,297,306,110,292,273]
        # 237,8,143,47,219,118,251,181,117,229,82,47,214,229,82,224,193,48,159,269,263,189]

    route_2 = []
    spawn_points = world.get_map().get_spawn_points()
    print()
    for i, spawn_point in enumerate(spawn_points):
        world.debug.draw_string(spawn_point.location, str(i), life_time=1500)   
    for ind in route_2_indices:
        route_2.append(spawn_points[ind].location)

    for ind in route_2_indices:
        spawn_points[ind].location
        world.debug.draw_string(spawn_points[ind].location, str(ind), life_time=1500, color=carla.Color(0,0,255))
    spawn_point_2 = spawn_points[337]
    world.debug.draw_string(spawn_point_2.location, 'Spawn point 2', life_time=300, color=carla.Color(0,0,255))
    #Spawn Point for Town 04
    rfc = spawn_points[337]
    vehicle = world.spawn_actor(bp, rfc)

    # vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=1.0))
    vehicle.set_autopilot(True)  # if you just wanted some NPCs to drive.
    #http://carla.org/Doxygen/html/df/d5a/classcarla_1_1traffic__manager_1_1TrafficManager.html
    # traffic_manager.random_left_lanechange_percentage(vehicle, 0)
    # traffic_manager.random_right_lanechange_percentage(vehicle, 0)
    traffic_manager.auto_lane_change(vehicle,True)
    traffic_manager.set_path(vehicle, route_2)
    traffic_manager.set_desired_speed(vehicle,30)

 
    # physics_control = vehicle.get_physics_control()   
    # wheels = physics_control.wheels
    # max_steering_angle = wheels[0].max_steer_angle
    # angle = vehicle.get_control().steer*max_steering_angle
  
    print("Speed Limit",vehicle.get_speed_limit())
    print("traffic_light",vehicle.get_traffic_light())
    print('settings',settings)

    ###____________________######
    ####Steering angle Prediction
    control = carla.VehicleControl()
    print(control.steer,"steer angle")

    print("another steerin",vehicle.get_control().steer)

    #To get the Traffic_light from the vehicle :
    if vehicle.is_at_traffic_light():
        traffic_light = vehicle.get_traffic_light()
        print("traffic_light", traffic_light.get_state())

    actor_list.append(vehicle)

    # https://carla.readthedocs.io/en/latest/cameras_and_sensors
    # get the blueprint for this sensor
    blueprint = blueprint_library.find('sensor.camera.rgb')
    laneBlueprint = blueprint_library.find('sensor.other.lane_invasion')
    collisionBlueprint = blueprint_library.find('sensor.other.collision')
    
    
    # change the dimensions of the image
    blueprint.set_attribute('image_size_x', f'{IM_WIDTH}')
    blueprint.set_attribute('image_size_y', f'{IM_HEIGHT}')
    #Setting the Field of view to 90 degrees
    blueprint.set_attribute("fov",str(110)) 

    # Adjust sensor relative to vehicle
    spawn_point = carla.Transform(carla.Location(x=2.5, z=0.7))

    # spawn the sensor and attach to vehicle.
    sensor = world.spawn_actor(blueprint,  spawn_point, attach_to=vehicle)
    laneSensor = world.spawn_actor(laneBlueprint,  spawn_point ,attach_to=vehicle)
    collisionSensor = world.spawn_actor(collisionBlueprint, spawn_point, attach_to=vehicle)

    # add sensor to list of actors
    actor_list.append(sensor)
    actor_list.append(laneSensor)
    actor_list.append(collisionSensor)
    # measurements, sensor_data = client.read_data()
    # print('measurements',measurements)
    # print('sensor_data',sensor_data)
    '''
    Setting the weather to clear Noon :
    http://carla.org/Doxygen/html/db/ddb/classcarla_1_1rpc_1_1WeatherParameters.html
    '''
    world.set_weather(carla.WeatherParameters.ClearNoon)
    
    # carla.WeatherParameters(cloudiness=0)
    # do something with this sensor
    sensor.listen(lambda image: process_img(image))
    laneSensor.listen(lambda event: laneInvasionWarning(event))
    collisionSensor.listen(lambda event: collisionWarning(event))
    # sensor.listen(lambda image : image.save_to_disk('out7%06d.png'% image.frame))
    print('loop')
    print('counter',len(imageArray))
    time.sleep(1200)


finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')