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


def process_img(image):
    # print('Frame_Number',image.frame)
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
    
    image.save_to_disk('_out/%08d' % image.frame)
    last_steer =  str(vehicle.get_control().steer)
    frame_no = str(image.frame)
    text_value = frame_no +'.jpg'+ " " + last_steer
    with open('C:/CARLA_Latest/WindowsNoEditor/PythonAPI/examples/_out/{}.txt'.format(frame_no),'w') as f:
        f.write(text_value)


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
    client.set_timeout(0.5)

    world = client.get_world()
    settings = world.get_settings()

    blueprint_library = world.get_blueprint_library()

    bp = blueprint_library.filter('model3')[0]
    print(bp)

    spawn_point = random.choice(world.get_map().get_spawn_points())

    vehicle = world.spawn_actor(bp, spawn_point)
   
    # vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=1.0))
    vehicle.set_autopilot(True)  # if you just wanted some NPCs to drive.

 
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
    blueprint.set_attribute("fov",str(105))

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
    '''
    world.set_weather(carla.WeatherParameters.ClearNoon)
    
    # carla.WeatherParameters(cloudiness=0)
    # do something with this sensor
    sensor.listen(lambda image: process_img(image))
    laneSensor.listen(lambda event: laneInvasionWarning(event))
    collisionSensor.listen(lambda event: collisionWarning(event))
    # sensor.listen(lambda image : image.save_to_disk('out7%06d.png'% image.frame))
    print('loop')
    time.sleep(120)

finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')