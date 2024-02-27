#!/usr/bin/env python3
"""
Copyright Aeva 2024

Open3D FMCW LiDAR visualization example for CARLA

Example usage:
python open3d_fmcw_lidar.py --motion-compensate --channel-color velocity
"""

import argparse
import glob
import os
import random
import sys
import time
import yaml
from queue import Queue
from queue import Empty

import cv2
import numpy as np
import open3d as o3d
from matplotlib import cm

try:
    sys.path.append(
        glob.glob('../carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
LABEL_COLORS = np.array([
    # CARLA standard tags. See LibCarla/source/carla/image/CityScapesPalette.h
    (255, 255, 255),  #  0 - None
    (128, 64, 128),  #  1 - Road
    (244, 35, 232),  #  2 - Sidewalk
    (70, 70, 70),  #  3 - Building
    (102, 102, 156),  #  4 - Wall
    (100, 40, 40),  #  5 - Fences
    (153, 153, 153),  #  6 - Pole
    (250, 170, 30),  #  7 - TrafficLight
    (220, 220, 0),  #  8 - TrafficSign
    (107, 142, 35),  #  9 - Vegetation
    (145, 170, 100),  # 10 - Terrain
    (70, 130, 180),  # 11 - Sky
    (220, 20, 60),  # 12 - Pedestrian
    (220, 20, 10),  # 13 - Rider
    (245, 150, 100),  # 14 - Cars
    (70, 130, 180),  # 15 - Trucks
    (250, 80, 100),  # 16 - Buses
    (150, 60, 30),  # 17 - Trains
    (90, 30, 150),  # 18 - Motorbikes
    (245, 230, 100),  # 19 - Bicycles
    (110, 190, 160),  # 20 - Static
    (170, 120, 50),  # 21 - Dynamic
    (55, 90, 80),  # 22 - Other
    (45, 60, 150),  # 23 - Water
    (157, 234, 50),  # 24 - RoadLines
    (81, 0, 81),  # 25 - Ground
    (150, 100, 100),  # 26 - Bridge
    (230, 150, 140),  # 27 - RailTrack
    (180, 165, 180),  # 28 - GuardRail
]) / 255.0  # Normalize each channel [0-1] since is what Open3D uses


def generate_lidar_blueprint(args, world, blueprint_library, sim_delta):
    """Generates a CARLA blueprint based on the script parameters."""
    lidar_bp = blueprint_library.find('sensor.lidar.fmcw')
    lidar_bp.set_attribute('range', str(args.range))
    lidar_bp.set_attribute('channels', str(args.threads))

    lidar_bp.set_attribute('pattern_file', args.pattern_file)
    lidar_bp.set_attribute('pattern_name', args.pattern_name)
    lidar_bp.set_attribute('motion_compensate',
                           str(args.motion_compensate).lower())

    raycast_modes = {"frame": "0", "line": "1", "point": "2"}
    lidar_bp.set_attribute('raycast_mode', raycast_modes[args.raycast_mode])

    if args.no_noise:
        lidar_bp.set_attribute('dropoff_general_rate', '0.0')
        lidar_bp.set_attribute('dropoff_intensity_limit', '1.0')
        lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
    else:
        lidar_bp.set_attribute('noise_stddev', '0.05')
        lidar_bp.set_attribute('dropoff_general_rate', '0.3')

    for attr in sorted(lidar_bp, key=lambda a: a.id):
        print('  - {}'.format(attr))

    return lidar_bp


def linear_interpolate(point1, point2, alphas):
    """Linearly interpolates point1 --> point2, given a vector of alphas."""
    points1 = np.repeat(np.expand_dims(point1, 0), len(alphas), axis=0)
    points2 = np.repeat(np.expand_dims(point2, 0), len(alphas), axis=0)
    return (1 - alphas[:, None]) * points1 + alphas[:, None] * points2


def generate_intensity_colors(intensity_channel):
    """Generates point cloud colors based on the intensity channel."""
    intensity_col = 1.0 - np.log(intensity_channel) / np.log(
        np.exp(-0.004 * 100))
    return np.c_[np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
                 np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
                 np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]


def generate_semantic_colors(class_channel):
    """Generates point cloud colors based on the CityScapes color palette."""
    labels = np.array(class_channel)

    return LABEL_COLORS[labels]


def generate_velocity_colors(velocity_channel,
                             min_velocity=-15,
                             max_velocity=15,
                             static_color=[1.0, 1.0, 1.0]):
    """Generates point cloud colors based on the velocity channel [N x 1] array."""
    # Color all points with static_color.
    colors = np.tile(static_color, (len(velocity_channel), 1))

    # Color all positive velocity points static_color --> red till max velocity.
    pos_idx = velocity_channel > 0
    pos_mag = np.clip(velocity_channel[pos_idx] / max_velocity, 0, 1)
    colors[pos_idx] = linear_interpolate(static_color, [1.0, 0.0, 0.0],
                                         pos_mag)

    # Color all negative velocity points static_color --> blue till min velocity.
    neg_idx = velocity_channel < 0
    neg_mag = np.clip(velocity_channel[neg_idx] / min_velocity, 0, 1)
    colors[neg_idx] = linear_interpolate(static_color, [0.0, 0.0, 1.0],
                                         neg_mag)

    return colors


def generate_dynamic_colors(dynamic_channel,
                            static_color=[1.0, 1.0, 1.0],
                            dynamic_color=[1.0, 0.0, 0.0]):
    """Generates point cloud colors based on the dynamic flag."""
    # Color all points with static_color.
    colors = np.tile(static_color, (len(dynamic_channel), 1))

    # Color all dynamic points red.
    dyn_idx = (dynamic_channel == 1)
    colors[dyn_idx] = dynamic_color

    return colors


def generate_valid_colors(valid_channel,
                          valid_color=[0.5, 1.0, 0.5],
                          invalid_color=[1.0, 0.5, 0.5]):
    """Generates point cloud colors based on the valid flag."""
    # Color all points with valid_color.
    colors = np.tile(valid_color, (len(valid_channel), 1))

    # Color all invalid points red.
    dyn_idx = (valid_channel == 0)
    colors[dyn_idx] = invalid_color

    return colors


def generate_beam_colors(beam_channel, colormap='tab10'):
    """Generates point cloud colors based on which beam the point belongs to."""
    assert len(beam_channel.shape) == 1
    cmap = cm.get_cmap(colormap)

    return cmap(beam_channel)[:, :3]


def spherical_to_cartesian(azimuth_deg, elevation_deg, range):
    azimuth = np.deg2rad(azimuth_deg)
    elevation = np.deg2rad(90 + elevation_deg)
    x = range * np.sin(elevation) * np.cos(azimuth)
    y = range * np.sin(elevation) * np.sin(azimuth)
    z = range * np.cos(elevation)

    return [x, y, -z]


def update_point_cloud(pcd, data, color_channel):
    if color_channel != 'valid':
        data = data[data['valid'] == 1]

    if color_channel == 'intensity':
        colors = generate_intensity_colors(data['intensity'])

    if color_channel == 'semantic':
        colors = generate_semantic_colors(data['class'])

    if color_channel == 'velocity':
        colors = generate_velocity_colors(data['velocity'])

    if color_channel == 'beam':
        colors = generate_beam_colors(data['beam_idx'])

    if color_channel == 'dynamic':
        colors = generate_dynamic_colors(data['dynamic'])

    if color_channel == 'valid':
        colors = generate_valid_colors(data['valid'])

    points = np.array(
        spherical_to_cartesian(data['azimuth'], data['elevation'],
                               data['range'])).T
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)


def lidar_callback(data, lidar_queue):
    """
    Prepares a point cloud with intensity and velocity
    colors ready to be consumed by Open3D.
    """
    point_cloud = np.frombuffer(data.raw_data,
                                dtype=np.dtype([('azimuth', np.float32),
                                                ('elevation', np.float32),
                                                ('range', np.float32),
                                                ('intensity', np.float32),
                                                ('velocity', np.float32),
                                                ('cos_angle', np.float32),
                                                ('instance', np.uint32),
                                                ('class', np.uint32),
                                                ('point_idx', np.uint32),
                                                ('beam_idx', np.ubyte),
                                                ('valid', bool),
                                                ('dynamic', bool)])).copy()

    # Negate the azimuth to account for scanning direction
    point_cloud['azimuth'] = -point_cloud['azimuth']

    point_count = len(point_cloud)
    if len(point_cloud) != data.get_point_count_channel() * data.channels != data.get_point_count_beam() * data.beams:
        print(
            '\nWARNING: Number of points received (%d) do not match expected count (%d)(%d)'
            % (len(point_cloud), data.get_point_count_channel() * data.channels, data.get_point_count_beam() * data.beams))

    lidar_queue.put((point_count, point_cloud))


def camera_callback(img, data, window_name):
    img = np.frombuffer(data.raw_data, dtype=np.dtype('uint8')).reshape(
        (data.height, data.width, 4))[:, :, :3][:, :, ::-1]


def main(args):
    """Main function of the script"""
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)
    world = client.get_world()

    # This sensor must have control of the simulator tick.
    settings = world.get_settings()
    if settings.synchronous_mode:
        print(
            '''ERROR: Another client has already taken control of the synchronous tick. Please
                 close any other synchronous clients before running the fmcw lidar.'''
        )
        exit(1)

    try:
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(0)

        original_settings = world.get_settings()
        settings = world.get_settings()

        # We create the sensor queue in which we keep track of the information
        # already received. This structure is thread safe and can be
        # accessed by all the sensors callback concurrently without problem.
        lidar_queue = Queue()

        with open(args.pattern_file) as file:
            node = yaml.safe_load(file)
            pattern = node['patterns'][args.pattern_name]
            # Adjust simulation timestep based on raycast mode.
            # See RaycastMode in FMCWLidarPattern.h.
            if args.raycast_mode == 'frame':
                sim_delta = args.frame_period
            elif args.raycast_mode == 'line':
                sim_delta = args.frame_period / len(
                    pattern['elevation_steps_deg'])
            elif args.raycast_mode == 'point':
                sim_delta = args.frame_period / (
                    len(pattern['elevation_steps_deg']) *
                    (pattern['points_per_line'] / args.threads))
            else:
                print('ERROR: Unsupported raycast mode.')
                exit(1)

        settings.fixed_delta_seconds = sim_delta
        settings.synchronous_mode = True
        settings.no_rendering_mode = args.no_rendering
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter(args.filter)[0]
        vehicle_transform = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)
        print('Vehicle Actor ID ', vehicle.id)
        vehicle.set_autopilot(args.no_autopilot)

        user_offset = carla.Location(args.x, args.y, args.z)
        lidar_transform = carla.Transform(
            carla.Location(x=1.04, z=1.69) + user_offset)
        lidar_bp = generate_lidar_blueprint(args, world, blueprint_library,
                                            sim_delta)
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
        lidar.listen(lambda data: lidar_callback(data, lidar_queue))

        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name='CARLA FMCW LiDAR',
                          width=960,
                          height=540,
                          left=480,
                          top=270)
        vis.get_render_option().background_color = [0.05, 0.05, 0.05]
        vis.get_render_option().point_size = 2
        vis.get_render_option().show_coordinate_frame = True

        # Add camera.
        if args.display_camera:
            image_size = (800, 600)
            window_name = 'Camera'
            img = np.zeros((image_size[1], image_size[0], 3))
            camera_bp = blueprint_library.find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', str(image_size[0]))
            camera_bp.set_attribute('image_size_y', str(image_size[1]))
            camera = world.spawn_actor(camera_bp,
                                       lidar_transform,
                                       attach_to=vehicle)
            camera.listen(lambda data: camera_callback(img, data, window_name))
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, *image_size)

        pcd = o3d.geometry.PointCloud()

        # Set up the starting location of the visualization camera based on a rough approximation
        # of the y (horizontal) extent.
        # Create 2 dummy points with the given y location, then set the view parameters.
        points = np.array([[0, -1.0 * args.view_distance, 0],
                           [0, args.view_distance, 0]])
        pcd.points = o3d.utility.Vector3dVector(points)
        vis.add_geometry(pcd)
        vis.get_view_control().set_front([-0.3, 0, 0.2])
        vis.get_view_control().set_up([0.2, 0, 0.3])
        vis.get_view_control().set_lookat([23, 0, 0])
        vis.get_view_control().set_zoom(0.25)

        frame = 0
        start_time = time.time()
        while True:
            world.tick()
            timestamp = world.get_snapshot().timestamp.elapsed_seconds

            # Now, we wait to the sensors data to be received.
            # As the queue is blocking, we will wait in the queue.get() methods
            # until all the information is processed and we continue with the next frame.
            # We include a timeout of 1.0 s (in the get method) and if some information is
            # not received in this time we continue.
            try:
                point_count, point_cloud = lidar_queue.get(True, 1.0)
                update_point_cloud(pcd, point_cloud, args.channel_color)
                vis.update_geometry(pcd)

                vis.poll_events()
                vis.update_renderer()
                time.sleep(0.005)  # This can fix Open3D jittering issues.

            except Empty:
                print('Some of the sensor information is missed')

            process_time = time.time() - start_time
            sys.stdout.write(
                '\rWorld Time: %.2fs Frame: %05d Points: (%06d/%06d) FPS: %.2f'
                % (timestamp, frame, len(
                    pcd.points), point_count, 1.0 / process_time))
            sys.stdout.flush()
            start_time = time.time()
            frame += 1

    finally:
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)

        vehicle.destroy()
        lidar.destroy()
        vis.destroy_window()

        if args.display_camera:
            camera.destroy()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    executable_dir = os.path.dirname(os.path.abspath(__file__))
    pattern_yaml_path = os.path.abspath(
        os.path.join(executable_dir, "../../ScanPatterns.yaml"))

    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument('-p',
                           '--port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        help='Use the no-redering mode which will provide some extra'
        ' performance but you will lose the articulated objects in the'
        ' LiDAR, such as pedestrians')
    argparser.add_argument('--pattern-file',
                           default=str(pattern_yaml_path),
                           type=str,
                           help='Absolute path to scan pattern yaml file')
    argparser.add_argument('--pattern-name',
                           default='64-19.2-Uniform',
                           type=str,
                           help='Name of scan pattern')
    argparser.add_argument(
        '--motion-compensate',
        action='store_true',
        help='Motion compensate the doppler velocity with ego-vehicle velocity'
    )
    argparser.add_argument(
        '--raycast-mode',
        default='frame',
        type=str,
        choices=['frame', 'line', 'point'],
        help='Sets what should be raycast on every simulation delta step.'
        'Enable modelling of rolling shutter effect when raycast by line or point.'
        'Raycast modes: 0 (by frame), 1 (by line), 2 (by point)')
    argparser.add_argument(
        '--no-noise',
        action='store_true',
        help='Remove the drop off and noise from the normal LiDAR')
    argparser.add_argument(
        '--no-autopilot',
        action='store_false',
        help='Disables the autopilot so the vehicle will remain stopped')
    argparser.add_argument('--filter',
                           metavar='PATTERN',
                           default='model3',
                           help='CARLA actor filter (default: "vehicle.*")')
    argparser.add_argument('--threads',
                           default=64,
                           type=int,
                           help='Ray-casting thread count (default: 64)')
    argparser.add_argument(
        '--range',
        default=200.0,
        type=float,
        help='LiDAR\'s maximum range in meters (default: 200.0)')
    argparser.add_argument(
        '--frame-period',
        default=0.1,
        type=float,
        help='Number of seconds between each complete frame')
    argparser.add_argument(
        '--channel-color',
        default='velocity',
        type=str,
        choices=[
            'velocity', 'intensity', 'semantic', 'beam', 'dynamic', 'valid'
        ],
        help='Channel to colorize point cloud (default: velocity)')
    argparser.add_argument('--display-camera',
                           action='store_true',
                           help='Display images from camera stream')
    argparser.add_argument(
        '-x',
        default=0.0,
        type=float,
        help='X-offset in the sensor position in meters (default: 0.0)')
    argparser.add_argument(
        '-y',
        default=0.0,
        type=float,
        help='Y-offset in the sensor position in meters (default: 0.0)')
    argparser.add_argument(
        '-z',
        default=0.0,
        type=float,
        help='Z-offset in the sensor position in meters (default: 0.0)')
    argparser.add_argument(
        '--view-distance',
        default=100.0,
        type=float,
        help=
        'Adjusts the initial closeness/view extent of the point cloud visualizer.'
    )
    args = argparser.parse_args()

    try:
        main(args)
    except KeyboardInterrupt:
        print('\nExited by user')
