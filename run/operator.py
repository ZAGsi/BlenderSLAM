import os
import time
import bpy

import numpy as np
import g2o
import cv2

from threading import Thread, Lock
from space_view3d_point_cloud_visualizer import PCVControl
from ruamel import yaml
from mathutils import Matrix
from math import radians
from elas import Elas

from ..SLAM.stereo_ptam import components
from ..SLAM.stereo_ptam import dataset
from ..SLAM.stereo_ptam.components import StereoFrame
from ..SLAM.stereo_ptam.feature import ImageFeature
from ..SLAM.stereo_ptam.params import ParamsKITTI, ParamsEuroc
from ..SLAM.stereo_ptam.dataset import KITTIOdometry, EuRoCDataset
from ..SLAM.stereo_ptam.sptam import SPTAM


class SLAM_worker(Thread):

    def __init__(self, l):
        Thread.__init__(self)
        self.lock = l

        self.context = bpy.context
        self.scene = self.context.scene
        self.config = self.scene.SLAMAcquisitionProperties
        self.calibration = self.scene.CalibrationProperties
        self.props = self.scene.SLAMSettings

        self.params = None
        self.dataset = None
        self.cam = None
        self.sptam = None
        self.pts = []
        self.durations = []
        self.max_images = None
        self.is_running = False
        self.current_position = None
        self.current_orientation = None

        # Dense frame properties
        self.dense_frame_frequency = 10
        self.elas = Elas()
        self.dense_pts = []

    def start(self):
        if self.config.standard_set.lower() == 'kitti':
            params = ParamsKITTI(self.props.feature_descriptor)
            dataset = KITTIOdometry(self.config.path)
        elif self.config.standard_set.lower() == 'euroc':
            params = ParamsEuroc()
            dataset = EuRoCDataset(self.config.path)
        else:
            params = ParamsKITTI(self.props.feature_descriptor)  # TODO: maybe have a custom Params creator?

            # change param variables from m to mm
            params.frustum_near *= 1000
            params.frustum_far *= 1000
            params.lc_max_inbetween_distance *= 1000
            params.lc_distance_threshold *= 1000

            dataset = CustomDataset(self.context)
        self.sptam = SPTAM(params)

        self.cam = components.Camera(
            dataset.cam.fx, dataset.cam.fy, dataset.cam.cx, dataset.cam.cy,
            dataset.cam.width, dataset.cam.height,
            params.frustum_near, params.frustum_far,
            dataset.cam.baseline)

        self.max_images = len(dataset)
        self.dataset = dataset
        self.params = params
        if self.props.max_images != 0 and self.props.max_images < len(dataset):
            self.max_images = self.props.max_images

        self.is_running = True
        self.saved_keyframes = set()

        Thread.start(self)

    def stop(self):
        self.is_running = False

    def run(self):
        def add_dense_pointcloud(keyframe):
            print("compute dense frame", keyframe.id)
            if len(self.dataset.left[i].shape) == 3:
                left = self.dataset.left[i] / 255
                # turn to B&W
                left_bw = cv2.cvtColor(self.dataset.left[i], cv2.COLOR_BGR2GRAY)
                right_bw = cv2.cvtColor(self.dataset.right[i], cv2.COLOR_BGR2GRAY)
            else:
                left = cv2.cvtColor(self.dataset.left[i], cv2.COLOR_GRAY2BGR) / 255
                left_bw = self.dataset.left[i]
                right_bw = self.dataset.right[i]

            # create disparity images
            disp_left, disp_right = self.elas.process(left_bw, right_bw)

            # reproject to 3D and reshape array to list of 3D vectors
            point_cloud = cv2.reprojectImageTo3D(disparity=disp_left, Q=Q).reshape(-1, 3)
            colour = left.reshape(-1, 3)

            # remove all points that are in the back of the frame (Z-value)
            point_cloud[point_cloud[:, 2] > 0] = None
            # remove all points outside view frustum
            depth = np.linalg.norm(point_cloud, axis=1)
            point_cloud[depth < self.cam.frustum_near] = None
            point_cloud[depth > self.cam.frustum_far] = None

            # use np.isnan as a filter to remove all data values
            pc_filter = ~np.isnan(point_cloud).any(axis=1)
            return zip(point_cloud[pc_filter], colour[pc_filter])

        Q = np.identity(4, dtype='d')
        Q[:, 3] = np.array([-self.cam.cx, -self.cam.cy, self.cam.fx, 0])
        Q[3, 2] = -1 / self.cam.baseline

        for i in range(len(self.dataset))[:self.max_images]:
            if not self.is_running:
                break

            featurel = ImageFeature(self.dataset.left[i], self.params)
            featurer = ImageFeature(self.dataset.right[i], self.params)
            timestamp = self.dataset.timestamps[i]

            time_start = time.time()

            t = Thread(target=featurer.extract)
            t.start()
            featurel.extract()
            t.join()

            frame = StereoFrame(i, g2o.Isometry3d(), featurel, featurer, self.cam, timestamp=timestamp)

            if not self.sptam.is_initialized():
                # self.sptam.initialize(frame)
                try:
                    self.sptam.initialize(frame)
                except AssertionError as e:
                    print(f'unable to match points in image {i:06d}: {e}')
                    continue
            else:
                self.sptam.track(frame)

            duration = time.time() - time_start
            self.durations.append(duration)
            print('duration', duration)
            print()
            print()

            pts = []
            dense_pointclouds = []
            for kf in self.sptam.graph.keyframes()[-20:]:
                if kf.id not in self.saved_keyframes:
                    # add a dense cloud where keyframe is
                    if kf.id % self.dense_frame_frequency == 0:
                        dense_pointclouds.append([kf.id, add_dense_pointcloud(kf)])
                    self.saved_keyframes.add(kf.id)
                    for m in kf.measurements():
                        if m.from_triangulation():
                            pts.append([m.mappoint.position, m.mappoint.normal, m.mappoint.color])

            with self.lock:
                self.pts.extend(pts)
                self.dense_pts.extend(dense_pointclouds)

        print('num frames', len(self.durations))
        print('num keyframes', len(self.sptam.graph.keyframes()))
        print('average time', np.mean(self.durations))
        self.is_running = False
        self.sptam.stop()


class RunSLAM(bpy.types.Operator):
    bl_idname = "slam.run_slam"
    bl_label = "Start SLAM algorithm"
    bl_context = "scene"

    t = None
    timer = None
    pc_idx = 0
    lock = None
    last_idx = 0

    def modal(self, context, event):
        def update_pose(obj, position, orientation):
            # change y and z axis
            obj.location = [position[0], position[2], position[1]]
            euler_rotation = Matrix(orientation).to_euler()
            obj.rotation_euler = [euler_rotation.x + radians(90), euler_rotation.z + radians(180), -euler_rotation.y]

        process_cancelled = event.type == 'ESC'
        process_finished = not self.t.is_running

        # Stop the thread if ESCAPE is pressed.
        if process_cancelled:
            with self.lock:
                if self.t is not None:
                    self.t.stop()
            if self.timer is not None:
                context.window_manager.event_timer_remove(self.timer)
            return {'CANCELLED'}

        # Update point cloud object with received data
        if event.type == 'TIMER':
            with self.lock:
                if len(self.t.pts) < 1:
                    return {'PASS_THROUGH'}
                pts_list, self.t.pts = self.t.pts, []
                dense_pts_list, self.t.dense_pts = self.t.dense_pts, []
                kf_data = {kf.id: (kf.position, kf.orientation.matrix()) for kf in self.t.sptam.graph.keyframes()}

            vertices = []
            normals = []
            colours = []
            # for pts in pts_list[self.last_idx:]:
            for p, n, c in pts_list:
                vertices.append([p[0], p[2], p[1]])
                normals.append([n[0], n[2], n[1]])
                colours.append([c[0], c[1], c[2]])
            self.last_idx += 1
            sc_obj = self.create_pointcloud_object(f"sparse_{self.pc_idx}")

            sc = PCVControl(sc_obj)
            sc.draw(vs=vertices, ns=normals, cs=colours)

            # update previous keyframes position
            for kf_id, kf_obj_name in self.dense_pc_objs.items():
                position, orientation = kf_data[kf_id]
                kf_obj = context.scene.objects[kf_obj_name]
                update_pose(kf_obj, position, orientation)

            for kf_id, dense_pts in dense_pts_list:
                kf_obj_name = f"dense_{self.pc_idx}_{kf_id}"
                dc_obj = self.create_pointcloud_object(kf_obj_name)
                vertices, colours = [], []

                for p, c in dense_pts:
                    vertices.append(list(p))
                    colours.append(list(c))

                # update keyframe position
                position, orientation = kf_data[kf_id]
                update_pose(dc_obj, position, orientation)

                # draw dense point cloud using PointCloudViewer
                dc = PCVControl(dc_obj)
                dc.draw(vs=vertices, cs=colours)

                # add dense point cloud to dictionary, to enable updating position and orientation
                self.dense_pc_objs[kf_id] = kf_obj_name

            self.pc_idx += 1

        if process_finished:
            print("SLAM finished")
            context.window_manager.event_timer_remove(self.timer)
            self.t.stop()
            return {'FINISHED'}

        return {'PASS_THROUGH'}

    def execute(self, context):
        self.scene = context.scene
        self.props = self.scene.SLAMSettings
        self.lock = Lock()
        self.t = SLAM_worker(self.lock)
        self.t.start()
        self.dense_pc_objs = {}

        wm = context.window_manager
        self.timer = wm.event_timer_add(time_step=self.props.update_speed, window=context.window)
        wm.modal_handler_add(self)
        return {"RUNNING_MODAL"}

    @staticmethod
    def create_pointcloud_object(name):
        mesh = bpy.data.meshes.new("geom_" + name)
        obj = bpy.data.objects.new(name, mesh)
        col = bpy.data.collections.get("Collection")
        col.objects.link(obj)
        bpy.context.view_layer.objects.active = obj
        return obj


# Similar to EuRoCDataset, but with YAML file from validation
class CustomDataset(object):  # Stereo + IMU
    '''
    path example: 'path/to/your/dataset/'
    '''

    def __init__(self, context, rectify=True):
        sequence_path = context.scene.SLAMAcquisitionProperties.path
        calibration_path = context.scene.CalibrationProperties.path
        config = self.import_calibration_values(self.get_calibration_file(calibration_path))
        rect_l, rect_r, proj_mat_l, proj_mat_r, Q, roiL, roiR = cv2.stereoRectify(**config)
        self.left_cam = dataset.Camera(
            width=config["imageSize"][0], height=config["imageSize"][1],
            intrinsic_matrix=config["cameraMatrix1"],
            undistort_rectify=rectify,
            distortion_coeffs=config["distCoeffs1"],
            rectification_matrix=rect_l,
            projection_matrix=proj_mat_l,
            extrinsic_matrix=np.identity(4)
        )
        self.right_cam = dataset.Camera(
            width=config["imageSize"][0], height=config["imageSize"][1],
            intrinsic_matrix=config["cameraMatrix2"],
            undistort_rectify=rectify,
            distortion_coeffs=config["distCoeffs2"],
            rectification_matrix=rect_r,
            projection_matrix=proj_mat_r,
            extrinsic_matrix=np.vstack((np.hstack((config["R"], config["T"])), [0, 0, 0, 1]))
        )

        timestamps = np.loadtxt(os.path.join(sequence_path, 'times.txt'))
        sequence_path = os.path.expanduser(sequence_path)
        self.left = dataset.ImageReader(
            self.listdir(os.path.join(sequence_path, 'left')),
            timestamps)
        self.right = dataset.ImageReader(
            self.listdir(os.path.join(sequence_path, 'right')),
            timestamps)
        assert len(self.left) == len(self.right)
        self.timestamps = self.left.timestamps

        self.cam = dataset.StereoCamera(self.left_cam, self.right_cam)

    def listdir(self, directory):
        files = [_ for _ in os.listdir(directory) if self.is_image(_)]
        return [os.path.join(directory, _) for _ in sorted(files)]

    def __len__(self):
        return len(self.left)

    @staticmethod
    def is_image(file):
        return file.endswith('.jpg') or file.endswith('.png') or file.endswith('.raw')

    @staticmethod
    def import_calibration_values(YAML):
        with open(YAML, 'r') as stream:
            calibration = yaml.safe_load(stream)

        rectify_scale = 0.3
        config = {"cameraMatrix1": np.matrix(calibration["cam0"]["intrinsics"]),
                  "distCoeffs1": np.matrix(calibration["cam0"]["distortion_coeffs"]),
                  "cameraMatrix2": np.matrix(calibration["cam1"]["intrinsics"]),
                  "distCoeffs2": np.matrix(calibration["cam1"]["distortion_coeffs"]),
                  "imageSize": np.array(calibration['cam0']['resolution']),
                  "R": np.matrix(calibration["cam1"]["rotation"]),
                  "T": np.matrix(calibration["cam1"]["translation"]),
                  "newImageSize": (0, 0),
                  "alpha": rectify_scale}
        return config

    @staticmethod
    def get_calibration_file(path):
        return os.path.join(path, "stereo_parameters.yaml")
