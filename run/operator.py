import bpy
import os
import time

import numpy as np
import g2o
import cv2

from threading import Thread,  Lock
from space_view3d_point_cloud_visualizer import PCVControl
from ruamel import yaml

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
        self.config = self.scene.SLAM_config_properties
        self.calibration = self.scene.calibration_properties
        self.props = self.scene.SLAM_properties

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


    def start(self):
        if self.config.standard_set.lower() == 'kitti':
            params = ParamsKITTI(self.props.feature_descriptor)
            dataset = KITTIOdometry(self.config.path)
        elif self.config.standard_set.lower() == 'euroc':
            params = ParamsEuroc()
            dataset = EuRoCDataset(self.config.path)
        else:
            params = ParamsKITTI(self.props.feature_descriptor)  # TODO: maybe have a custom Params creator?
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
        self.running = False

    def run(self):
        for i in range(len(self.dataset))[:self.max_images]:
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
                try:
                    self.sptam.initialize(frame)
                except AssertionError:
                    print(f'unable to match points in image {i:06d}')
                    continue
            else:
                self.sptam.track(frame)

            duration = time.time() - time_start
            self.durations.append(duration)
            print('duration', duration)
            print()
            print()

            pts = []
            # for kf in self.sptam.graph.keyframes()[-20:]:
            #     if kf.id not in self.saved_keyframes:
            #         self.saved_keyframes.add(kf.id)
            #         for m in kf.measurements():
            #             if m.from_triangulation():
            #                 pts.append([m.mappoint.position, m.mappoint.normal, m.mappoint.color])

            with self.lock:
                self.pts.append(pts)
                self.current_orientation = frame.orientation
                self.current_position = frame.position

            # break if process is cancelled
            if not self.is_running:
                break

        print('num frames', len(self.durations))
        print('num keyframes', len(self.sptam.graph.keyframes()))
        print('average time', np.mean(self.durations))
        self.is_running = False
        self.sptam.stop()

class SLAM_OT_operator(bpy.types.Operator):
    bl_idname = "slam.startalgorithm"
    bl_label = "Start SLAM algorithm"
    bl_context = "scene"

    t = None
    timer = None
    pc_idx = 0
    lock = None
    last_idx = 0

    def modal(self, context, event):
        process_cancelled = event.type == 'ESC'
        process_finished = not self.t.is_running

        # Stop the thread if ESCAPE is pressed.
        if process_cancelled:
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
            vertices = []
            normals = []
            colours = []
            # for pts in pts_list[self.last_idx:]:
            for pts in pts_list:
                for pt in pts:
                    p, n, c = pt

                    vertices.append([p[0], p[2], p[1]])
                    normals.append([n[0], n[2], n[1]])
                    colours.append([c[0], c[1], c[2]])
                self.last_idx += 1
            obj = self.create_pointcloud_object(f"pointcloud_{self.pc_idx}")
            self.pc_idx += 1
            c = PCVControl(obj)
            c.draw(vertices, normals, colours)

        if process_finished:
            print("SLAM finished")
            context.window_manager.event_timer_remove(self.timer)
            self.t.stop()
            return {'FINISHED'}

        return {'PASS_THROUGH'}

    def execute(self, context):
        self.scene = context.scene
        self.props = self.scene.SLAM_properties
        self.lock = Lock()
        self.t = SLAM_worker(self.lock)
        self.t.start()

        wm = context.window_manager
        self.timer = wm.event_timer_add(time_step=self.props.update_speed, window=context.window)
        wm.modal_handler_add(self)
        return {"RUNNING_MODAL"}

    @staticmethod
    def create_pointcloud_object(name):
        mesh = bpy.data.meshes.new("geom_" + name)
        obj = bpy.data.objects.new("pointcloud_" + name, mesh)
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
        sequence_path = context.scene.SLAM_config_properties.path
        calibration_path = context.scene.calibration_properties.path
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

    def listdir(self, dir):
        files = [_ for _ in os.listdir(dir) if _.endswith('.png')]
        return [os.path.join(dir, _) for _ in self.sort(files)]

    def sort(self, xs):
        return sorted(xs, key=lambda x:float(x[:-4]))

    def __len__(self):
        return len(self.left)

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