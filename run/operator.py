import numpy as np
import time
import g2o
import bpy
from threading import Thread,  Lock
from space_view3d_point_cloud_visualizer import PCVControl

from ..SLAM.stereo_ptam.components import Camera
from ..SLAM.stereo_ptam.components import StereoFrame
from ..SLAM.stereo_ptam.feature import ImageFeature
from ..SLAM.stereo_ptam.params import ParamsKITTI, ParamsEuroc
from ..SLAM.stereo_ptam.dataset import KITTIOdometry, EuRoCDataset
from ..SLAM.stereo_ptam.sptam import SPTAM

class PTAM_worker(Thread):

    def __init__(self, l):
        Thread.__init__(self)
        self.lock = l

        self.scene = bpy.context.scene
        self.props = self.scene.PTAM_properties

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
        dataset = None
        params = None
        if self.props.dataset.lower() == 'kitti':
            params = ParamsKITTI()
            dataset = KITTIOdometry(self.props.path)
        elif self.props.dataset.lower() == 'euroc':
            params = ParamsEuroc()
            dataset = EuRoCDataset(self.props.path)

        self.sptam = SPTAM(params)

        self.cam = Camera(
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
                self.sptam.initialize(frame)
            else:
                self.sptam.track(frame)

            duration = time.time() - time_start
            self.durations.append(duration)
            print('duration', duration)
            print()
            print()

            pts = []
            for m in self.sptam.reference.measurements():
                if m.from_triangulation():
                    pts.append([m.mappoint.position, m.mappoint.normal, m.mappoint.color])

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

class PTAM_OT_operator(bpy.types.Operator):
    bl_idname = "ptam.startalgorithm"
    bl_label = "Start PTAM algorithm"
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
                    position, normal, colour = pt

                    vertices.append([position[0], position[2], position[1]])
                    normals.append([normal[0], normal[2], normal[1]])
                    colours.append([colour[0], colour[1], colour[2]])
                self.last_idx += 1
            obj = self.create_pointcloud_object(f"pointcloud_{self.pc_idx}")
            self.pc_idx += 1
            c = PCVControl(obj)
            c.draw(vertices, normals, colours)

        if process_finished:
            print("PTAM finished")
            context.window_manager.event_timer_remove(self.timer)
            self.t.stop()
            return {'FINISHED'}

        return {'PASS_THROUGH'}

    def execute(self, context):
        self.scene = context.scene
        self.props = self.scene.PTAM_properties
        self.lock = Lock()
        self.t = PTAM_worker(self.lock)
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
