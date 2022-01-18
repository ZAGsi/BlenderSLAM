import bpy
import addon_utils

import os
import requests

import cv2
import numpy as np
from ruamel import yaml
from tqdm import tqdm

from . import image_to_plane

class calibrate_OT_operator(bpy.types.Operator):
    bl_idname = "slam.calibrate"
    bl_label = "Calibrate stereo cameras"
    bl_context = "scene"

    def execute(self, context):
        bpy.ops.slam.start_calibration('INVOKE_DEFAULT')

        scene = context.scene
        if scene.CalibrationProperties.path == "":
            print("no (output) path given")
            bpy.ops.slam.end_calibration('INVOKE_DEFAULT')
            return {"FINISHED"}
        if scene.CalibrationProperties.calibration_method == "LIVE":
            return {"FINISHED"}
        lp = os.path.join(scene.CalibrationProperties.path, 'left')
        rp = os.path.join(scene.CalibrationProperties.path, 'right')
        print(lp, rp)

        if not os.path.exists(rp):
            os.makedirs(rp)
        if not os.path.exists(lp):
            os.makedirs(lp)

        bpy.ops.slam.compute_calibration('INVOKE_DEFAULT')
        return {"FINISHED"}

class fetch_image_OT_operator(bpy.types.Operator):
    bl_idname = "slam.fetch_image"
    bl_label = "Capture image"
    bl_context = "scene"

    def execute(self, context):
        scene = context.scene
        cal_props = scene.CalibrationProperties
        slam_config_props = scene.SLAMAcquisitionProperties
        left_img_path = get_img_path(cal_props.path, 'left', cal_props.current_img_id)
        right_img_path = get_img_path(cal_props.path, 'right', cal_props.current_img_id)

        # CAPTURE IMG
        img_l, img_r = None, None

        if slam_config_props.acquisition_method == "IP":
            # img_l, img_r = self.capture_image_IP(slam_config_props.IP_address, slam_config_props.IP_address)
            bpy.ops.slam.still('INVOKE_DEFAULT')
            img_l, img_r = cv2.imread(left_img_path), cv2.imread(right_img_path)  # this is maybe not the best way, Image00 is not always Image00 etc.
        elif slam_config_props.acquisition_method == "USB":
            # TODO: CAPTURE WITH USB
            pass
            # img_l, img_r = self.capture_image_USB(slam_config_props.USB_port, slam_config_props.USB_port)
        elif slam_config_props.acquisition_method == "TEST":
            img_l, img_r = cv2.imread(left_img_path), cv2.imread(right_img_path)

        if img_l is None or img_r is None:
            print(f'Capturing images failed... Image {left_img_path} or Image {right_img_path} was not present')
            return {"FINISHED"}

        # SAVE IMG
        if not slam_config_props.acquisition_method == "TEST":
            cv2.imwrite(left_img_path, img_l)
            cv2.imwrite(right_img_path, img_r)

        image_importer = image_to_plane.ImageImport(cal_props.res_x, cal_props.res_y)
        image_importer.change_image(img_l, 'left')
        image_importer.change_image(img_r, 'right')

        context.view_layer.update()
        # # IMPORT IMG IN BLENDER
        # bpy.ops.import_image.to_plane(shader='SHADELESS', height=10,
        #                               files=[{'name': left_img_path}, {'name': right_img_path}])
        cal_props.is_image_captured = True
        return {"FINISHED"}

    # DEPRICATED #
    def capture_image_IP(self, ip_left, ip_right, auth=('admin', '')):
        bpy.ops.slam.still('INVOKE_DEFAULT')
        r_left = requests.get(ip_left, auth=auth)
        r_right = requests.get(ip_right, auth=auth)

        img_l = cv2.imdecode(np.frombuffer(r_left.content, np.uint8), flags=1)
        img_r = cv2.imdecode(np.frombuffer(r_right.content, np.uint8), flags=1)
        return img_l, img_r

    def capture_imgae_USB(self, usb_port_left, usb_port_right):
        pass

class CancelCalibrationOperator(bpy.types.Operator):
    bl_idname = "slam.cancel_calibration"
    bl_label = "Cancel calibration"
    bl_context = "scene"

    def execute(self, context):
        context.scene.CalibrationProperties.current_img_id = 1
        context.scene.CalibrationProperties.is_calibrating = False
        bpy.ops.slam.stop_still('INVOKE_DEFAULT')
        return {'FINISHED'}


class capture_image_OK_OT_operator(bpy.types.Operator):
    bl_idname = "slam.capture_image_ok"
    bl_label = "OK"
    bl_context = "scene"

    def execute(self, context):
        scene = context.scene
        cal_props = scene.CalibrationProperties
        cal_props.current_img_id += 1
        if cal_props.current_img_id > cal_props.n_imgs:
            cal_props.all_images_captured = True
        cal_props.is_image_captured = False
        return {"FINISHED"}


class capture_image_recapture_OT_operator(bpy.types.Operator):
    bl_idname = "slam.capture_image_recapture"
    bl_label = "Recapture image"
    bl_context = "scene"


    def execute(self, context):
        scene = context.scene
        cal_props = scene.CalibrationProperties
        print(f'recapturing img {cal_props.current_img_id}...')
        bpy.ops.slam.fetch_image('INVOKE_DEFAULT')
        return {"FINISHED"}

class compute_calibration_OT_operator(bpy.types.Operator):
    bl_idname = "slam.compute_calibration"
    bl_label = "Compute calibration"
    bl_context = "scene"

    def execute(self, context):
        scene = context.scene
        cal_props = scene.CalibrationProperties
        calibration = Calibrate(context)
        if cal_props.is_calibrated:
            bpy.ops.slam.import_yaml('INVOKE_DEFAULT')
        else:
            rectify_config = calibration.calibrate()
            print(rectify_config)

        self.end_calibration(context)
        return {"FINISHED"}

    def end_calibration(self, context):
        bpy.ops.slam.end_calibration('INVOKE_DEFAULT')
        context.scene.CalibrationProperties.is_calibrated = True


class EndCalibration(bpy.types.Operator):
    bl_idname = "slam.end_calibration"
    bl_label = "End calibration"
    bl_context = "scene"

    def execute(self, context):
        scene = context.scene
        if scene.CalibrationProperties.calibration_method == "LIVE":
            bpy.ops.slam.stop_still('INVOKE_DEFAULT')
        context.scene.CalibrationProperties.is_calibrating = False
        # context.scene.CalibrationProperties.is_calibrated = True
        return {"FINISHED"}


class StartCalibration(bpy.types.Operator):
    bl_idname = "slam.start_calibration"
    bl_label = "Start calibration"
    bl_context = "scene"

    def execute(self, context):
        scene = context.scene

        cal_props = context.scene.CalibrationProperties
        cal_props.is_calibrating = True
        if scene.CalibrationProperties.calibration_method == "LIVE":
            bpy.ops.slam.start_still('INVOKE_DEFAULT')
        if scene.CalibrationProperties.calibration_method != "YAML":
            image_importer = image_to_plane.ImageImport(res_x=1024, res_y=720)
            image_importer.initialize_planes()
        return {"FINISHED"}

class ImportYAMLFiles(bpy.types.Operator):
    bl_idname = "slam.import_yaml"
    bl_label = "Import YAML file from folder"
    bl_context = "scene"

    def execute(self, context):
        scene = context.scene
        cal_props = scene.CalibrationProperties
        if scene.CalibrationProperties.path == "":
            print("no (output) path given")
            return {"FINISHED"}
        calibration = Calibrate(context)
        rectify_config = calibration.import_calibration_values()
        print(rectify_config)
        cal_props.is_calibrated = True
        return {"FINISHED"}


class RemoveCalibration(bpy.types.Operator):
    bl_idname = "slam.remove_calibration"
    bl_label = "Remove calibration"
    bl_context = "scene"

    def execute(self, context):
        scene = context.scene
        cal_props = scene.CalibrationProperties
        cal_props.is_calibrated = False
        cal_props.is_calibrating = False
        cal_props.is_image_captured = False
        cal_props.all_images_captured = False
        cal_props.current_img_id = 1
        return {"FINISHED"}

class Calibrate:
    def __init__(self, context):
        self.config = context.scene.CalibrationProperties
        self.CHESSBOARD_DIM = self.config.chess_dim_w, self.config.chess_dim_h

        # Set parameters for calibration
        self.objp = np.zeros((self.config.chess_dim_w * self.config.chess_dim_h, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.config.chess_dim_w, 0:self.config.chess_dim_h].T.reshape(-1, 2)
        # Termination criteria for refining the detected corners
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        if not os.path.exists(self.config.path):
            os.makedirs(self.config.path)
        if not os.path.exists(left_path(self.config.path)):
            os.makedirs(left_path(self.config.path))
        if not os.path.exists(right_path(self.config.path)):
            os.makedirs(right_path(self.config.path))

    def calibrate(self):
        img_ptsL, img_ptsR, obj_pts, img_res = self.get_images()

        # Calibrating left camera
        retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(obj_pts, img_ptsL, img_res, None, None)
        wL, hL = img_res
        new_mtxL, roiL = cv2.getOptimalNewCameraMatrix(mtxL, distL, (wL, hL), 1, (wL, hL))

        # Calibrating right camera
        retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(obj_pts, img_ptsR, img_res, None, None)
        wR, hR = img_res
        new_mtxR, roiR = cv2.getOptimalNewCameraMatrix(mtxR, distR, (wR, hR), 1, (wR, hR))

        flags = 0
        flags |= cv2.CALIB_FIX_INTRINSIC
        # Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat are calculated.
        # Hen   ce intrinsic parameters are the same
        criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # This step is performed to transformation between the two cameras and calculate Essential and Fundamental matrix
        retS, new_mtxL, distL, new_mtxR, distR, Rot, Trns, Emat, Fmat = cv2.stereoCalibrate(obj_pts, img_ptsL, img_ptsR,
                                                                                            new_mtxL, distL, new_mtxR,
                                                                                            distR,
                                                                                            img_res,
                                                                                            criteria_stereo, flags)
        print(f"Saving parameters to {yaml_file(self.config.path)}......")
        calibration = {"cam0": {"intrinsics": new_mtxL.tolist(),
                                "distortion_coeffs": distL.tolist(),
                                "resolution": list(img_res)
                                },
                       "cam1": {"intrinsics": new_mtxR.tolist(),
                                "distortion_coeffs": distR.tolist(),
                                "rotation": Rot.tolist(),
                                "translation": Trns.tolist(),
                                "resolution": list(img_res)
                                }
                       }
        # print(calibration)
        with open(yaml_file(self.config.path), 'w') as outfile:
            yaml.dump(calibration, outfile, block_seq_indent=2)

        rectify_scale = 1
        rectify_config = {"cameraMatrix1": new_mtxL,
                          "distCoeffs1": distL,
                          "cameraMatrix2": new_mtxR,
                          "distCoeffs2": distR,
                          "imageSize": img_res,
                          "R": Rot,
                          "T": Trns,
                          "newImageSize": (0, 0)}
        return rectify_config

    def get_images(self):
        img_ptsL = []
        img_ptsR = []
        obj_pts = []
        imgL_gray = None

        for i in tqdm(range(1, 12)):
            left_img_path = get_img_path(self.config.path, 'left', i)
            right_img_path = get_img_path(self.config.path, 'right', i)
            imgL = cv2.imread(left_img_path)
            imgR = cv2.imread(right_img_path)
            imgL_gray = cv2.imread(left_img_path, 0)
            imgR_gray = cv2.imread(right_img_path, 0)

            ret, cornersL, cornersR = self.find_chessboard_corners(imgL, imgR, imgL_gray, imgR_gray)
            if ret:
                obj_pts.append(self.objp)
                img_ptsL.append(cornersL)
                img_ptsR.append(cornersR)

        img_res = (0, 0)
        if imgL_gray is not None:
            img_res = imgL_gray.shape[::-1]
        return img_ptsL, img_ptsR, obj_pts, img_res

    def find_chessboard_corners(self, imgL, imgR, imgL_gray, imgR_gray):
        outputL = imgL.copy()
        outputR = imgR.copy()

        retR, cornersR = cv2.findChessboardCorners(outputR, self.CHESSBOARD_DIM, None)
        retL, cornersL = cv2.findChessboardCorners(outputL, self.CHESSBOARD_DIM, None)

        # If not found return none
        if not retR and not retL:
            print('Corners not found')
            # if self.live:
            return False, None, None

        cv2.cornerSubPix(imgR_gray, cornersR, (11, 11), (-1, -1), self.criteria)
        cv2.cornerSubPix(imgL_gray, cornersL, (11, 11), (-1, -1), self.criteria)

        return True, cornersL, cornersR

    def import_calibration_values(self):
        with open(yaml_file(self.config.path), 'r') as stream:
            calibration = yaml.safe_load(stream)

        rectify_scale = 1
        rectify_config = {"cameraMatrix1": np.matrix(calibration["cam0"]["intrinsics"]),
                  "distCoeffs1": np.matrix(calibration["cam0"]["distortion_coeffs"]),
                  "cameraMatrix2": np.matrix(calibration["cam1"]["intrinsics"]),
                  "distCoeffs2": np.matrix(calibration["cam1"]["distortion_coeffs"]),
                  "imageSize": np.array(calibration['cam0']['resolution']),
                  "R": np.matrix(calibration["cam1"]["rotation"]),
                  "T": np.matrix(calibration["cam1"]["translation"]),
                  "newImageSize": (0, 0)}

        return rectify_config

def left_path(path):
    return os.path.join(path, "left")

def right_path(path):
    return os.path.join(path, "right")

def yaml_file(path):
    return os.path.join(path, "stereo_parameters.yaml")

def get_img_path(path, side, id):
    side_path = left_path(path) if side == 'left' else right_path(path)
    return os.path.join(side_path, f'Image{(id - 1):02d}.jpg')

# DEPRICATED
# def get_img_path(path, side, id):
#     side_path = left_path(path) if side == 'left' else right_path(path)
#     return os.path.join(side_path, f'img{id}.png')