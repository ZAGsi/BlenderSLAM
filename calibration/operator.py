import bpy
import addon_utils

import os
import requests

import cv2
import numpy as np
from ruamel import yaml
from tqdm import tqdm
# from utils import imshow_concat

class calibrate_OT_operator(bpy.types.Operator):
    bl_idname = "slam.calibrate"
    bl_label = "Calibrate stereo cameras"
    bl_context = "scene"

    def execute(self, context):
        self.start_calibration(context)
        addon_utils.enable("io_import_images_as_planes")

        scene = context.scene
        is_live = scene.SLAM_config_properties.is_live
        if is_live:
            return {"FINISHED"}
        if scene.calibration_properties.path == "":
            print("no (output) path given")
            self.end_calibration(context)
            return {"FINISHED"}

        bpy.ops.slam.compute_calibration('INVOKE_DEFAULT')
        return {"FINISHED"}

    def start_calibration(self, context):
        context.scene.calibration_properties.is_calibrating = True

    def end_calibration(self, context):
        context.scene.calibration_properties.is_calibrating = False


class capture_image_OT_operator(bpy.types.Operator):
    bl_idname = "slam.capture_image"
    bl_label = "Capture image"
    bl_context = "scene"

    def execute(self, context):
        scene = context.scene
        cal_props = scene.calibration_properties
        slam_config_props = scene.SLAM_config_properties
        left_img_path = get_img_path(cal_props.path, 'left', cal_props.current_img_id)
        right_img_path = get_img_path(cal_props.path, 'right', cal_props.current_img_id)

        # CAPTURE IMG
        img_l, img_r = None, None

        if slam_config_props.acquisition_method == "IP":
            pass
            # img_l, img_r = self.capture_image_IP(slam_config_props.IP_address, slam_config_props.IP_address)
        elif slam_config_props.acquisition_method == "USB":
            # TODO: CAPTURE WITH USB
            pass
            # img_l, img_r = self.capture_image_USB(slam_config_props.USB_port, slam_config_props.USB_port)
        elif slam_config_props.acquisition_method == "TEST":
            img_l, img_r = cv2.imread(left_img_path), cv2.imread(right_img_path)

        if img_l is None or img_r is None:
            print('Capturing images failed...')
            return {"FINISHED"}

        # SAVE IMG
        if not slam_config_props.acquisition_method == "TEST":
            cv2.imwrite(left_img_path, img_l)
            cv2.imwrite(right_img_path, img_r)

        # IMPORT IMG IN BLENDER
        bpy.ops.import_image.to_plane(shader='SHADELESS', height=10,
                                      files=[{'name': left_img_path}, {'name': right_img_path}])
        cal_props.is_image_captured = True
        return {"FINISHED"}

    def capture_image_IP(self, ip_left, ip_right, auth=('admin', '')):
        r_left = requests.get(ip_left, auth=auth)
        r_right = requests.get(ip_right, auth=auth)

        img_l = cv2.imdecode(np.frombuffer(r_left.content, np.uint8), flags=1)
        img_r = cv2.imdecode(np.frombuffer(r_right.content, np.uint8), flags=1)
        return img_l, img_r

    def capture_imgae_USB(self, usb_port_left, usb_port_right):
        pass


class capture_image_OK_OT_operator(bpy.types.Operator):
    bl_idname = "slam.capture_image_ok"
    bl_label = "OK"
    bl_context = "scene"


    def execute(self, context):
        scene = context.scene
        cal_props = scene.calibration_properties
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
        cal_props = scene.calibration_properties
        print(f'recapturing img {cal_props.current_img_id}...')
        bpy.ops.slam.capture_image('INVOKE_DEFAULT')
        return {"FINISHED"}

class compute_calibration_OT_operator(bpy.types.Operator):
    bl_idname = "slam.compute_calibration"
    bl_label = "Compute calibration"
    bl_context = "scene"

    def execute(self, context):
        scene = context.scene
        cal_props = scene.calibration_properties
        calibration = Calibrate(context)
        if cal_props.is_calibrated:
            rectify_config = calibration.import_calibration_values()
        else:
            rectify_config = calibration.calibrate()
        print(rectify_config)
        self.end_calibration(context)
        return {"FINISHED"}

    def end_calibration(self, context):
        context.scene.calibration_properties.is_calibrating = False
        context.scene.calibration_properties.is_calibrated = True


class Calibrate:
    def __init__(self, context):
        self.config = context.scene.calibration_properties
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
        # Hence intrinsic parameters are the same
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
        if not retR or retL:
            print('Corners not found')
            # if self.live:
            #     cv2.waitKey(0)
            return False, None, None

        cv2.cornerSubPix(imgR_gray, cornersR, (11, 11), (-1, -1), self.criteria)
        cv2.cornerSubPix(imgL_gray, cornersL, (11, 11), (-1, -1), self.criteria)

        # if self.live:
        #     imshow_concat('Corners', outputL, outputR)
        #     cv2.waitKey(0)
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
    return os.path.join(side_path, f'img{id}.png')