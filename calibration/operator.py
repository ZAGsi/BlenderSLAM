import bpy
import os
# import requests

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
        scene = context.scene
        cal_props = scene.calibration_properties
        is_live = scene.SLAM_config_properties.is_live

        CAL_DIR = cal_props.path
        calibration_settings = {'calibration_dir': CAL_DIR,
                                'pathL': CAL_DIR + "/left/",
                                'pathR': CAL_DIR + "/right/",
                                'YAML': CAL_DIR + "/stereo_parameters.yaml",
                                'live': is_live,
                                'chessboard_dim': (cal_props.chess_dim_w, cal_props.chess_dim_h)}

        if is_live:
            calibration_settings['IP_addressL'] = scene.SLAM_config_properties.IP_address
            calibration_settings['IP_addressR'] = scene.SLAM_config_properties.IP_address
            calibration_settings['authorisation'] = ('admin', '')
            print("live calibrating not implemented yet...")
            return {"FINISHED"}

        calibration = Calibrate(**calibration_settings)
        if cal_props.is_calibrated:
            rectify_config = calibration.import_calibration_values()
        else:
            rectify_config = calibration.calibrate()
        print(rectify_config)

        return {"FINISHED"}

class Calibrate:
    def __init__(self, live, calibration_dir, pathL, pathR, chessboard_dim,
                 IP_addressL=None, IP_addressR=None, authorisation=None, YAML=None):
        self.live = live
        self.CAL_DIR = calibration_dir
        self.pathL = pathL
        self.pathR = pathR
        self.CHESSBOARD_DIM = chessboard_dim
        self.ipL = IP_addressL
        self.ipR = IP_addressR
        self.AUTH = authorisation
        self.YAML = YAML

        # Set parameters for calibration
        self.objp = np.zeros((self.CHESSBOARD_DIM[0] * self.CHESSBOARD_DIM[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.CHESSBOARD_DIM[0], 0:self.CHESSBOARD_DIM[1]].T.reshape(-1, 2)
        # Termination criteria for refining the detected corners
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        if not os.path.exists(self.CAL_DIR):
            os.makedirs(self.CAL_DIR)
        if not os.path.exists(pathL):
            os.makedirs(pathL)
        if not os.path.exists(pathR):
            os.makedirs(pathR)

    def calibrate(self):
        if self.live:
            print("live calibration not yet implemented")
            return
            # img_ptsL, img_ptsR, obj_pts, img_res = self.get_live_images()
        else:
            img_ptsL, img_ptsR, obj_pts, img_res = self.get_offline_images()

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
        print(f"Saving parameters to {self.YAML}......")
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
        with open(self.YAML, 'w') as outfile:
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

    # def get_live_images(self):
    #     try:
    #         img_ptsL = []
    #         img_ptsR = []
    #         obj_pts = []
    #         imgL_gray = None
    #         # Capture 11 images for calibration
    #         for i in tqdm(range(1, 12)):
    #             # Fetch data from IP address
    #             r_left = requests.get(self.ipL, auth=self.AUTH)
    #             r_right = requests.get(self.ipR, auth=self.AUTH)
    #
    #             #
    #             imgL = cv2.imdecode(np.frombuffer(r_left.content, np.uint8), flags=1)
    #             imgR = cv2.imdecode(np.frombuffer(r_right.content, np.uint8), flags=1)
    #             imgL_gray = cv2.imdecode(np.frombuffer(r_left.content, np.uint8), flags=0)
    #             imgR_gray = cv2.imdecode(np.frombuffer(r_right.content, np.uint8), flags=0)
    #             cv2.imwrite(self.pathL + "img%d.png" % i, imgL)
    #             cv2.imwrite(self.pathR + "img%d.png" % i, imgR)
    #
    #             ret, cornersL, cornersR = self.find_chessboard_corners(imgL, imgR, imgL_gray, imgR_gray)
    #             if ret:
    #                 obj_pts.append(self.objp)
    #                 img_ptsL.append(cornersL)
    #                 img_ptsR.append(cornersR)
    #
    #         img_res = (0, 0)
    #         if imgL_gray is not None:
    #             img_res = imgL_gray.shape[::-1]
    #     except:
    #         imshow_concat("failed image", imgL, imgR)
    #         cv2.waitKey(0)
    #         quit()
    #     return img_ptsL, img_ptsR, obj_pts, img_res

    def get_offline_images(self):
        img_ptsL = []
        img_ptsR = []
        obj_pts = []
        imgL_gray = None

        for i in tqdm(range(1, 12)):
            imgL = cv2.imread(self.pathL + "img%d.png" % i)
            imgR = cv2.imread(self.pathR + "img%d.png" % i)
            imgL_gray = cv2.imread(self.pathL + "img%d.png" % i, 0)
            imgR_gray = cv2.imread(self.pathR + "img%d.png" % i, 0)

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
        if not retR and retL:
            print('Corners not found')
            # if self.live:
            #     cv2.waitKey(0)
            return False, None, None

        cv2.cornerSubPix(imgR_gray, cornersR, (11, 11), (-1, -1), self.criteria)
        cv2.cornerSubPix(imgL_gray, cornersL, (11, 11), (-1, -1), self.criteria)
        cv2.drawChessboardCorners(outputR, self.CHESSBOARD_DIM, cornersR, retR)
        cv2.drawChessboardCorners(outputL, self.CHESSBOARD_DIM, cornersL, retL)

        # if self.live:
        #     imshow_concat('Corners', outputL, outputR)
        #     cv2.waitKey(0)
        return True, cornersL, cornersR

    def import_calibration_values(self):
        with open(self.YAML, 'r') as stream:
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