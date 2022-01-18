import os
import numpy as np

import cv2
import addon_utils
import bpy


class ImageImport:
    def __init__(self, res_x=3040, res_y=4056):
        addon_utils.enable("io_import_images_as_planes")

        self.to_plane = bpy.ops.import_image.to_plane
        self.res = res_y, res_x, 3
        self.dir = os.path.dirname(__file__)
        self.left_img_file = os.path.join(self.dir, "left.png")
        self.right_img_file = os.path.join(self.dir,  "right.png")


    def initialize_images(self):
        black_image = np.zeros(self.res)
        cv2.imwrite(self.left_img_file, black_image)
        cv2.imwrite(self.right_img_file, black_image)

    def initialize_planes(self):
        if not os.path.exists(self.right_img_file) or os.path.exists(self.right_img_file):
            self.initialize_images()
        self.to_plane(shader='SHADELESS', height=10,
                                      files=[{'name': self.left_img_file}, {'name': self.right_img_file}])

    def change_image(self, img, side):
        if side not in ['left', 'right']:
            raise ValueError(f'The variable side can only be \'left\' or \'right\', not {side}.')

        cur_img = f'{side}.png'
        if cur_img not in bpy.data.images:
            self.initialize_planes()

        _size = bpy.data.images[cur_img].size
        if img.shape != _size:
            resized = cv2.resize(img, _size, interpolation=cv2.INTER_AREA)
        else:
            resized = img

        rgb = np.flip(resized, axis=[0, 2])

        rgba = np.ones((_size[1], _size[0], 4), dtype=np.float32)
        rgba[:, :, :-1] = np.float32(rgb) / 255
        bpy.data.images[cur_img].pixels = rgba.flatten()
