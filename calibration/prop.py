from bpy.props import BoolProperty, StringProperty, IntProperty, EnumProperty
from bpy.types import PropertyGroup
from . import image_to_plane


class CalibrationProperties(PropertyGroup):
    calibration_method: EnumProperty(name="calibration_method", description="", default="LIVE",
                                     items=[("LIVE", "Live calibration with IP", ""),
                                            ("OFFLINE", "Offline calibration from images", ""),
                                            ("YAML", "Import YAML file", "")])

    is_calibrated: BoolProperty(name="Is calibrated?", default=False)
    path: StringProperty(name="Calibration path", subtype='DIR_PATH')
    chess_dim_w:IntProperty(name="width", default=13)
    chess_dim_h: IntProperty(name="height", default=9)
    is_calibrating: BoolProperty(name="Is calibrating", default=False)
    current_img_id: IntProperty(name="Image ID", default=1)
    n_imgs: IntProperty(name="Amount of calibration images", default=11)
    is_image_captured: BoolProperty(name="Image captured?", default=False)
    all_images_captured: BoolProperty(name="All images captured?", default=False)
    res_x: IntProperty(name="res_x", default=1024)
    res_y: IntProperty(name="res_y", default=720)
