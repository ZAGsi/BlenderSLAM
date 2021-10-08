import bpy

class calibration_properties(bpy.types.PropertyGroup):
    is_calibrated: bpy.props.BoolProperty(name="calibrated", default=False)
    path: bpy.props.StringProperty(name="path", default="", subtype='DIR_PATH')
    chess_dim_w: bpy.props.IntProperty(name="width", default=13)
    chess_dim_h: bpy.props.IntProperty(name="height", default=9)
    is_calibrating: bpy.props.BoolProperty(name="calibrating", default=False)
    current_img_id: bpy.props.IntProperty(name="img_id", default=1)
    n_imgs:  bpy.props.IntProperty(name="img_id", default=11)
    is_image_captured: bpy.props.BoolProperty(name="calibrating", default=False)
    all_images_captured: bpy.props.BoolProperty(name="calibrating", default=False)
