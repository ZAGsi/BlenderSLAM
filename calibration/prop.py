import bpy

class calibration_properties(bpy.types.PropertyGroup):
    is_calibrated: bpy.props.BoolProperty(name="calibrated", default=False)
    path: bpy.props.StringProperty(name="path", default="~/Documents/KITTI/dataset/sequences/00/calibration")
    chess_dim_w: bpy.props.IntProperty(name="Width", default=13)
    chess_dim_h: bpy.props.IntProperty(name="Height", default=9)