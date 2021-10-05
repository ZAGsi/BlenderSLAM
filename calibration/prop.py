import bpy

class calibration_properties(bpy.types.PropertyGroup):
    calibrated: bpy.props.BoolProperty(name="calibrated", default=False)
    path: bpy.props.StringProperty(name="path", default="~/Documents/KITTI/dataset/sequences/00/calibration.yaml")