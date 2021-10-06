import bpy
from . import operator
from . import prop
from . import ui

classes = (prop.calibration_properties,
           operator.calibrate_OT_operator,
           ui.SLAM_PT_calibration_gui)

def register():
    bpy.types.Scene.calibration_properties = bpy.props.PointerProperty(type=prop.calibration_properties)

def unregister():
    del bpy.types.Scene.calibration_properties