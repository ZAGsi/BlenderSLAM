import bpy
from . import operator
from . import prop

classes = (prop.calibration_properties,
           operator.calibrate_OT_operator)

def register():
    bpy.types.Scene.calibration_properties = bpy.props.PointerProperty(type=prop.calibration_properties)

def unregister():
    del bpy.types.Scene.calibration_properties