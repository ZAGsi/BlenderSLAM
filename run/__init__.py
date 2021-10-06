import bpy
from . import prop
from . import operator
from . import ui

classes = (
    prop.SLAM_config_properties,
    prop.SLAM_properties,
    operator.SLAM_OT_operator,
    ui.SLAM_PT_config_gui,
    ui.SLAM_PT_gui
)

def register():
    bpy.types.Scene.SLAM_config_properties = bpy.props.PointerProperty(type=prop.SLAM_config_properties)
    bpy.types.Scene.SLAM_properties = bpy.props.PointerProperty(type=prop.SLAM_properties)

def unregister():
    del bpy.types.Scene.SLAM_config_properties
    del bpy.types.Scene.SLAM_properties
