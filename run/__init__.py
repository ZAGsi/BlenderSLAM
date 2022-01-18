import bpy
from . import prop
from . import operator
from . import ui

classes = (
    prop.SLAMAcquisitionProperties,
    prop.SLAMSettings,
    operator.RunSLAM,
    ui.SLAMGui,
    ui.AcquisitionGui,
    ui.SLAMSettingsGui
)

def register():
    bpy.types.Scene.SLAMAcquisitionProperties = bpy.props.PointerProperty(type=prop.SLAMAcquisitionProperties)
    bpy.types.Scene.SLAMSettings = bpy.props.PointerProperty(type=prop.SLAMSettings)

def unregister():
    del bpy.types.Scene.SLAMAcquisitionProperties
    del bpy.types.Scene.SLAMSettings
