import bpy
from . import prop
from . import operator
from . import ui

classes = (
    operator.PTAM_OT_operator,
    ui.PTAM_PT_gui,
    prop.PTAM_properties
)

def register():
    bpy.types.Scene.PTAM_properties = bpy.props.PointerProperty(type=prop.PTAM_properties)

def unregister():
    del bpy.types.Scene.PTAM_properties
