import bpy
import sys
import os

# stereo_ptam is not a module so this is a quick fix to get it working.
# TODO: ask stereo_ptam creators to make it an installable module.
sys.path.append(os.path.join(os.path.dirname(__file__), 'stereo_ptam'))

from . import prop
from . import operator
from . import ui
from setuptools import setup

bl_info = {
    "name": "S-PTAM",
    "author": "Laurens Oostwegel",
    "version": (0, 1),
    "blender": (2, 93, 0),
    "location": "Scene",
    "description": "Create point clouds using the Stereo-PTAM algorithm",
    "warning": "",
    "wiki_url": "",
    "category": "SLAM",
}


classes = (
    operator.PTAM_OT_operator,
    ui.PTAM_PT_gui,
    prop.PTAM_properties
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Scene.PTAM_properties = bpy.props.PointerProperty(type=prop.PTAM_properties)

def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)

    del bpy.types.Scene.PTAM_properties

if __name__ == '__main__':
    register()