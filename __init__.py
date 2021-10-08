import bpy
import sys
import os
import importlib

# stereo_ptam is not a module so this is a quick fix to get it working.
# TODO: ask stereo_ptam creators to make it an installable module.
sys.path.append(os.path.join(os.path.dirname(__file__), 'SLAM', 'stereo_ptam'))

bl_info = {
    "name": "BlenderSLAM",
    "author": "Laurens J.N. Oostwegel",
    "version": (0, 2),
    "blender": (2, 92, 0),
    "location": "Scene",
    "description": "Create point clouds using a SLAM algorithm (Stereo-PTAM)",
    "warning": "",
    "wiki_url": "",
    "category": "SLAM",
}

modules = {"run": None,
           "calibration": None}


for name in modules.keys():
    modules[name] = importlib.import_module(f"BlenderSLAM.{name}")

classes = []

for mod in modules.values():
    classes.extend(mod.classes)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    for mod in modules.values():
        mod.register()

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)

    for mod in reversed(list(modules.values())):
        mod.unregister()

if __name__ == '__main__':
    register()