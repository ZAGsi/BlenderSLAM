import bpy
from . import prop
from . import operator
from . import ui

bl_info = {
    "name": "S-PTAM",
    "author": "Laurens Oostwegel",
    "version": (0, 1),
    "blender": (2, 92, 0),
    "location": "Scene",
    "description": "Visualize, edit and export 3D City Models encoded in CityJSON format",
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