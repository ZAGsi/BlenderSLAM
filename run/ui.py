import bpy

class PTAM_PT_gui(bpy.types.Panel):
    """Creates a Panel in the scene context of properties editor"""
    bl_idname = "PTAM_PT_gui"
    bl_label = "PTAM"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "scene"

    def draw(self, context):
        layout = self.layout

        scene = context.scene
        props = scene.PTAM_properties

        layout.label(text="PTAM arguments:")
        row = layout.row(align=True)
        row.prop(props, "dataset")
        row = layout.row(align=True)
        row.prop(props, "path")
        row = layout.row(align=True)
        row.prop(props, "update_speed")
        row = layout.row(align=True)
        row.prop(props, "max_images")
        row = layout.row(align=True)
        row.operator("ptam.startalgorithm")
