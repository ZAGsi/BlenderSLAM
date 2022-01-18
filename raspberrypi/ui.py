import bpy
from ..run.ui import SLAM_template_gui


class RPIGui(SLAM_template_gui, bpy.types.Panel):
    bl_parent_id = "SLAM_PT_gui"
    bl_label = "RPI configuration"
    bl_options = {"DEFAULT_CLOSED"}
    bl_order = 4

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        config_props = scene.rpi_properties

        if config_props.running == "NOTHING":
            row = layout.row(align=True)
            row.operator("slam.find_rpi")
            row = layout.row(align=True)
            row.prop(config_props, "clean_cameras")
            row.prop(config_props, "clean_input")
            row.prop(config_props, "clean_output")
            row = layout.row(align=True)
            row.operator("slam.clean")
            row = layout.row(align=True)
            row.operator("slam.start_video")
            row = layout.row(align=True)
            row.operator("slam.start_still")
            row = layout.row(align=True)
            row.operator("slam.start_stream")

        elif config_props.running == "VIDEO":
            row = layout.row(align=True)
            row.operator("slam.stop_video")

        elif config_props.running == "STREAM":
            row = layout.row(align=True)
            row.operator("slam.stop_stream")

        elif config_props.running == "STILL":
            row = layout.row(align=True)
            row.operator("slam.still")
            row = layout.row(align=True)
            row.operator("slam.stop_still")
