import bpy
from ..run.ui import SLAM_template_gui

class SLAM_PT_calibration_gui(SLAM_template_gui, bpy.types.Panel):
    bl_parent_id = "SLAM_PT_config_gui"
    bl_label = "Calibration configuration"
    bl_options = {"DEFAULT_CLOSED"}
    bl_order = 0

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        cal_props = scene.calibration_properties

        row = layout.row(align=True)
        row.prop(cal_props, "is_calibrated")
        row = layout.row(align=True)
        row.prop(cal_props, "path")
        row = layout.row(align=True)
        row.label(text="Chessboard dimensions")
        row = layout.row(align=True)
        row.prop(cal_props, "chess_dim_w")
        row.prop(cal_props, "chess_dim_h")
        row = layout.row(align=True)
        row.operator("slam.calibrate")