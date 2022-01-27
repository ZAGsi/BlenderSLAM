import bpy
from ..run.ui import SLAM_template_gui


class CalibrationGui(SLAM_template_gui, bpy.types.Panel):
    bl_parent_id = "SLAM_PT_gui"
    bl_label = "Calibration configuration"
    bl_options = {"DEFAULT_CLOSED", "HIDE_HEADER"}
    bl_order = 1

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        cal_props = scene.CalibrationProperties

        start_screen = not cal_props.is_calibrating and not cal_props.is_calibrated
        calibration_screen = (cal_props.calibration_method == "LIVE" and
                              cal_props.is_calibrating and
                              not cal_props.all_images_captured)
        compute_calibration_screen = cal_props.all_images_captured and cal_props.is_calibrating

        # Options for the start screen
        if start_screen:
            row = layout.row(align=True)
            row.prop(cal_props, "calibration_method")
            row = layout.row(align=True)
            row.prop(cal_props, "path")

            if cal_props.calibration_method == "LIVE":
                row = layout.row(align=True)
                row.operator("slam.find_rpi")
                # TODO: show IP addresses
                # TODO: ask for .pub file

            if cal_props.calibration_method == "YAML":
                row = layout.row(align=True)
                row.operator("slam.import_yaml")
            else:
                row = layout.row(align=True)
                row.label(text="Chessboard dimensions")
                row = layout.row(align=True)
                row.prop(cal_props, "chess_dim_w")
                row.prop(cal_props, "chess_dim_h")
                row = layout.row(align=True)
                row.prop(cal_props, "chess_dim_size")
                row = layout.row(align=True)
                row.prop(cal_props, "max_number_imgs")
                row = layout.row(align=True)
                row.operator("slam.calibrate")

        # options for the calibration
        elif calibration_screen:
            if not cal_props.is_image_captured:
                row = layout.row(align=True)
                row.label(text=f"Image {cal_props.current_img_id}/{cal_props.max_number_imgs}.")
                row = layout.row(align=True)
                row.operator("slam.fetch_image")
            else:
                row = layout.row(align=True)
                row.label(text=f"Image {cal_props.current_img_id}/{cal_props.max_number_imgs}.")
                row = layout.row(align=True)
                row.operator("slam.capture_image_ok")
                row = layout.row(align=True)
                row.operator("slam.capture_image_recapture")
            row = layout.row(align=True)
            row.operator("slam.cancel_calibration")

        # Options when all calibration files are captured
        elif compute_calibration_screen:
            row = layout.row(align=True)
            row.operator("slam.compute_calibration")
            row = layout.row(align=True)
            row.operator("slam.cancel_calibration")

        # Options when calibration is computed
        else:
            row = layout.row(align=True)
            row.label(text=f"calibration file: {cal_props.path}/stereo_parameters.yaml")
            row = layout.row(align=True)
            row.operator("slam.remove_calibration")



