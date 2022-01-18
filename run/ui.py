from bpy.types import Panel

class SLAM_template_gui:
    """Creates a Panel in the scene context of properties editor"""
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "scene"


class SLAMGui(SLAM_template_gui, Panel):
    bl_idname = "SLAM_PT_gui"
    bl_label = "SLAM"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        calibration_props = scene.CalibrationProperties
        slam_props = scene.SLAMAcquisitionProperties
        row = layout.row(align=True)
        row.operator("slam.run_slam")
        if not calibration_props.is_calibrated and slam_props.standard_set == "NO":
            row.enabled = False
            row = layout.row(align=True)
            row.label(text="Calibrate first before SLAM can be run.")


class AcquisitionGui(SLAM_template_gui, Panel):
    bl_parent_id = "SLAM_PT_gui"
    bl_label = "Acquisition settings"
    bl_options = {"DEFAULT_CLOSED"}
    bl_order = 3

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        acquisition_props = scene.SLAMAcquisitionProperties

        row = layout.row(align=True)
        row.prop(acquisition_props, "is_live")
        row = layout.row(align=True)

        row.prop(acquisition_props, "acquisition_method")
        if not acquisition_props.is_live:
            row.enabled = False

        row = layout.row(align=True)
        row.prop(acquisition_props, "IP_address")
        if not acquisition_props.is_live or acquisition_props.acquisition_method != "IP":
            row.enabled = False            

        row = layout.row(align=True)
        row.prop(acquisition_props, "USB_port")
        if not acquisition_props.is_live or acquisition_props.acquisition_method != "USB":
            row.enabled = False

        row = layout.row(align=True)
        row.prop(acquisition_props, "path")
        row = layout.row(align=True)
        row.prop(acquisition_props, "standard_set")  # previous: dataset


class SLAMSettingsGui(SLAM_template_gui, Panel):
    bl_parent_id = "SLAM_PT_gui"
    bl_label = "SLAM configuration"
    bl_options = {"DEFAULT_CLOSED"}
    bl_order = 2

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        slam_settings = scene.SLAMSettings

        row = layout.row(align=True)
        row.prop(slam_settings, "SLAM_method")
        row = layout.row(align=True)
        row.prop(slam_settings, "feature_descriptor")
        row = layout.row(align=True)
        row.prop(slam_settings, "output_path")
        row = layout.row(align=True)
        row.prop(slam_settings, "viz")
        row = layout.row(align=True)
        row.prop(slam_settings, "update_speed")
        row = layout.row(align=True)
        row.prop(slam_settings, "max_images")
