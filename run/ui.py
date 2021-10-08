import bpy

class SLAM_template_gui:
    """Creates a Panel in the scene context of properties editor"""
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "scene"

class SLAM_PT_config_gui(SLAM_template_gui,bpy.types.Panel):
    bl_idname = "SLAM_PT_config_gui"
    bl_label = "SLAM"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        config_props = scene.SLAM_config_properties

        row = layout.row(align=True)
        row.prop(config_props, "is_live")
        row = layout.row(align=True)

        row.prop(config_props, "acquisition_method")  # TODO: implement if-statement for rows following this one
        if not config_props.is_live:
            row.enabled = False

        row = layout.row(align=True)
        row.prop(config_props, "IP_address")
        if not config_props.is_live or config_props.acquisition_method != "IP":
            row.enabled = False

        row = layout.row(align=True)
        row.prop(config_props, "USB_port")
        if not config_props.is_live or config_props.acquisition_method != "USB":
            row.enabled = False

        row = layout.row(align=True)
        row.prop(config_props, "path")
        row = layout.row(align=True)
        row.prop(config_props, "standard_set")  # previous: dataset

class SLAM_PT_gui(SLAM_template_gui, bpy.types.Panel):
    bl_parent_id = "SLAM_PT_config_gui"
    bl_label = "SLAM configuration"
    bl_options = {"DEFAULT_CLOSED"}
    bl_order = 1


    def draw(self, context):
        layout = self.layout
        scene = context.scene
        SLAM_props = scene.SLAM_properties

        row = layout.row(align=True)
        row.prop(SLAM_props, "SLAM_method")
        row = layout.row(align=True)
        row.prop(SLAM_props, "output_path")
        row = layout.row(align=True)
        row.prop(SLAM_props, "viz")
        row = layout.row(align=True)
        row.prop(SLAM_props, "update_speed")
        row = layout.row(align=True)
        row.prop(SLAM_props, "max_images")
        row = layout.row(align=True)
        row.operator("slam.startalgorithm")
