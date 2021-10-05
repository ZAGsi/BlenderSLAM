import bpy

class calibrate_OT_operator(bpy.types.Operator):
    bl_idname = "ptam.calibrate"
    bl_label = "Calibrate stereo cameras"
    bl_context = "scene"

    def execute(self, context):
        print("caliibrating not implemented yet...")
        return {"FINISHED"}