from bpy.types import Operator
import subprocess
import os


class BatchExecutor:
    def call_subprocess(self, commands):
        dir_name = os.path.dirname(__file__)
        subprocess.run(commands, cwd=os.path.join(dir_name, 'bat'), shell=True)

# image_capture (MODAL) (Start/Stop) --> fetchNSeconds.bat ## DEPRECATED ##
# class capture_image_OT_operator(bpy.types.Operator, BatchExecutor):
#     bl_idname = "slam.capture_image"
#     bl_label = "Capture image"
#     bl_context = "scene"
#
#     def execute(self, context):
#         self.call_subprocess(['StartFetching.bat'])


class FindRPI(Operator, BatchExecutor):
    bl_idname = "slam.find_rpi"
    bl_label = "Find Raspberry Pi IP address"
    bl_context = "scene"

    def execute(self, context):
        self.call_subprocess("FindCameras.bat")

        return {'FINISHED'}


# Clean RPI camera
class CleanRPI(Operator, BatchExecutor):
    bl_idname = "slam.clean"
    bl_label = "Clean input/output"
    bl_context = "scene"

    def execute(self, context):
        scene = context.scene
        props = scene.rpi_properties

        commands = []
        if props.clean_cameras:
            commands.append('CleanCameras.bat')
        if props.clean_input:
            commands.append('CleanInput.bat')
        if props.clean_output:
            commands.append('CleanOutput.bat')
        if commands:
            self.call_subprocess(commands)

        return {"FINISHED"}


class StartVideo(Operator, BatchExecutor):
    bl_idname = "slam.start_video"
    bl_label = "Start video capture"
    bl_context = "scene"

    def execute(self, context):
        self.call_subprocess('StartFetching.bat')
        scene = context.scene
        config_props = scene.rpi_properties
        config_props.running = "VIDEO"

        return {"FINISHED"}


class StopVideo(Operator, BatchExecutor):
    bl_idname = "slam.stop_video"
    bl_label = "Stop video capture"
    bl_context = "scene"

    def execute(self, context):
        self.call_subprocess('StopFetching.bat')
        scene = context.scene
        config_props = scene.rpi_properties
        config_props.running = "NOTHING"

        return {"FINISHED"}


class StartStill(Operator, BatchExecutor):
    bl_idname = "slam.start_still"
    bl_label = "Start still capture"
    bl_context = "scene"

    def execute(self, context):
        self.call_subprocess('InitializeImageCaptures.bat')
        scene = context.scene
        config_props = scene.rpi_properties
        config_props.running = "STILL"

        return {"FINISHED"}


class StopStill(Operator, BatchExecutor):
    bl_idname = "slam.stop_still"
    bl_label = "Stop still capture"
    bl_context = "scene"

    def execute(self, context):
        self.call_subprocess('StopImageCaptures.bat')
        scene = context.scene
        config_props = scene.rpi_properties
        config_props.running = "NOTHING"

        return {"FINISHED"}


class CaptureStill(Operator, BatchExecutor):
    bl_idname = "slam.still"
    bl_label = "Capture still image"
    bl_context = "scene"

    def execute(self, context):
        scene = context.scene
        destination = scene.CalibrationProperties.path
        self.call_subprocess('CaptureImageAtCameras.bat')
        self.call_subprocess(['MoveImages.bat', destination])

        return {"FINISHED"}


class StartStream(Operator, BatchExecutor):
    bl_idname = "slam.start_stream"
    bl_label = "Start to stream cameras"
    bl_context = "scene"

    def execute(self, context):
        self.call_subprocess('StartStreamingOnCameras.bat')
        scene = context.scene
        config_props = scene.rpi_properties
        config_props.running = "STREAM"

        return {"FINISHED"}


class StopStream(Operator, BatchExecutor):
    bl_idname = "slam.stop_stream"
    bl_label = "Stop to stream cameras"
    bl_context = "scene"

    def execute(self, context):
        self.call_subprocess('StopStreamingOnCameras.bat')
        scene = context.scene
        config_props = scene.rpi_properties
        config_props.running = "NOTHING"

        return {"FINISHED"}
