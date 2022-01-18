import bpy
from . import operator
from . import prop
from . import ui
# from . import image_to_plane

classes = (prop.CalibrationProperties,
           operator.calibrate_OT_operator,
           operator.compute_calibration_OT_operator,
           operator.fetch_image_OT_operator,
           operator.capture_image_OK_OT_operator,
           operator.capture_image_recapture_OT_operator,
           operator.CancelCalibrationOperator,
           operator.ImportYAMLFiles,
           operator.RemoveCalibration,
           operator.EndCalibration,
           operator.StartCalibration,
           ui.CalibrationGui
           )


def register():
    bpy.types.Scene.CalibrationProperties = bpy.props.PointerProperty(type=prop.CalibrationProperties)


def unregister():
    del bpy.types.Scene.CalibrationProperties