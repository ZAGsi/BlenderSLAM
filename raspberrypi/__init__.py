import bpy
from . import operator
from . import prop
from . import ui

classes = (prop.RPiProperties,
           operator.StartStill,
           operator.StartVideo,
           operator.StartStream,
           operator.StopStill,
           operator.StopVideo,
           operator.StopStream,
           operator.CaptureStill,
           operator.CleanRPI,
           operator.FindRPI,
           ui.RPIGui)


def register():
    bpy.types.Scene.rpi_properties = bpy.props.PointerProperty(type=prop.RPiProperties)


def unregister():
    del bpy.types.Scene.rpi_properties