from bpy.types import PropertyGroup
from bpy.props import StringProperty, EnumProperty, BoolProperty

class RPiProperties(PropertyGroup):
    pub_file_left: StringProperty(name="pub_file", default="", subtype='FILE_PATH')
    pub_file_right:  StringProperty(name="pub_file", default="", subtype='FILE_PATH')
    ip_left: StringProperty(name="ip_left", default="")
    ip_right: StringProperty(name="ip_left", default="")
    running: EnumProperty(name="running", description="", default="NOTHING",
                         items=[("NOTHING", "Nothing is running", ""),
                                ("VIDEO", "Video capture is running", ""),
                                ("STREAM", "Streaming", ""),
                                ("STILL", "Still capture is running", "")
                                ])
    clean_cameras: BoolProperty(name="clean_cameras", default=False)
    clean_input: BoolProperty(name="clean_input", default=False)
    clean_output: BoolProperty(name="clean_output", default=False)
