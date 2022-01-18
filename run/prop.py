from bpy.types import PropertyGroup
from bpy.props import BoolProperty, EnumProperty, StringProperty, FloatProperty, IntProperty


class SLAMAcquisitionProperties(PropertyGroup):
    is_live: BoolProperty(name="Live capture", default=True)
    acquisition_method: EnumProperty(name="Acquisition method", default="IP", items=[
        ("IP", "IP-address", ""),
        ("USB", "USB", ""),
        ("TEST", "Test environment", "")])
    IP_address: StringProperty(name="IP Address", default="192.168.0.0")
    USB_port: StringProperty(name="USB Port", default="")
    path: StringProperty(name="Dataset path", default="", subtype='DIR_PATH')
    standard_set: EnumProperty(name="Standard dataset", description="",
                                         items=[("NO", "No standard set", ""),
                                                ("KITTI", "KITTI", ""),
                                                ("EuRoC", "EuRoC", "")
                                                ])


class SLAMSettings(PropertyGroup):
    SLAM_method: EnumProperty(name="method", description="",
                                        items=[("STEREOPTAM", "Stereo-PTAM", "")
                                               ])
    feature_descriptor: EnumProperty(name="Feature descriptor", default="GFTT-BRIEF", items=[
        ("GFTT-BRIEF", "GFTT-BRIEF", ""),
        ("GFTT-BRISK", "GFTT-BRISK", ""),
        ("ORB-ORB", "ORB-ORB", "")])
    output_path: StringProperty(name="Output file path", default="", subtype='FILE_PATH')
    viz: BoolProperty(name="Visualisation", default=True)
    update_speed: FloatProperty(name="Update speed in seconds", default=0.5)
    max_images: IntProperty(name="Max images", default=0)
