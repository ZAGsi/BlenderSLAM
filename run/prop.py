import bpy


class SLAM_config_properties(bpy.types.PropertyGroup):
    is_live: bpy.props.BoolProperty(name="Live capture", default=False)
    acquisition_method: bpy.props.EnumProperty(name="Acquisition method", default="TEST", items=[
        ("IP", "IP-address", ""),
        ("USB", "USB", ""),
        ("TEST", "Test environment", "")])
    IP_address: bpy.props.StringProperty(name="IP Address", default="192.168.0.0")
    USB_port: bpy.props.StringProperty(name="USB Port", default="")
    path: bpy.props.StringProperty(name="Dataset path", default="", subtype='DIR_PATH')
    standard_set: bpy.props.EnumProperty(name="Standard dataset", description="",
                                         items=[("NO", "No standard set", ""),
                                                ("KITTI", "KITTI", ""),
                                                ("EuRoC", "EuRoC", "")
                                                ])


class SLAM_properties(bpy.types.PropertyGroup):
    SLAM_method: bpy.props.EnumProperty(name="method", description="",
                                        items=[("STEREOPTAM", "Stereo-PTAM", "")
                                               ])
    feature_descriptor: bpy.props.EnumProperty(name="Feature descriptor", default="GFTT-BRIEF", items=[
        ("GFTT-BRIEF", "GFTT-BRIEF", ""),
        ("GFTT-BRISK", "GFTT-BRISK", ""),
        ("ORB-ORB", "ORB-ORB", "")])
    output_path: bpy.props.StringProperty(name="Output file path", default="", subtype='FILE_PATH')
    viz: bpy.props.BoolProperty(name="Visualisation", default=True)
    update_speed: bpy.props.FloatProperty(name="Update speed", default=0.5)
    max_images: bpy.props.IntProperty(name="Max images", default=0)
