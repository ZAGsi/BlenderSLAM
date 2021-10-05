import bpy


class SLAM_config_properties(bpy.types.PropertyGroup):
    is_live: bpy.props.BoolProperty(name="Live capture", default=False)
    acquisition_method: bpy.props.EnumProperty(name="Acquisition method", items=[
        ("IP", "IP-address", ""),
        ("USB", "USB", "")
    ])
    IP_address: bpy.props.StringProperty(name="IP Address", default="192.168.0.0")
    USB_address: bpy.props.StringProperty(name="USB Port", default="")
    path: bpy.props.StringProperty(name="Dataset path", default="~/Documents/KITTI/dataset/sequences/00")
    standard_set: bpy.props.EnumProperty(name="Standard dataset", description="",
                                         items=[("NO", "No standard set", ""),
                                                ("KITTI", "KITTI", "")
                                                ])


class SLAM_properties(bpy.types.PropertyGroup):
    SLAM_method: bpy.props.EnumProperty(name="method", description="",
                                        items=[("STEREOPTAM", "Stereo-PTAM", "")
                                               ])
    output_path: bpy.props.StringProperty(name="Output path", default="~/Documents/KITTI/pointcloud.txt")
    viz: bpy.props.BoolProperty(name="Visualisation", default=True)
    update_speed: bpy.props.FloatProperty(name="Update speed", default=0.5)
    max_images: bpy.props.IntProperty(name="Max images", default=0)
