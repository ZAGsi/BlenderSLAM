import bpy


class PTAM_properties(bpy.types.PropertyGroup):
    path: bpy.props.StringProperty(name="path", default="C:\\Users\\oostwegel\\Documents\\SensorVision\\DemoSet\\KITTI odometry\\dataset\\sequences\\00")
    dataset: bpy.props.StringProperty(name="dataset", default="KITTI")
    update_speed: bpy.props.FloatProperty(name="no-viz", default=0.5)
    max_images: bpy.props.IntProperty(name="max-images", default=0)