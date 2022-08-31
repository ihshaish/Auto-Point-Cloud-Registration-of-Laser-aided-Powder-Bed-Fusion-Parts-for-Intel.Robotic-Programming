
import sys
sys.path.append('/usr/local/lib')
import pyrealsense2 as rs
from collections import namedtuple
import numpy as np
import open3d
print("Environment Ready")

def SALSA_BagToFrame(files):
    depth_scale, depth_intrin, color_intrin, depth_to_color_extrin, color_frame, depth_frame = [], [], [], [], [], []
    for i, file in enumerate(files):
        #================>Initiate Camera/File<=================#
        pipe = rs.pipeline()  # Declare RealSense pipeline, encapsulating the actual device and sensors
        cfg = rs.config()
        cfg.enable_device_from_file(file)
        profile = pipe.start(cfg)  # Start streaming with default recommended configuration
        #================>Depth Scale<======================#
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale.append(depth_sensor.get_depth_scale())  # 1/depth_scale to convert to meters
        #================>Capture/Load Frames<=================#
        # Skip 5 first frames to give the Auto-Exposure time to adjust
        for x in range(5):
            pipe.wait_for_frames()
        # Store next frameset for later processing:
        frameset = pipe.wait_for_frames()
        color_frame.append(frameset.get_color_frame())
        depth_frame.append(frameset.get_depth_frame())
        # Cleanup:
        pipe.stop()
        print("Frames Captured")
        #================>Intrinsic & Extrinsic Camera Properties<========#
        # ppx/y: center of projection, fx/y: focal length (multiple of ppx/y), model & coeffs: distortion model
        depth_stream = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        depth_intrin.append(depth_stream.get_intrinsics())  # intrinsic camera properties
        color_stream = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        color_intrin.append(color_stream.get_intrinsics())  # intrinsic camera properties
        depth_to_color_extrin.append(depth_frame[i].profile.get_extrinsics_to(color_frame[i].profile))  # extrinsic camera properties
    BagData = namedtuple('BagData', 'dscale dintrin cintrin extrin color_frame depth_frame')
    return BagData(dscale=depth_scale, dintrin=depth_intrin, cintrin=color_intrin, extrin=depth_to_color_extrin, color_frame=color_frame, depth_frame=depth_frame)


def SALSA_DepthFilter_Sharpen_ShortRange(files, NumOfFrames):
    d_frame = []
    pipe = rs.pipeline()  # Declare RealSense pipeline, encapsulating the actual device and sensors
    cfg = rs.config()
    for i, file in enumerate(files):
        cfg.enable_device_from_file(file)
        profile = pipe.start(cfg)  # Start streaming with default recommended configuration
        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        align = rs.align(align_to)
        aligned_depth_frame = []
        for x in range(NumOfFrames):
            frameset = pipe.wait_for_frames()
            aligned_frameset = align.process(frameset)
            aligned_depth_frame.append(aligned_frameset.get_depth_frame())
        # Cleanup:
        pipe.stop()
        print("Frames Captured")
        # Depth Filters
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.filter_magnitude, 5)
        spatial.set_option(rs.option.filter_smooth_alpha, 1)
        spatial.set_option(rs.option.filter_smooth_delta, 50)
        spatial.set_option(rs.option.holes_fill, 3)
        temporal = rs.temporal_filter()
        hole_filling = rs.hole_filling_filter()
        for x in range(NumOfFrames):
            dframe = aligned_depth_frame[x]
            dframe = spatial.process(dframe)
            dframe = temporal.process(dframe)
            dframe = hole_filling.process(dframe)
        d_frame.append(dframe)
    return d_frame


def SALSA_CropImage(color, depth_nofilter, depth_filter):
    color_raw, depth_raw, depth_raw_filter = [], [], []
    for i in range(len(color)):
        target = 300
        height, width = color[i].shape[:2]
        aspect = width / height
        crop_start_x = round(width / 2) - 75
        crop_start_y = round(height / 2) - 50
        crop_color = color[i][crop_start_y:(crop_start_y + target), crop_start_x:(crop_start_x + round(target * 0.7))].astype(np.uint8)
        crop_depth_nofilter = depth_filter[i][crop_start_y:(crop_start_y + target), crop_start_x:(crop_start_x + round(target * 0.7))].astype(np.uint16)
        crop_depth_filter = depth_filter[i][crop_start_y:(crop_start_y + target), crop_start_x:(crop_start_x + round(target * 0.7))].astype(np.uint16)
        color_raw.append(open3d.Image(crop_color))
        depth_raw.append(open3d.Image(crop_depth_nofilter))
        depth_raw_filter.append(open3d.Image(crop_depth_filter))
    return color_raw, depth_raw, depth_raw_filter


def SALSA_ImageToPointCloud(color_raw, depth_raw, depth_raw_filter, dscale, dintrin, extrin):
    pcd, pcd_filter = [], []
    for i in range(len(color_raw)):
        rgbd = open3d.create_rgbd_image_from_color_and_depth(color_raw[i], depth_raw[i], depth_scale=1 / dscale[i], depth_trunc=0.7, convert_rgb_to_intensity=False)
        rgbd_filter = open3d.create_rgbd_image_from_color_and_depth(color_raw[i], depth_raw_filter[i], depth_scale=1 / dscale[i], depth_trunc=0.7, convert_rgb_to_intensity=False)
        intrinsics = open3d.camera.PinholeCameraIntrinsic(dintrin[i].width, dintrin[i].height, dintrin[i].fx, dintrin[i].fy, dintrin[i].ppx, dintrin[i].ppy)
        extrin_rot = np.reshape(extrin[i].rotation, (3, 3))
        extrin_trans = np.reshape(extrin[i].translation, (3, 1))
        extrin_stack = np.hstack((extrin_rot, extrin_trans))
        extrinsics = np.vstack((extrin_stack, [0, 0, 0, 1]))
        pcd.append(open3d.geometry.create_point_cloud_from_rgbd_image(rgbd, intrinsics, extrinsics))
        pcd_filter.append(open3d.geometry.create_point_cloud_from_rgbd_image(rgbd_filter, intrinsics, extrinsics))
    return pcd, pcd_filter
    np.resh
