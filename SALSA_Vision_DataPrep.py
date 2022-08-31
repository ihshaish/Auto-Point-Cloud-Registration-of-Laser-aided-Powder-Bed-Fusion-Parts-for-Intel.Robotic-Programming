
# ============================================= #
import RS_functions as rsf
import numpy as np
import matplotlib.pyplot as plt
import open3d

files = (
        ("./RawData/SALSA_IntelRS_Testing_01.bag"),
        ("./RawData/SALSA_IntelRS_Testing_02.bag"),
        ("./RawData/SALSA_IntelRS_Testing_03.bag")
)

#--------------------->Camera Properties<------------------------#
p = rsf.SALSA_BagToFrame(files)
#--------------------->Depth Filtering<--------------------------#
NumOfFrames = 10
d_frame = rsf.SALSA_DepthFilter_Sharpen_ShortRange(files, NumOfFrames)
#-------------------->Frames to Array<---------------------------#
color, depth_nofilter, depth_filter = [], [], []
for i in range(len(files)):
    color.append(np.asanyarray(p.color_frame[i].get_data()))
    depth_nofilter.append(np.asanyarray(p.depth_frame[i].get_data()))
    depth_filter.append(np.asanyarray(d_frame[i].get_data()))
#-------------------->Image Cropping<----------------------------#
color_raw, depth_raw, depth_raw_filter = rsf.SALSA_CropImage(color, depth_nofilter, depth_filter)
fig, axes = plt.subplots(1, len(files), figsize=(10, 3))
for i, file, ax in zip(range(len(files)), files, axes):
    ax.imshow(color_raw[i])
    ax.set_title(file[7:])
plt.show()
#------------------>Point Cloud Creation<------------------------#
pcd, pcd_filter = rsf.SALSA_ImageToPointCloud(color_raw, depth_raw, depth_raw_filter, p.dscale, p.dintrin, p.extrin)
#------------------->Save Point Cloud<---------------------------#
# for i, file in enumerate(files):
#    open3d.write_point_cloud(file[7:]+".pcd", pcd[i])
#    open3d.write_point_cloud(file[7:] + "_filter.pcd", pcd_filter[i])
open3d.draw_geometries([*pcd[:]])
open3d.draw_geometries([*pcd_filter[:]])
