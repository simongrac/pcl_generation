# Point Cloud from texture & depth map

This code is used for generating point cloud (with type pcl::PointCloud<pcl::PointXYZRGB>) from RGB texture image and depth map. This data was previously obtained from OpenGL program, where we are simulating reprojection of 3D objects (bin in this case) back to the 2D images + coresponding depth maps from different views.

TODOs (problems):
- (probably) problems with Opencv Mat type, we are dont know why we have to multiply column value with constant 1.5 when we are reading from depth map. When we are not doing this, generated point cloud has z projection extended along x axe.
- generated point cloud has weird stair-like structure

![generated point cloud sample](https://github.com/simongrac/pcl_generation/blob/master/sample.png)

There are Texture files in to_pcl/texture folder, depth maps in to_pcl/depth folder and generated pointclouds in _pcl/pointcloud folder
