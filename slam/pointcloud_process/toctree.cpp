#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

int main (int argc, char** argv)
{
    const char *path_2_cloud = "../map.pcd";
    // Load point cloud data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	int ret = pcl::io::loadPCDFile (path_2_cloud, *cloud);
    if (ret < 0) {
        PCL_ERROR("Couldn't read file %s\n", path_2_cloud);
        return -1;
    }
    // Build octree
    float resolution = 0.1f; // Resolution = 10cm
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(resolution); 
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    // Extract occupied voxel centers and visualize as points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_centers_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Traverse the octree and extract occupied voxel centers
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::LeafNodeIterator it;
    int num_leaf_node = 0;
    for (it = octree.leaf_depth_begin(); it != octree.leaf_depth_end(); ++it)
    {
        pcl::octree::OctreeContainerPointIndices &leaf = it.getLeafContainer();
        pcl::Indices indices;
        // std::vector<int> indices;
        leaf.getPointIndices(indices); // Indices of points in the voxel   
        pcl::PointXYZRGB center(0, 0, 0);
        for (const auto& index : indices)
        {
            center.x += cloud->points[index].x;
            center.y += cloud->points[index].y;
            center.z += cloud->points[index].z;
            center.r = cloud->points[index].r;
            center.g = cloud->points[index].g;
            center.b = cloud->points[index].b;
        }
        center.x /= indices.size();
        center.y /= indices.size();
        center.z /= indices.size();

        voxel_centers_cloud->push_back(center);
        num_leaf_node += 1;
    }

    // Visualize octree
    pcl::visualization::PCLVisualizer viewer("Octree Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud<pcl::PointXYZRGB>(voxel_centers_cloud, "voxel_centers_cloud");
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
    
    cout<<"origin size = "<<cloud->size()<<endl;
    cout<<"octree leaf node = "<<num_leaf_node<<endl;
    return 0;
}
