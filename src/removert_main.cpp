#include "removert/Removerter.h"

std::map<int, int> keyframe_sequence_grouping(int index_head, int index_tail, int group_size = 300)
{
    int head = index_head;
    int tail = head + group_size - 1;
    std::map<int, int> res;

    while (true)
    {
        res[head] = tail;

        if (tail >= index_tail - 1)
            break;
        else
        {
            head = tail + 1;
            tail = head + group_size - 1;
        }
    }

    return res;
}

void octreeDownsampling(const pcl::PointCloud<PointType>::Ptr &src, pcl::PointCloud<PointType>::Ptr &map_ds, const double &save_resolution)
{
    pcl::octree::OctreePointCloudVoxelCentroid<PointType> octree(save_resolution);
    octree.setInputCloud(src);
    octree.defineBoundingBox();
    octree.addPointsFromInputCloud();
    pcl::octree::OctreePointCloudVoxelCentroid<PointType>::AlignedPointTVector centroids;
    octree.getVoxelCentroids(centroids);

    map_ds->points.assign(centroids.begin(), centroids.end());
    map_ds->width = 1;
    map_ds->height = map_ds->points.size();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "removert");
    ROS_INFO("\033[1;32m----> Removert Main Started.\033[0m");

    double save_globalmap_resolution;
    ros::param::param("removert/save_globalmap_resolution", save_globalmap_resolution, 0.2);

    std::shared_ptr<Removerter> rmv_ptr = std::make_shared<Removerter>();
    const auto map_static_save_dir = rmv_ptr->map_static_save_dir_;
    auto keyframe_groups = keyframe_sequence_grouping(0, rmv_ptr->sequence_scan_paths_.size() - 1);
    rmv_ptr.reset();

    pcl::PointCloud<PointType>::Ptr pcl_map_full(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr pcl_submap(new pcl::PointCloud<PointType>());

    for (auto &group : keyframe_groups)
    {
        Removerter RMV;
        RMV.start_idx_ = group.first;
        RMV.end_idx_ = group.second;
        RMV.run();

        pcl::io::loadPCDFile(map_static_save_dir + "/StaticMapScansideMapGlobal.pcd", *pcl_submap);
        *pcl_map_full += *pcl_submap;
    }

    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_static_save_dir + "/map_full.pcd", *pcl_map_full);
    std::cout << "map_full has saved!" << std::endl;

    octreeDownsampling(pcl_map_full, pcl_map_full, save_globalmap_resolution);

    pcd_writer.writeBinary(map_static_save_dir + "/globalmap.pcd", *pcl_map_full);
    std::cout << "map (downsamp) has saved!" << std::endl;

    return 0;
}