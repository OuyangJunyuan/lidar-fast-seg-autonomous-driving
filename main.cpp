#include "patchwork.hpp"
#include "CVC_cluster.h"

#include <vector>
#include <chrono>
#include <iostream>
#include <unordered_map>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

using PCXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PXYZ = pcl::PointXYZ;
PatchWork<PXYZ> PatchworkGroundSeg;


ros::Publisher ground_publisher, no_ground_publisher, cluster_publisher;

void CallBack(const sensor_msgs::PointCloud2::ConstPtr &msgs) {
    std::cout << "=======================================" << std::endl;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msgs, cloud);

    // ground segmentation
    PCXYZ pc_ground, pc_non_ground;
    {
        auto t1 = std::chrono::system_clock::now();
        PatchworkGroundSeg.estimate_ground(cloud, pc_ground, pc_non_ground);
        auto t2 = std::chrono::system_clock::now();
        std::cout << "remove ground consume: " << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()
                  << std::endl;
        std::cout << "total points: " << cloud.size() << endl;
        std::cout << "ground points: " << pc_ground.size() << " " << (float) (pc_ground.size()) / cloud.size() << endl;
        std::cout << "non ground points: " << pc_non_ground.size() << " "
                  << (float) (pc_non_ground.size()) / cloud.size() << endl;
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pc_ground, msg);
    msg.header = msgs->header;
    ground_publisher.publish(msg);

    pcl::toROSMsg(pc_non_ground, msg);
    msg.header = msgs->header;
    no_ground_publisher.publish(msg);

    // curved_voxel clustering
    vector<float> param(3, 0);
    param[0] = 2;
    param[1] = 0.4;
    param[2] = 1.5;

    CVC Cluster(param);

    auto t1 = std::chrono::system_clock::now();
    std::vector<PointAPR> capr;
    std::unordered_map<int, Voxel> hash_table;
    vector<int> cluster_indices, cluster_id;

    PCXYZ cluster_point = pc_non_ground;
    capr.reserve(cluster_point.size());
    Cluster.calculateAPR(cluster_point, capr);
    Cluster.build_hash_table(capr, hash_table);
    cluster_indices = Cluster.cluster(hash_table, capr);
    Cluster.most_frequent_value(cluster_indices, cluster_id);
    auto t2 = std::chrono::system_clock::now();
    std::cout << "cluster consume: " << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count()
              << std::endl;
    cout << "cluster_point: " << cluster_point.points.size() << endl;
    cout << "capr: " << capr.size() << endl;
    cout << "cluster num: " << cluster_id.size() << endl;

    pcl::PointCloud<pcl::PointXYZL> cluster_cloud;
    for (int j = 0; j < cluster_id.size(); ++j) {
        for (int i = 0; i < cluster_indices.size(); ++i) {
            if (cluster_indices[i] == cluster_id[j]) {
                pcl::PointXYZL p;
                p.x = cluster_point.points[i].x;
                p.y = cluster_point.points[i].y;
                p.z = cluster_point.points[i].z;
                p.label = cluster_id[j];
                cluster_cloud.push_back(p);
            }
        }
    }
    pcl::toROSMsg(cluster_cloud, msg);
    msg.header = msgs->header;
    cluster_publisher.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cvc");
    auto node = ros::NodeHandle();
    ground_publisher = node.advertise<sensor_msgs::PointCloud2>("ground", 1);
    no_ground_publisher = node.advertise<sensor_msgs::PointCloud2>("no_ground", 1);
    cluster_publisher = node.advertise<sensor_msgs::PointCloud2>("object", 1);
    auto sub = node.subscribe("rslidar_points", 1, CallBack);
    ros::spin();
    return 0;
}



