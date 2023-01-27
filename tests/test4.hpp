/*
    TEST 4: Nearest neighbour search
    Description: This test checks if the nearest neighbour search is working correctly.
*/

#include <iostream>

#include "../include/VDBWrapper.hpp"

namespace test
{

// return false if test fails and true if test passes

bool test4(std::string path)
{
    try
    {
        // grid resolution 
        float res = 0.1;

        // create the grid
        using TreeType = openvdb::tree::Tree4<float, 5, 3, 2>::Type;
        using PointType = pcl::PointXYZ;
        vdb::VDBWrapper<TreeType, PointType> grid(res);

        // create a point cloud
        pcl::PointCloud <PointType> cloud;
        
        // add 10 random points to the cloud
        for (int i = 0; i < 10; i++)
        {
            pcl::PointXYZ pt;
            pt.x = rand() % 100;
            pt.y = rand() % 100;
            pt.z = rand() % 100;
            cloud.push_back(pt);
        }

        // add the points to the grid
        grid.addPoints(cloud);

        // offset the points by small amounts and add them to the test cloud
        pcl::PointCloud <PointType> testCloud;
        for (auto pt : cloud)
        {
            pcl::PointXYZ pt2;
            pt2.x = pt.x + 0.01;
            pt2.y = pt.y + 0.01;
            pt2.z = pt.z + 0.01;
            testCloud.push_back(pt2);
        }

        // get neighbour point cloud
        pcl::PointCloud <PointType> neighbourCloud;
        neighbourCloud = grid.getNeighbourCloud(testCloud);

        // check if the neighbour cloud is the same as the original cloud
        for (int i = 0; i < cloud.size(); i++)
        {
            if (cloud[i].x != neighbourCloud[i].x || cloud[i].y != neighbourCloud[i].y || cloud[i].z != neighbourCloud[i].z)
            {
                std::cout << "Nearest neighbour search failed" << std::endl;
                return false;
            }
        }


        return true;
    }
    catch (...)
    {
        std::cout << "Nearest neighbour search failed" << std::endl;
        return false;
    }
}   
}   // namespace test

