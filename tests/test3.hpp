/*
    Test 3:      Test to filter a point cloud using a grid of obstacles.
    Description: Generates a random point cloud and a grid of obstacles then filters the point cloud using the grid of obstacles and prints the filtered point cloud.
*/

#include <iostream>
#include "../include/VDBWrapper.hpp"


namespace test
{
    bool test3(std::string path)
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
            pcl::PointCloud <PointType> obstacleCloud;
            
            // add 500 random points to the cloud
            for (int i = 0; i < 500; i++)
            {
                pcl::PointXYZ pt;
                pt.x = rand() % 100;
                pt.y = rand() % 100;
                pt.z = rand() % 100;
                obstacleCloud.push_back(pt);
            }

            // add the points to the grid using check for duplicates
            grid.addPointsChecked(obstacleCloud);

            // create a new point cloud with random points and points from obstacleCloud
            pcl::PointCloud <PointType> cloud;
            for (int i = 0; i < 500; i++)
            {
                pcl::PointXYZ pt;
                pt.x = rand() % 100;
                pt.y = rand() % 100;
                pt.z = rand() % 100;
                cloud.push_back(pt);
            }
            for (auto pt : obstacleCloud)
            {
                cloud.push_back(pt);
            }

            // filter the point cloud
            pcl::PointCloud <PointType> filteredCloud;
            
            filteredCloud = grid.filterFreeSpace(cloud, 0.0001); // 0.0001 is check radius

            // create a verification cloud 
            pcl::PointCloud <PointType> verificationCloud;
            for (auto pt : cloud)
            {
                if (!grid.queryGrid(pt))
                {
                    verificationCloud.push_back(pt);
                }
            }

            // check if the filtered cloud is the same as the verification cloud
            if (filteredCloud.size() != verificationCloud.size())
            {
                std::cout << "Filtered cloud size does not match verification cloud size" << std::endl;
                return false;
            }

            //  Optional: print the filtered cloud and the verification cloud
            /*
            // print the filtered cloud
            std::cout << "Filtered cloud:" << std::endl;
            for (auto pt : filteredCloud)
            {
                std::cout << pt.x << " " << pt.y << " " << pt.z << std::endl;
            }

            // print the verification cloud
            std::cout << "Verification cloud:" << std::endl;
            for (auto pt : verificationCloud)
            {
                std::cout << pt.x << " " << pt.y << " " << pt.z << std::endl;
            }
            */

            return true;
        }
        catch (...)
        {
            return false;
        }


    }
    
}   // namespace test
