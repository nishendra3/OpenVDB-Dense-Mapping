/*
    Test 2     : Test the VDBWrapper class with a different tree type
    Description: With a different tree config, test to add points to a grid and check if they are in the grid.
*/

#include <iostream>

// include the test functions
#include "../include/VDBWrapper.hpp"

namespace test
{
    bool test2(std::string path)
    {
        try
        {
            // grid resolution 
            float res = 0.1;

            // create the grid
            using TreeType = openvdb::tree::Tree4<int, 3, 2, 1>::Type;
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

            // add the points to the grid using check for duplicates
            grid.addPointsChecked(cloud);

            // for each point in the cloud, check if it is in the grid
            for (auto pt : cloud)
            {
                if (!grid.queryGrid(pt))
                {
                    std::cout << "Some of the added points not detected in grid" << std::endl;
                    return false;
                }
            }



            return true;
        }
        catch (...)
        {
            return false;
        }


    }
} // namespace test