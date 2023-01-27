/*
    Test 1 :     Test to add points to a grid and convert the grid to a point cloud
    Description: This test creates a grid, adds points to the grid, saves the grid, clears the grid, and loads the grid. It then converts the grid to a point cloud and prints the points in the cloud.
*/

#include <iostream>
#include "../include/VDBWrapper.hpp"


namespace test
{
   

// return false if test fails and true if test passes
bool test1(std::string path)
{
    try
    {
        // grid resolution 
        float res = 0.1;

        // create the grid
        using TreeType = openvdb::tree::Tree4<float, 5, 4, 3>::Type;
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

        // save the grid
        grid.saveGrid(path + "/test1.vdb");

        // reset the grid
        grid.resetGrid();

        // read the grid
        grid.readGrid(path + "/test1.vdb");

        // convert the grid to a point cloud
        pcl::PointCloud <pcl::PointXYZ> cloud2;

        grid.grid2cloud(cloud2);

        // optional: print the points in the cloud
        /*
        // print the points in the cloud
        for (auto pt : cloud2)
        {
            std::cout << pt.x << " " << pt.y << " " << pt.z << std::endl;
        }
        */

        return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}

} // namespace test
