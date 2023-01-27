// Wrapper File for OpenVDB Functions
// Author: Nishendra Singh

# pragma once

// genereal libraries
#include <execution>

// OpenVDB Libraries
# include <openvdb/openvdb.h>
# include <openvdb/math/Transform.h>
# include <openvdb/tools/FindActiveValues.h>

// Custom OpenVDB Libraries
# include "utils/Neighbours.hpp"

// PCL Libraries
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace vdb {


// Template for tree type and point type
template <typename TreeType, typename PointType>
class VDBWrapper {

    private:
        // set aliases
        using GridType = openvdb::Grid <TreeType> ;
        using GridPtr = typename GridType :: Ptr;

        // grid parameters
        float mResolution;

        // member variables
        GridPtr mGrid{GridType::create()};
        typename GridType::Accessor mAcc{mGrid->getAccessor()};
        openvdb::math::Transform::Ptr mTransform{mGrid->transformPtr()};


    protected:
        // utility functions
        openvdb::Coord pcl2vdbPt_(const PointType& pt); // PCL -> VDB
        PointType vdb2pclPt_(const openvdb::Coord& pt); // VDB -> PCL

    public:
        
        VDBWrapper(const float& res); // constructor
        void resetGrid();             // reset the grid
        

        // Data Read/Write

        void addPoints( pcl::PointCloud < PointType > & cloud );                       // add points to the grid
        void addPointsChecked( pcl::PointCloud < PointType > & cloud );                // add points to the grid checked
        bool queryGrid(const PointType& pt)  {return mAcc.isValueOn(pcl2vdbPt_(pt));}  // query the grid for a point
        

        // I/O functions 
        
        void saveGrid(const std::string& filename); // save the grid to a file
        void readGrid(const std::string& filename); // read the grid from a file


        // PCL functions
        void grid2cloud(pcl::PointCloud < PointType > & cloud); // grid to point cloud

        // low level functions
        typename GridType::Accessor getAccessor() { return mAcc; } // return the grid accessor
        GridPtr getGrid()                         { return mGrid;} // return the grid pointer
        
        // Fiter functions
        pcl::PointCloud < PointType > filterFreeSpace(const pcl::PointCloud <PointType>& points, double checkRadius); // get free points from a point cloud

        // Neighbour functions
        pcl::PointCloud < PointType > getNeighbourCloud(const pcl::PointCloud <PointType>& points); // get neighbours of a point
        


};

} // namespace vdb

