/*
    Implementation of the VDBWrapper class.
*/

#include "VDBWrapper.hpp"

namespace vdb {

// instantiate - to separate tempated class from header file

// add more tree types here - 
using TreeType1 = openvdb::tree::Tree4<int, 3, 2, 1>::Type;
using TreeType2 = openvdb::tree::Tree4<float, 5, 4, 3>::Type;
using TreeType3 = openvdb::tree::Tree4<float, 5, 3, 2>::Type;

// instantiate the template class for all tree types & point types -
template class VDBWrapper<TreeType1, pcl::PointXYZ>;
template class VDBWrapper<TreeType2, pcl::PointXYZ>;
template class VDBWrapper<TreeType3, pcl::PointXYZ>;

// 

// Constructor and Destructor
template <typename TreeType, typename PointType>
VDBWrapper<TreeType, PointType>::VDBWrapper(const float& res)
: mResolution(res) 
{
    // initialize the grid
    openvdb::initialize(); 

    // initialize the grid
    mGrid->setTransform(openvdb::math::Transform::createLinearTransform(mResolution));
    mTransform = mGrid->transformPtr();

}

// reset the grid
template <typename TreeType, typename PointType>
void VDBWrapper<TreeType, PointType>::resetGrid()
{
    mGrid = GridType::create(0.0f);
    mGrid->setTransform(openvdb::math::Transform::createLinearTransform(mResolution));
    mTransform = mGrid->transformPtr();
    mAcc = mGrid->getAccessor();
}


// Data Read/Write

// add points to the grid
template <typename TreeType, typename PointType>
void VDBWrapper<TreeType, PointType>::addPoints( pcl::PointCloud < PointType > & cloud )
{
    // iterate through the cloud and add points to the grid
    for (auto pt : cloud){mAcc.setValueOn(pcl2vdbPt_(pt));}
}

// add points to the grid checked
template <typename TreeType, typename PointType>
void VDBWrapper<TreeType, PointType>::addPointsChecked( pcl::PointCloud < PointType > & cloud )
{
    // iterate through the cloud and add points to the grid
    for (auto pt : cloud)
    {
        if (!mAcc.isValueOn(pcl2vdbPt_(pt))) {mAcc.setValueOn(pcl2vdbPt_(pt));}
    }
}


// I/O functions

// save the grid to a file
template <typename TreeType, typename PointType>
void VDBWrapper<TreeType, PointType>::saveGrid(const std::string& filename)
{
    openvdb::io::File file(filename);
    openvdb::GridPtrVec grids;
    grids.push_back(mGrid);
    file.write(grids);
    file.close();
}

// read the grid from a file
template <typename TreeType, typename PointType>
void VDBWrapper<TreeType, PointType>::readGrid(const std::string& filename)
{
    openvdb::io::File file(filename);
    file.open();
    openvdb::GridPtrVecPtr grids = file.getGrids();
    for (auto grid : *grids)
    {
        mGrid = openvdb::gridPtrCast<GridType>(grid);
        mAcc = mGrid->getAccessor();
    }
    file.close();
}



// utility functions

// utility function to do point conversion from pcl to openvdb
template <typename TreeType, typename PointType>
openvdb::Coord VDBWrapper<TreeType, PointType>::pcl2vdbPt_(const PointType& pt)
{
    return openvdb::Coord::round(mTransform->worldToIndex(openvdb::Vec3d(pt.x, pt.y, pt.z)));
}

// utility function to do point conversion from openvdb to pcl
template <typename TreeType, typename PointType>
PointType VDBWrapper<TreeType, PointType>::vdb2pclPt_(const openvdb::Coord& pt)
{
    PointType pclPt;
    openvdb::Vec3d vdbPt = mTransform->indexToWorld(pt);
    pclPt.x = vdbPt[0];
    pclPt.y = vdbPt[1];
    pclPt.z = vdbPt[2];
    return pclPt;
}

// PCL functions

// convert the grid to a point cloud
template <typename TreeType, typename PointType>
void VDBWrapper<TreeType, PointType>::grid2cloud(pcl::PointCloud < PointType > & cloud)
{
    // iterate through the grid and add points to the cloud
    for (auto it = mGrid->beginValueOn(); it; ++it)
    {
        cloud.push_back(vdb2pclPt_(it.getCoord()));
    }
}

// filter functions

// filter the the free space   
template <typename TreeType, typename PointType>
pcl::PointCloud<PointType> VDBWrapper<TreeType, PointType>::filterFreeSpace(const pcl::PointCloud <PointType>& points, double checkRadius)
{
    // cloud to store the free points
    pcl::PointCloud < PointType > freePoints;

    //  create a functor to check for active values
    openvdb::tools::FindActiveValues<TreeType> obstacles{mGrid->constTree()};

    for (auto& pt : points) {
        // convert from world to index space
        auto maxBBox = mGrid->worldToIndex(openvdb::Vec3d(pt.x + checkRadius, pt.y + checkRadius, pt.z + checkRadius));
        auto minBBox = mGrid->worldToIndex(openvdb::Vec3d(pt.x - checkRadius, pt.y - checkRadius, pt.z - checkRadius));
        openvdb::CoordBBox bbox(openvdb::Coord(minBBox[0], minBBox[1], minBBox[2]),
                                openvdb::Coord(maxBBox[0], maxBBox[1], maxBBox[2]));
        // check if there are any active values in the bounding box
        if(!obstacles.anyActiveValues(bbox)) {freePoints.push_back(pt);}
    }
    return freePoints;
}


// return neighbors of points in the point cloud
template <typename TreeType, typename PointType>
pcl::PointCloud<PointType> VDBWrapper<TreeType, PointType>::getNeighbourCloud(const pcl::PointCloud <PointType>& points)
{
    // initialize the Neighbors Search object from utils/Neighbors.hpp
    vdb::NeighbourSearch<TreeType> ns_{mTransform};

    // cloud to store the neighbors
    pcl::PointCloud < PointType > neighbors;

    // iterate through the points and get the nearest neighbor
    for (auto& pt : points) {
        // get the neighbors
        openvdb::Vec3d neighbour = ns_.getNNeighbour(mAcc, openvdb::Vec3d(pt.x, pt.y, pt.z));
        // add the neighbor to the cloud
        neighbors.push_back(PointType(neighbour[0], neighbour[1], neighbour[2]));

    }
    return neighbors;
}

} // namespace vdb