// Header file for NeighbourSearch class
// Author: Nishendra Singh

#pragma once

#include <openvdb/openvdb.h>

#include <iostream>
#include <queue>
#include <execution>

namespace vdb
{
    template <typename vdbTree>
    class NeighbourSearch
    {
        private:
            

            // internal nodes
            using RootType = typename vdbTree::RootNodeType;   // level 3 RootNode
            using Int1Type = typename RootType::ChildNodeType;  // level 2 InternalNode
            using Int2Type = typename Int1Type::ChildNodeType;  // level 1 InternalNode
            using LeafType = typename vdbTree::LeafNodeType;   // level 0 LeafNode

            //
            using VDBGrid = typename openvdb::Grid<vdbTree>;
            using ptType  = openvdb::Vec3d;

            // pair typedef
            typedef std::pair<float, uint32_t> pi;
            typedef std::pair<float, ptType> pi2;

            //
            // VDBGrid::Accessor mAccessor;
            openvdb::math::Transform mTransform;


        public:
            NeighbourSearch(openvdb::math::Transform::Ptr transform_) 
            : mTransform(*transform_) {};
            openvdb::Coord getOffset(const openvdb::Coord &ijk, const uint32_t &idx)
            {
                uint32_t DIM = Int2Type::DIM / LeafType::DIM;
                
                auto dx = uint32_t(idx / (DIM*DIM)) ;
                auto dy = uint32_t((idx - dx*DIM*DIM ) / DIM);
                auto dz = uint32_t(idx - (dx * DIM * DIM) - (dy * DIM ));
                
                return ijk.offsetBy(dx * LeafType::DIM, dy * LeafType::DIM , dz * LeafType::DIM);
            };

            ptType getNNeighbourLeaf(const typename VDBGrid::Accessor& accessor_, const LeafType* leafNode, const ptType& pt_)
            {  
                
                                       
                // current min as INF
                auto currentMin = std::numeric_limits<float>::max();
                ptType closestPoint;

                for(auto iter = leafNode->cbeginValueOn(); iter; ++iter)
                {
                    // get distance to the point
                    auto candidatePt = mTransform.indexToWorld(iter.getCoord());
                    auto dist = (pt_ - candidatePt).length();
                    
                    // check if the distance is less than the current min
                    if (dist < currentMin)
                    {
                        currentMin = dist;
                        closestPoint = candidatePt;
                    }
                }

                // get bounding box of the leaf node .. 
                auto bbox = leafNode->getNodeBoundingBox();
                
                // get minimum and maximum of the coordinates
                auto minBound = bbox.getStart();
                auto maxBound = bbox.getEnd();

                // convert the point to index
                auto ptIdx = mTransform.worldToIndex(pt_);
                const int &i = ptIdx.x(), &j = ptIdx.y(), &k = ptIdx.z();


                // check if distance to closet point is more than distance to the faces of bounding box
                double dx = std::min(std::abs(minBound.x()-i), std::abs(maxBound.x()-i));
                double dy = std::min(std::abs(minBound.y()-j), std::abs(maxBound.y()-j));
                double dz = std::min(std::abs(minBound.z()-k), std::abs(maxBound.z()-k));
                
                // find the minimum distance of dx dy dz
                auto dist2boundary = std::min(std::min(dx, dy), dz);

                // check if the distance is less than the current min
                if (dist2boundary < currentMin)
                {
                    
                    auto center = bbox.getCenter();
                    // get 26 neighbours of the center
                    
                    auto size = LeafType::DIM;
                    // priority queue to store the closest points
                    std::priority_queue<pi2, std::vector<pi2>, std::greater<pi2>> pq;            
                    for (int i_{0}; i_<27; ++i_)
                    {
                        if (i_ == 13) continue;
                        dx = (i_%3)-1;
                        dy = ((i_/3)%3)-1;
                        dz = (i_/9)-1;

                        auto n = ptType(center.x() + dx*size, center.y() + dy*size, center.z() + dz*size);
                        auto dist = (ptIdx - n).length();
                        pq.emplace(dist, n);
                    }

                    
                    // iterate over the top 4 entries of the priority queue
                    for (int i{0}; i<4; ++i)
                    {
                        auto n = pq.top().second;
                        pq.pop();
                        // get the leaf node
                        auto leafNode2 = accessor_.probeConstLeaf(openvdb::Coord(n.x(), n.y(), n.z()));
                        if (leafNode2)
                        {
                            // iterate over the leaf node
                            for(auto iter = leafNode2->cbeginValueOn(); iter; ++iter)
                            {
                                // get distance to the point
                                auto candidatePt = mTransform.indexToWorld(iter.getCoord());
                                auto dist = (pt_ - candidatePt).length();
                                
                                // check if the distance is less than the current min
                                if (dist < currentMin)
                                {
                                    currentMin = dist;
                                    closestPoint = candidatePt;
                                }
                            }
                        }
                    }
                }
                

                return closestPoint;
                
            }

            // get the closest point in the parent node
            ptType getNNeighbourParent(const typename VDBGrid::Accessor& accessor_, const Int2Type* parentNode, const ptType& pt)
            {
                
                    const auto* table = parentNode->getTable();
                    auto mask         = parentNode->getChildMask();

                    // get origin of parent
                    auto parentOrigin = parentNode->origin();
                    
                    // priority queue to store the distance and index
                    std::priority_queue<pi, std::vector<pi>, std::greater<pi>> pq;

                    // get center of leaf node
                    for (auto i = mask.beginOn(); i; ++i)
                    {
                        auto childOrigin = this->getOffset(parentOrigin, i.pos());
                        auto childCenter = childOrigin.offsetBy(LeafType::DIM >> 1); // divide by 2
                        auto dist = (pt - mTransform.indexToWorld(childCenter)).length(); // get the distance
                        pq.emplace(dist, i.pos());  
                    }

                    // get the closest node and iterate over it
                    auto leafNode = table[pq.top().second].getChild();
                    return this->getNNeighbourLeaf(accessor_, leafNode, pt);
                
            }

            // get the closest point
            ptType getNNeighbour(const typename VDBGrid::Accessor& accessor_, const ptType &pt)
            {

                auto ijk        = this->mTransform.worldToIndexCellCentered(pt);
                auto leafNode   = accessor_.probeConstLeaf(ijk);
                if (leafNode)
                {
                    return this->getNNeighbourLeaf(accessor_, leafNode, pt);
                }
                else
                {   
                    // get the parent node
                    auto parentNode = accessor_.template probeConstNode<Int2Type>(ijk);
                    if (!parentNode)
                    {
                        // std::cout << "[ERROR] " ;
                        return ptType(0,0,0);
                    }
                    return this->getNNeighbourParent(accessor_, parentNode, pt);
                }

            }
    };
}; // namespace vdb
