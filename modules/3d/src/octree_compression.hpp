// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.
//
// Copyright (C) 2021, Huawei Technologies Co., Ltd. All rights reserved.
// Third party copyrights are property of their respective owners.
//
// Author: Zihao Mu <zihaomu6@gmail.com>
//         Liangqian Kong <chargerKong@126.com>
//         Longbu Wang <riskiest@gmail.com>

#ifndef OPENCV_3D_SRC_OCTREE_COMPRESS_HPP
#define OPENCV_3D_SRC_OCTREE_COMPRESS_HPP

#include <vector>
#include <queue>
#include "opencv2/core.hpp"

namespace cv {

/** @brief OctreeCompressNode for OctreeCompress.

The class OctreeCompressNode represents the node of the octree. Each node contains 8 children, which are used to divide the
space cube into eight parts. Each octree node represents a cube.
And these eight children will have a fixed order, the order is described as follows:

For illustration, assume,
    rootNode: origin == (0, 0, 0), size == 2
 Then,
    children[0]: origin == (0, 0, 0), size == 1
    children[1]: origin == (1, 0, 0), size == 1, along X-axis next to child 0
    children[2]: origin == (0, 1, 0), size == 1, along Y-axis next to child 0
    children[3]: origin == (1, 1, 0), size == 1, in X-Y plane
    children[4]: origin == (0, 0, 1), size == 1, along Z-axis next to child 0
    children[5]: origin == (1, 0, 1), size == 1, in X-Z plane
    children[6]: origin == (0, 1, 1), size == 1, in Y-Z plane
    children[7]: origin == (1, 1, 1), size == 1, furthest from child 0

There are two kinds of nodes in an octree, intermediate nodes and leaf nodes, which are distinguished by isLeaf.
Intermediate nodes are used to contain leaf nodes, and leaf nodes will contain pointers to all pointcloud data
within the node, which will be used for octree indexing and mapping from point clouds to octree. Note that,
in an octree, each leaf node contains at least one point cloud data. Similarly, every intermediate OctreeNode
contains at least one non-empty child pointer, except for the root node.
*/
    class OctreeCompressNode{
    public:

        /**
        * There are multiple constructors to create OctreeCompressNode.
        * */
        OctreeCompressNode();

        /** @overload
        *
        * @param _depth The depth of the current node. The depth of the root node is 0, and the leaf node is equal
        * to the depth of Octree.
        * @param _size The length of the OctreeNode. In space, every OctreeNode represents a cube.
        * @param _origin The absolute coordinates of the center of the cube.
        * @param _parentIndex The serial number of the child of the current node in the parent node,
        * @param _pointNum The number of points belonging to the volume attached to the node,
        * the range is (-1~7). Among them, only the root node's _parentIndex is -1.
        */
        OctreeCompressNode(int _depth, double _size, const Point3f& _origin, int _parentIndex, int _pointNum);

        bool isPointInBound(const Point3f& _point, const Point3f& _origin, double _size) const;

        bool isPointInBound(const Point3f& _point) const;

        //! Contains 8 pointers to its 8 children.
        std::vector< Ptr<OctreeCompressNode> > children;

        //! Point to the parent node of the current node. The root node has no parent node and the value is NULL.
        Ptr<OctreeCompressNode> parent = nullptr;

        //! The depth of the current node. The depth of the root node is 0, and the leaf node is equal to the depth of Octree.
        int depth;

        //! The length of the OctreeCompressNode. In space, every OctreeCompressNode represents a cube.
        double size;

        //! Absolute coordinates of the smallest point of the cube.
        //! And the center of cube is `center = origin + Point3f(size/2, size/2, size/2)`.
        Point3f origin;

        /**  The serial number of the child of the current node in the parent node,
        * the range is (-1~7). Among them, only the root node's _parentIndex is -1.
        */
        int parentIndex;

        /**  The number of points belonging to the volume attached to the node.
        * only used for encoding octree. In restored octree, pointNum = -1
        */
        int pointNum;

        /**  The list of 6 adjacent neighbor node.
        *    index mapping:
        *     +z                        [101]
        *      |                          |    [110]
        *      |                          |  /
        *      O-------- +x    [001]----{000} ----[011]
        *     /                       /   |
        *    /                   [010]    |
        *  +y                           [100]
        *  index 000, 111 are reserved
        */
        std::vector< Ptr<OctreeCompressNode> > neigh;

        //! If the OctreeCompressNode is LeafNode.
        bool isLeaf = false;
    };

    class OctreeCompressKey{
    public:
        size_t x_key;
        size_t y_key;
        size_t z_key;

    public:
        OctreeCompressKey():x_key(0),y_key(0),z_key(0){};
        OctreeCompressKey(size_t x,size_t y,size_t z):x_key(x),y_key(y),z_key(z){};
        inline unsigned char findChildIdxByMask(size_t mask) const{
            return static_cast<unsigned char>((!!(z_key&mask))<<2)|((!!(y_key&mask))<<1)|(!!(x_key&mask));
        }

        static inline unsigned char getBitPattern(OctreeCompressNode &node) {
            unsigned char res=0;
            for (unsigned char i=0; i<node.children.size();i++){
                res|=static_cast<unsigned char>((!node.children[i].empty()) << i);
            }
            return res;
        }
    };

}
#endif //OPENCV_3D_SRC_OCTREE_COMPRESS_HPP