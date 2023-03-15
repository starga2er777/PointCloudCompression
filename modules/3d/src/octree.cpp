// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html

#include "precomp.hpp"
#include "octree.hpp"
#include "octree_key.h"

#define OCTREE_CHILD_NUM 8

namespace cv {

// Locate the OctreeNode corresponding to the input point from the given OctreeNode.
    static Ptr<OctreeNode> index(const Point3f &point, Ptr<OctreeNode> &node);

    static bool _isPointInBound(const Point3f &_point, const Point3f &_origin, double _size);

    static bool insertPointRecurse(Ptr<OctreeNode> &node, const Point3f &point, int maxDepth, const OctreeKey &key,
                                   size_t depthMask);

    bool deletePointRecurse(Ptr<OctreeNode> &node);

    void traverseByLevel(std::vector<unsigned char> &bit_out, OctreeNode &node);

    void restoreOctree(const std::vector<unsigned char> &bit_out, OctreeNode &root);

    void getPointRecurse(std::vector<Point3f> &restorePointCloud, unsigned long x_key, unsigned long y_key,
                         unsigned long z_key, Ptr<OctreeNode> &_node, double resolution, Point3f ori);


// For Nearest neighbor search.
    template<typename T>
    struct PQueueElem; // Priority queue
    static void radiusNNSearchRecurse(const Ptr<OctreeNode> &node, const Point3f &query, float squareRadius,
                                      std::vector<PQueueElem<Point3f> > &candidatePoint);

    static void KNNSearchRecurse(const Ptr<OctreeNode> &node, const Point3f &query, const int K,
                                 float &smallestDist, std::vector<PQueueElem<Point3f> > &candidatePoint);

    OctreeNode::OctreeNode() : children(OCTREE_CHILD_NUM, nullptr), depth(0), size(0), origin(0, 0, 0),
                               parentIndex(-1) {
    }

    OctreeNode::OctreeNode(int _depth, double _size, const Point3f &_origin, int _parentIndex) : children(
            OCTREE_CHILD_NUM), depth(_depth), size(_size), origin(_origin), parentIndex(_parentIndex) {
    }

    bool OctreeNode::empty() const {
        if (this->isLeaf) {
            if (this->pointList.empty())
                return true;
            else
                return false;
        } else {
            for (size_t i = 0; i < OCTREE_CHILD_NUM; i++) {
                if (!this->children[i].empty()) {
                    return false;
                }
            }
            return true;
        }
    }

    bool OctreeNode::isPointInBound(const Point3f &_point) const {
        return isPointInBound(_point, origin, size);
    }

    bool OctreeNode::isPointInBound(const Point3f &_point, const Point3f &_origin, double _size) const {
        return _isPointInBound(_point, _origin, _size);
    }

    bool _isPointInBound(const Point3f &_point, const Point3f &_origin, double _size) {
        float epsX = std::numeric_limits<float>::epsilon() * std::max(std::abs(_point.x), std::abs(_origin.x));
        float epsY = std::numeric_limits<float>::epsilon() * std::max(std::abs(_point.y), std::abs(_origin.y));
        float epsZ = std::numeric_limits<float>::epsilon() * std::max(std::abs(_point.z), std::abs(_origin.z));

        if ((_point.x + epsX >= _origin.x && _point.y + epsY >= _origin.y && _point.z + epsZ >= _origin.z) &&
            (_point.x <= _origin.x + _size + epsX && _point.y <= _origin.y + _size + epsY &&
             _point.z <= _origin.z + _size + epsZ)) {
            return true;
        } else {
            return false;
        }
    }

    struct Octree::Impl {
    public:
        Impl() : maxDepth(-1), size(0), origin(0, 0, 0), resolution(0), depthMask(0) {}

        ~Impl() {}

        // The pointer to Octree root node.
        Ptr<OctreeNode> rootNode = nullptr;
        //! Max depth of the Octree. And depth must be greater than zero
        int maxDepth;
        //! The size of the cube of the .
        double size;
        //! The origin coordinate of root node.
        Point3f origin;
        //!
        double resolution;
        //!
        size_t depthMask;
    };

    Octree::Octree() : p(new Impl) {
        p->maxDepth = -1;
        p->size = 0;
        p->origin = Point3f(0, 0, 0);
    }

    Octree::Octree(int _maxDepth, double _size, const Point3f &_origin) : p(new Impl) {
        p->maxDepth = _maxDepth;
        p->size = _size;
        p->origin = _origin;
    }

    Octree::Octree(const std::vector<Point3f> &_pointCloud, int _maxDepth) : p(new Impl) {
        CV_Assert(_maxDepth > -1);
        this->create(_pointCloud, _maxDepth);
    }

    Octree::Octree(int _maxDepth) : p(new Impl) {
        p->maxDepth = _maxDepth;
        p->size = 0;
        p->origin = Point3f(0, 0, 0);
    }

    Octree::~Octree() {}

    bool Octree::insertPoint(const Point3f &point, double resolution, size_t depthMask) {
        if (p->rootNode.empty()) {
            p->rootNode = new OctreeNode(0, p->size, p->origin, -1);
        }

        OctreeKey key(floor((point.x - this->p->origin.x) / resolution),
                      floor((point.y - this->p->origin.y) / resolution),
                      floor((point.z - this->p->origin.z) / resolution));
        //std::cout<<"x="<<key.x_key<<" y="<<key.y_key<<" z="<<key.z_key<<std::endl;
        return insertPointRecurse(p->rootNode, point, p->maxDepth, key, depthMask);
    }


    bool Octree::create(const std::vector<Point3f> &pointCloud, double resolution) {
        if (resolution > 0) {
            p->resolution = resolution;
        }

        //CV_Assert( p->maxDepth > -1 && !pointCloud.empty());

        if (pointCloud.empty())
            return false;

        Point3f maxBound(pointCloud[0]);
        Point3f minBound(pointCloud[0]);
        Point3f center, temp;

        // Find center coordinate of PointCloud data.
        for (size_t idx = 0; idx < pointCloud.size(); idx++) {
            maxBound.x = max(pointCloud[idx].x, maxBound.x);
            maxBound.y = max(pointCloud[idx].y, maxBound.y);
            maxBound.z = max(pointCloud[idx].z, maxBound.z);

            minBound.x = min(pointCloud[idx].x, minBound.x);
            minBound.y = min(pointCloud[idx].y, minBound.y);
            minBound.z = min(pointCloud[idx].z, minBound.z);
        }

        double maxSize = max(max(maxBound.x - minBound.x, maxBound.y - minBound.y), maxBound.z - minBound.z);
        p->maxDepth = ceil(log2(maxSize / resolution));
        p->depthMask = 1 << (p->maxDepth - 1);


        center = (maxBound + minBound) * 0.5f;

        temp = center - minBound;
        float halfSize = std::max(temp.x, std::max(temp.y, temp.z));
        this->p->origin = Point3f(float(floor(minBound.x / resolution) * resolution),
                                  float(floor(minBound.y / resolution) * resolution),
                                  float(floor(minBound.z / resolution) * resolution));

        this->p->size = 2 * halfSize;

        // Insert every point in PointCloud data.
        int cnt = 0;
        for (size_t idx = 0; idx < pointCloud.size(); idx++) {
            if (!insertPoint(pointCloud[idx], resolution, p->depthMask)) {
                cnt++;
            }
        }

        CV_LOG_IF_WARNING(NULL, cnt != 0, "OverAll " << cnt
                                                     << " points has been ignored! The number of point clouds contained in the current octree is "
                                                     << pointCloud.size() - cnt);
        return true;
    }

    void Octree::traverse(std::vector<unsigned char> &bit_out) {
        traverseByLevel(bit_out, *p->rootNode);
        size_t length = bit_out.size();
        unsigned char len[8];
        memcpy(len, &length, sizeof(len));

        unsigned char ori[12];
        memcpy(ori, &p->origin.x, sizeof(ori) / 3);
        memcpy(&ori[4], &p->origin.y, sizeof(ori) / 3);
        memcpy(&ori[8], &p->origin.z, sizeof(ori) / 3);

        bit_out.push_back((unsigned char) log10(p->resolution));
        for (unsigned char &i: ori) {
            bit_out.push_back(i);
        }
        for (unsigned char &i: len) {
            bit_out.push_back(i);
        }
    }

    void Octree::reStore(const std::vector<unsigned char> &bit_out) {

        this->clear();
        size_t bit_out_len = bit_out.size();
        size_t len;
        char resolution_c;
        Point3f ori(0, 0, 0);
        memcpy(&len, &bit_out[bit_out_len - 8], sizeof(len));
        memcpy(&resolution_c, &bit_out[len], sizeof(char));
        memcpy(&ori.x, &bit_out[len + 1], sizeof(float));
        memcpy(&ori.y, &bit_out[len + 1 + 4], sizeof(float));
        memcpy(&ori.z, &bit_out[len + 1 + 8], sizeof(float));
        p->rootNode = new OctreeNode(0, p->size, Point3f(0, 0, 0), -1);
        p->origin = ori;
        p->resolution = pow(10, int(char(resolution_c)));
        restoreOctree(bit_out, *p->rootNode);
    }

    void Octree::getPointCloudByOctree(std::vector<Point3f> &restorePointCloud) {
        Ptr<OctreeNode> root = p->rootNode;
        double resolution = p->resolution;

        getPointRecurse(restorePointCloud, 0, 0, 0, root, resolution, p->origin);
    }

    void Octree::setMaxDepth(int _maxDepth) {
        if (_maxDepth)
            this->p->maxDepth = _maxDepth;
    }

    void Octree::setSize(double _size) {
        this->p->size = _size;
    };

    void Octree::setOrigin(const Point3f &_origin) {
        this->p->origin = _origin;
    }

    void Octree::clear() {
        if (!p->rootNode.empty()) {
            p->rootNode.release();
        }

        p->size = 0;
        p->maxDepth = -1;
        p->origin = Point3f(0, 0, 0); // origin coordinate
        p->resolution = 0;
        p->depthMask = 0;
    }

    bool Octree::empty() const {
        return p->rootNode.empty();
    }

    Ptr<OctreeNode> index(const Point3f &point, Ptr<OctreeNode> &_node) {
        OctreeNode &node = *_node;

        if (node.empty()) {
            return Ptr<OctreeNode>();
        }

        if (node.isLeaf) {
            for (size_t i = 0; i < node.pointList.size(); i++) {
                if ((point.x == node.pointList[i].x) &&
                    (point.y == node.pointList[i].y) &&
                    (point.z == node.pointList[i].z)
                        ) {
                    return _node;
                }
            }
            return Ptr<OctreeNode>();
        }

        if (node.isPointInBound(point)) {
            double childSize = node.size * 0.5;

            float epsX = std::numeric_limits<float>::epsilon() * std::max(std::abs(point.x), std::abs(node.origin.x));
            float epsY = std::numeric_limits<float>::epsilon() * std::max(std::abs(point.y), std::abs(node.origin.y));
            float epsZ = std::numeric_limits<float>::epsilon() * std::max(std::abs(point.z), std::abs(node.origin.z));

            size_t xIndex = point.x <= node.origin.x + float(childSize) + epsX ? 0 : 1;
            size_t yIndex = point.y <= node.origin.y + float(childSize) + epsY ? 0 : 1;
            size_t zIndex = point.z <= node.origin.z + float(childSize) + epsZ ? 0 : 1;
            size_t childIndex = xIndex + yIndex * 2 + zIndex * 4;

            if (!node.children[childIndex].empty()) {
                return index(point, node.children[childIndex]);
            }
        }
        return Ptr<OctreeNode>();
    }

    bool Octree::isPointInBound(const Point3f &_point) const {
        return _isPointInBound(_point, p->origin, p->size);
    }

    bool Octree::deletePoint(const Point3f &point) {
        Ptr<OctreeNode> node = index(point, p->rootNode);

        if (!node.empty()) {
            size_t i = 0;
            while (!node->pointList.empty() && i < node->pointList.size()) {
                if ((point.x == node->pointList[i].x) &&
                    (point.y == node->pointList[i].y) &&
                    (point.z == node->pointList[i].z)
                        ) {
                    node->pointList.erase(node->pointList.begin() + i);
                } else {
                    i++;
                }
            }

            // If it is the last point cloud in the OctreeNode, recursively delete the node.
            return deletePointRecurse(node);
        } else {
            return false;
        }
    }

    bool deletePointRecurse(Ptr<OctreeNode> &_node) {
        OctreeNode &node = *_node;

        if (_node.empty())
            return false;

        if (node.isLeaf) {
            if (!node.pointList.empty()) {
                Ptr<OctreeNode> parent = node.parent;
                parent->children[node.parentIndex] = nullptr;
                _node.release();

                return deletePointRecurse(parent);
            } else {
                return true;
            }
        } else {
            bool deleteFlag = true;

            // Only all children was deleted, can we delete the tree node.
            for (size_t i = 0; i < OCTREE_CHILD_NUM; i++) {
                if (!node.children[i].empty()) {
                    deleteFlag = false;
                    break;
                }
            }

            if (deleteFlag) {
                Ptr<OctreeNode> parent = node.parent;
                _node.release();
                return deletePointRecurse(parent);
            } else {
                return true;
            }
        }
    }

    bool insertPointRecurse(Ptr<OctreeNode> &_node, const Point3f &point, int maxDepth, const OctreeKey &key,
                            size_t depthMask) {
        OctreeNode &node = *_node;

        node.pointNum++;

        if (node.depth == maxDepth) {
            node.isLeaf = true;
            node.pointList.push_back(point);
            return true;
        }


        size_t childIndex = key.findChildIdxByMask(depthMask);
        //std::cout<<childIndex<<std::endl;
        if (node.children[childIndex].empty()) {
            node.children[childIndex] = new OctreeNode(node.depth + 1, 0, Point3f(0, 0, 0), int(childIndex));
            node.children[childIndex]->parent = _node;
        }
        return insertPointRecurse(node.children[childIndex], point, maxDepth, key, depthMask >> 1);
    }

    void traverseDCM(std::vector<unsigned char> &binary_tree_out_arg, OctreeNode &node) {
        if (node.isLeaf) {

        }
    }

    void traverseByLevel(std::vector<unsigned char> &binary_tree_out_arg, OctreeNode &node) {
        if (node.empty() || node.children.empty()) {
            return;
        }
        std::queue<OctreeNode *> nodeQueue;
        nodeQueue.push(&node);
        try {
            while (!nodeQueue.empty()) {
                OctreeNode *tmpNode = nodeQueue.front();

                // DCM eligibility
                bool eligible = false;
                for (unsigned char i=0; i<node.children.size();i++){
                    if (!node.children[i].empty())
                    {
                        if (eligible)
                        {
                            eligible = false;
                            break;
                        }
                        eligible = true;
                    }
                }
                if (eligible)
                {
                    if (node.pointNum <= 2)
                    {
                        // DCM applied

                    }
                    continue;
                }

                binary_tree_out_arg.push_back(OctreeKey::getBitPattern(*tmpNode));
                nodeQueue.pop();
                for (unsigned char i = 0; i < 8; i++) {
                    if (!tmpNode->children[i].empty()) {
                        nodeQueue.push(tmpNode->children[i]);
                    }
                }
            }
        }
        catch (std::bad_alloc) {

        }

    }

    void restoreOctree(const std::vector<unsigned char> &binary_tree_out_arg, OctreeNode &root) {
        std::queue<OctreeNode *> nodeQueue;
        nodeQueue.push(&root);
        size_t index = 0;
        try {
            while (!nodeQueue.empty()) {
                if (index == 1000000) {
                    std::cout << "";
                }
                OctreeNode *tmpNode = nodeQueue.front();
                nodeQueue.pop();
                unsigned char mask = 1;
                if (binary_tree_out_arg[index]) {
                    for (unsigned char i = 0; i < 8; i++) {
                        if (!!(binary_tree_out_arg[index] & mask)) {
                            tmpNode->children[i] = new OctreeNode(tmpNode->depth + 1, 0, Point3f(0, 0, 0), int(i));
                            tmpNode->children[i]->parent = tmpNode;
                            nodeQueue.push(tmpNode->children[i]);
                        }
                        mask = mask << 1;
                    }
                } else {
                    tmpNode->isLeaf = true;
                }
                index++;
            }
        }
        catch (std::bad_alloc) {

        }

    }

    void getPointRecurse(std::vector<Point3f> &restorePointCloud, unsigned long x_key, unsigned long y_key,
                         unsigned long z_key, Ptr<OctreeNode> &_node, double resolution, Point3f ori) {
        OctreeNode node = *_node;
        if (node.isLeaf) {
            restorePointCloud.emplace_back((float) (resolution * x_key) + ori.x, (float) (resolution * y_key) + ori.y,
                                           (float) (resolution * z_key) + ori.z);
            return;
        }
        unsigned char x_mask = 1;
        unsigned char y_mask = 2;
        unsigned char z_mask = 4;
        for (unsigned char i = 0; i < 8; i++) {
            unsigned long x_copy = x_key;
            unsigned long y_copy = y_key;
            unsigned long z_copy = z_key;
            if (!node.children[i].empty()) {
                if ((z_copy << 1) < z_copy || (y_copy << 1) < y_copy || (x_copy << 1) < x_copy) {
                    std::cout << "!!!";
                }
                size_t x_offSet = !!(x_mask & i);
                size_t y_offSet = !!(y_mask & i);
                size_t z_offSet = !!(z_mask & i);
                x_copy = (x_copy << 1) | x_offSet;
                y_copy = (y_copy << 1) | y_offSet;
                z_copy = (z_copy << 1) | z_offSet;
                getPointRecurse(restorePointCloud, x_copy, y_copy, z_copy, node.children[i], resolution, ori);
            }
        }
    };


// For Nearest neighbor search.
    template<typename T>
    struct PQueueElem {
        PQueueElem() : dist(0), t(0) {}

        PQueueElem(float _dist, T _t) : dist(_dist), t(_t) {}

        float dist;
        T t;

        bool
        operator<(const PQueueElem<T> p1) const {
            return (this->dist < p1.dist);
        }
    };

    static float SquaredDistance(const Point3f &query, const Point3f &origin) {
        Point3f diff = query - origin;
        return diff.dot(diff);
    }

    static bool overlap(const OctreeNode &node, const Point3f &query, float squareRadius) {
        float halfSize = float(node.size * 0.5);
        Point3f center = node.origin + Point3f(halfSize, halfSize, halfSize);

        float dist = SquaredDistance(center, query);
        float temp = float(node.size) * float(node.size) * 3.0f;

        return (dist + dist * std::numeric_limits<float>::epsilon()) <=
               float(temp * 0.25f + squareRadius + sqrt(temp * squareRadius));
    }

    void radiusNNSearchRecurse(const Ptr<OctreeNode> &node, const Point3f &query, float squareRadius,
                               std::vector<PQueueElem<Point3f> > &candidatePoint) {
        float dist;
        Ptr<OctreeNode> child;

        // iterate eight children.
        for (size_t i = 0; i < OCTREE_CHILD_NUM; i++) {
            if (!node->children[i].empty() && overlap(*node->children[i], query, squareRadius)) {
                if (!node->children[i]->isLeaf) {
                    // Reach the branch node.
                    radiusNNSearchRecurse(node->children[i], query, squareRadius, candidatePoint);
                } else {
                    // Reach the leaf node.
                    child = node->children[i];

                    for (size_t j = 0; j < child->pointList.size(); j++) {
                        dist = SquaredDistance(child->pointList[j], query);
                        if (dist + dist * std::numeric_limits<float>::epsilon() <= squareRadius) {
                            candidatePoint.emplace_back(dist, child->pointList[j]);
                        }
                    }
                }
            }
        }
    }

    int Octree::radiusNNSearch(const Point3f &query, float radius,
                               std::vector<Point3f> &pointSet, std::vector<float> &squareDistSet) const {
        if (p->rootNode.empty())
            return 0;
        float squareRadius = radius * radius;

        PQueueElem<Point3f> elem;
        std::vector<PQueueElem<Point3f> > candidatePoint;

        radiusNNSearchRecurse(p->rootNode, query, squareRadius, candidatePoint);

        for (size_t i = 0; i < candidatePoint.size(); i++) {
            pointSet.push_back(candidatePoint[i].t);
            squareDistSet.push_back(candidatePoint[i].dist);
        }
        return int(pointSet.size());
    }

    void KNNSearchRecurse(const Ptr<OctreeNode> &node, const Point3f &query, const int K,
                          float &smallestDist, std::vector<PQueueElem<Point3f> > &candidatePoint) {
        std::vector<PQueueElem<int> > priorityQue;
        Ptr<OctreeNode> child;
        float dist = 0;
        Point3f center; // the OctreeNode Center

        // Add the non-empty OctreeNode to priorityQue.
        for (size_t i = 0; i < OCTREE_CHILD_NUM; i++) {
            if (!node->children[i].empty()) {
                float halfSize = float(node->children[i]->size * 0.5);

                center = node->children[i]->origin + Point3f(halfSize, halfSize, halfSize);

                dist = SquaredDistance(query, center);
                priorityQue.emplace_back(dist, int(i));
            }
        }

        std::sort(priorityQue.rbegin(), priorityQue.rend());
        child = node->children[priorityQue.back().t];

        while (!priorityQue.empty() && overlap(*child, query, smallestDist)) {
            if (!child->isLeaf) {
                KNNSearchRecurse(child, query, K, smallestDist, candidatePoint);
            } else {
                for (size_t i = 0; i < child->pointList.size(); i++) {
                    dist = SquaredDistance(child->pointList[i], query);

                    if (dist + dist * std::numeric_limits<float>::epsilon() <= smallestDist) {
                        candidatePoint.emplace_back(dist, child->pointList[i]);
                    }
                }

                std::sort(candidatePoint.begin(), candidatePoint.end());

                if (int(candidatePoint.size()) > K) {
                    candidatePoint.resize(K);
                }

                if (int(candidatePoint.size()) == K) {
                    smallestDist = candidatePoint.back().dist;
                }
            }

            priorityQue.pop_back();

            // To next child.
            if (!priorityQue.empty())
                child = node->children[priorityQue.back().t];
        }
    }

    void Octree::KNNSearch(const Point3f &query, const int K, std::vector<Point3f> &pointSet,
                           std::vector<float> &squareDistSet) const {
        if (p->rootNode.empty())
            return;

        PQueueElem<Ptr<Point3f> > elem;
        std::vector<PQueueElem<Point3f> > candidatePoint;
        float smallestDist = std::numeric_limits<float>::max();

        KNNSearchRecurse(p->rootNode, query, K, smallestDist, candidatePoint);

        for (size_t i = 0; i < candidatePoint.size(); i++) {
            pointSet.push_back(candidatePoint[i].t);
            squareDistSet.push_back(candidatePoint[i].dist);
        }
    }

// Range Coding methods based on implementation by Julius Kammerl (julius@kammerl.de) from PCL
    void Octree::encodeCharVectorToStream(const std::vector<unsigned char> &inputCharVector_arg,
                                          std::ostream &outputByteStream_arg) {

        // histogram of char frequency
        std::uint64_t hist[257];

        // partition of symbol ranges from cumulative frequency, define by left index
        std::uint32_t part_idx[257];

        // range limits
        const std::uint32_t adjust_limit = static_cast<std::uint32_t> (1) << 24;
        const std::uint32_t bottom_limit = static_cast<std::uint32_t> (1) << 16;

        // encoding variables
        std::uint32_t low, range;
        size_t readPos;
        std::uint8_t symbol;

        size_t input_size = static_cast<size_t> (inputCharVector_arg.size());

        // output vector
        std::vector<unsigned char> outputCharVector_;
        outputCharVector_.clear();
        outputCharVector_.reserve(sizeof(unsigned char) * input_size);

        // calculate index
        memset(hist, 0, sizeof(hist));
        readPos = 0;
        while (readPos < input_size) {
            symbol = static_cast<std::uint8_t> (inputCharVector_arg[readPos++]);
            hist[symbol + 1]++;
        }
        part_idx[0] = 0;
        for (int i = 1; i <= 256; i++) {
            // partition must have at least 1 space for each symbol
            if (hist[i] <= 0) {
                part_idx[i] = part_idx[i - 1] + 1;
                continue;
            }
            part_idx[i] = part_idx[i - 1] + static_cast<std::uint32_t> (hist[i]);
        }

        // rescale if least partition range exceeds bottom_limit
        while (part_idx[256] >= bottom_limit) {
            for (int i = 1; i <= 256; i++) {
                part_idx[i] >>= 1;
                if (part_idx[i] <= part_idx[i - 1]) {
                    part_idx[i] = part_idx[i - 1] + 1;
                }
            }
        }

        // write cumulative frequency table to output stream
        outputByteStream_arg.write(reinterpret_cast<const char *> (&part_idx[0]), sizeof(part_idx));

        // start encoding, range initialize to maximum
        readPos = 0;
        low = 0;
        range = static_cast<std::uint32_t> (-1);

        while (readPos < input_size) {
            // read symol
            symbol = static_cast<std::uint8_t>(inputCharVector_arg[readPos++]);

            // map to range
            low += part_idx[symbol] * (range /= part_idx[256]);
            range *= part_idx[symbol + 1] - part_idx[symbol];

            // renormalization
            // first case: range is completely inside a block of size adjust_limit
            // second case: range is too small while first case continuously miss, preform resize to bottom_limit
            while ((low ^ (low + range)) < adjust_limit ||
                   ((range < bottom_limit) && ((range = -int(low) & (bottom_limit - 1)), 1))) {
                unsigned char out = static_cast<unsigned char> (low >> 24);
                range <<= 8;
                low <<= 8;
                outputCharVector_.push_back(out);
            }

        }

        // flush remaining data
        for (int i = 0; i < 4; i++) {
            unsigned char out = static_cast<unsigned char> (low >> 24);
            outputCharVector_.push_back(out);
            low <<= 8;
        }

//        size_t length = outputCharVector_.size();
//        unsigned char len[8];
//        memcpy(len,&length,sizeof(len));
//
//        unsigned char ori[12];
//        memcpy(ori,&p->origin.x,sizeof(ori)/3);
//        memcpy(&ori[4],&p->origin.y,sizeof(ori)/3);
//        memcpy(&ori[8],&p->origin.z,sizeof(ori)/3);
//
//        outputCharVector_.push_back((unsigned char)log10(p->resolution));
//        for(unsigned char & i : ori){
//            outputCharVector_.push_back(i);
//        }
//        for(unsigned char & i : len){
//            outputCharVector_.push_back(i);
//        }

        // write encoded data to stream
        outputByteStream_arg.write(reinterpret_cast<const char *> (&outputCharVector_[0]), outputCharVector_.size());

    }

    void Octree::decodeStreamToCharVector(std::istream &inputByteStream_arg,
                                          std::vector<unsigned char> &outputCharVector_arg) {

        // partition of symbol ranges from cumulative frequency, define by left index
        std::uint32_t part_idx[257];

        // range limits
        const std::uint32_t adjust_limit = static_cast<std::uint32_t> (1) << 24;
        const std::uint32_t bottom_limit = static_cast<std::uint32_t> (1) << 16;

        // decoding variables
        std::uint32_t low, range;
        std::uint32_t code;

        size_t outputPos;
        size_t output_size;

        output_size = static_cast<size_t> (outputCharVector_arg.size());

        outputPos = 0;

        // read cumulative frequency table
        inputByteStream_arg.read(reinterpret_cast<char *> (&part_idx[0]), sizeof(part_idx));

        // read code
        code = 0;
        for (size_t i = 0; i < 4; i++) {
            std::uint8_t out;
            inputByteStream_arg.read(reinterpret_cast<char *> (&out), sizeof(unsigned char));
            code = (code << 8) | out;
        }

        low = 0;
        range = static_cast<std::uint32_t> (-1);

        // decoding
        for (size_t i = 0; i < output_size; i++) {
            // symbol lookup in cumulative frequency table
            std::uint32_t count = (code - low) / (range /= part_idx[256]);

            // finding symbol using Jump search
            std::uint8_t symbol = 0;
            std::uint8_t step = 128;
            while (step > 0) {
                if (part_idx[symbol + step] <= count) {
                    symbol = static_cast<std::uint8_t> (symbol + step);
                }
                step /= 2;
            }

            // write symbol to output stream
            outputCharVector_arg[outputPos++] = symbol;

            low += part_idx[symbol] * range;
            range *= part_idx[symbol + 1] - part_idx[symbol];

            // check range limits
            while ((low ^ (low + range)) < adjust_limit ||
                   ((range < bottom_limit) && ((range = -int(low) & (bottom_limit - 1)), 1))) {
                std::uint8_t out;
                inputByteStream_arg.read(reinterpret_cast<char *> (&out), sizeof(unsigned char));
                code = code << 8 | out;
                range <<= 8;
                low <<= 8;
            }

        }

    }

}