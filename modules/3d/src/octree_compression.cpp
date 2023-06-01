// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html

#include "precomp.hpp"
#include "octree_compression.hpp"

#include <cmath>
#include <fstream>
#include <algorithm>

#define OCTREE_CHILD_NUM 8
#define OCTREE_NEIGH_SIZE 8

namespace cv {

    void Haar3DRecursive(OctreeCompressNode *node, std::vector<Point3f> &haarCoefficients,
                         std::vector<OctreeCompressNode *> &cubes, size_t &N);

    void invHaar3DRecursive(OctreeCompressNode *node, std::vector<Point3f> &haarCoefficients,
                            std::vector<OctreeCompressNode *> &cubes, size_t &N);

    static bool _isPointInBound(const Point3f &_point, const Point3f &_origin, double _size);

    static bool
    insertPointRecurse(Ptr<OctreeCompressNode> &node, const Point3f &point, const Point3f &color, int maxDepth,
                       const OctreeCompressKey &key,
                       size_t depthMask);

    void encodeColor(OctreeCompressData &raw_data_out, OctreeCompressNode &root, float QStep);

    void decodeColor(OctreeCompressData &raw_data_in, OctreeCompressNode &root, float QStep);

    void traverseByLevel(OctreeCompressData &raw_data_out, OctreeCompressNode &root);

    void restoreOctree(OctreeCompressData &raw_data_in, OctreeCompressNode &root, int max_depth, int dcm_max_depth);

    void getPointRecurse(std::vector<Point3f> &restorePointCloud, std::vector<Point3f> &restoreColor, unsigned long x_key, unsigned long y_key,
                         unsigned long z_key, Ptr<OctreeCompressNode> &_node, double resolution, Point3f ori, bool hasColor);

    void decodeStreamToCharVector(std::istream &inputByteStream_arg, std::vector<unsigned char> &outputCharVector_arg);

    void
    encodeCharVectorToStream(const std::vector<unsigned char> &inputCharVector_arg, std::ostream &outputByteStream_arg);


    OctreeCompressNode::OctreeCompressNode() : children(OCTREE_CHILD_NUM, nullptr), neigh(OCTREE_NEIGH_SIZE, nullptr),
                                               depth(0), size(0), origin(0, 0, 0), parentIndex(-1), pointNum(0) {
    }

    OctreeCompressNode::OctreeCompressNode(int _depth, double _size, const Point3f &_origin, const Point3f &_color,
                                           int _parentIndex, int _pointNum) : children(OCTREE_CHILD_NUM),
                                                                              neigh(OCTREE_NEIGH_SIZE), depth(_depth),
                                                                              size(_size), origin(_origin),
                                                                              color(_color), parentIndex(_parentIndex),
                                                                              pointNum(_pointNum) {
    }

    bool OctreeCompressNode::isPointInBound(const Point3f &_point) const {
        return isPointInBound(_point, origin, size);
    }

    bool OctreeCompressNode::isPointInBound(const Point3f &_point, const Point3f &_origin, double _size) const {
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

    struct OctreeCompress::Impl {
    public:
        Impl() : maxDepth(-1), size(0), origin(0, 0, 0), resolution(0), depthMask(0) {}

        ~Impl() {}

        // The pointer to Octree root node.
        Ptr<OctreeCompressNode> rootNode = nullptr;
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
        //!
        bool hasColor{};
    };

    OctreeCompress::OctreeCompress() : p(new Impl) {
        p->maxDepth = -1;
        p->size = 0;
        p->origin = Point3f(0, 0, 0);
    }

    OctreeCompress::OctreeCompress(int _maxDepth, double _size, const Point3f &_origin) : p(new Impl) {
        p->maxDepth = _maxDepth;
        p->size = _size;
        p->origin = _origin;
    }

    OctreeCompress::OctreeCompress(const std::vector<Point3f> &_pointCloud, int _maxDepth) : p(new Impl) {
        CV_Assert(_maxDepth > -1);
        this->create(_pointCloud, _maxDepth);
    }

    OctreeCompress::OctreeCompress(int _maxDepth) : p(new Impl) {
        p->maxDepth = _maxDepth;
        p->size = 0;
        p->origin = Point3f(0, 0, 0);
    }

    OctreeCompress::~OctreeCompress() = default;

    bool OctreeCompress::insertPoint(const Point3f &point, const Point3f &color, double resolution, size_t depthMask) {
        if (p->rootNode.empty()) {
            p->rootNode = new OctreeCompressNode(0, p->size, p->origin, color, -1, 0);
        }

        OctreeCompressKey key(floor((point.x - this->p->origin.x) / resolution),
                              floor((point.y - this->p->origin.y) / resolution),
                              floor((point.z - this->p->origin.z) / resolution));
        //std::cout<<"x="<<key.x_key<<" y="<<key.y_key<<" z="<<key.z_key<<std::endl;

        bool result = insertPointRecurse(p->rootNode, point, color, p->maxDepth, key, depthMask);
        return result;
    }

    bool OctreeCompress::create(const std::vector<Point3f> &pointCloud, const std::vector<Point3f> &colorAttribute,
                                double resolution) {
        if (resolution > 0) {
            p->resolution = resolution;
        }

        //CV_Assert( p->maxDepth > -1 && !pointCloud.empty());

        if (pointCloud.empty())
            return false;

        p->hasColor = !colorAttribute.empty();

        Point3f maxBound(pointCloud[0]);
        Point3f minBound(pointCloud[0]);
        Point3f center, temp;

        // Find center coordinate of PointCloud data.
        for (auto idx: pointCloud) {
            maxBound.x = max(idx.x, maxBound.x);
            maxBound.y = max(idx.y, maxBound.y);
            maxBound.z = max(idx.z, maxBound.z);

            minBound.x = min(idx.x, minBound.x);
            minBound.y = min(idx.y, minBound.y);
            minBound.z = min(idx.z, minBound.z);
        }

        double maxSize = max(max(maxBound.x - minBound.x, maxBound.y - minBound.y), maxBound.z - minBound.z);
        p->maxDepth = ceil(log2(maxSize / resolution));
        // TODO depthMask should not be a property of Octree
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
            Point3f insertColor = p->hasColor ? colorAttribute[idx] : Point3f(0.0f, 0.0f, 0.0f);
            if (!insertPoint(pointCloud[idx], insertColor, resolution, p->depthMask)) {
                cnt++;
            }
        }

        CV_LOG_IF_WARNING(nullptr, cnt != 0, "OverAll " << cnt
                                                        << " points has been ignored! The number of point clouds contained in the current octree is "
                                                        << pointCloud.size() - cnt)
        return true;
    }

    bool OctreeCompress::create(const std::vector<Point3f> &pointCloud, double resolution) {
        std::vector<Point3f> v;
        return this->create(pointCloud, v, resolution);
    }

    void OctreeCompress::traverse(std::ostream &outputStream) {
        OctreeCompressData raw_data;

        // BFS traverse Octree nodes (Geometry data)
        traverseByLevel(raw_data, *p->rootNode);

        // TODO： Here the QStep should be implemented as a user variable
        float QStep = 10.0f;
        // DFS traverse Octree nodes (Color data)
        if (p->hasColor)
            encodeColor(raw_data, *p->rootNode, QStep);
        else
            QStep = -1.0f;

        // Set header
        // +-----------------------------    +
        // | dcm_max_depth (1char)           |
        // | resolution (double, 8bytes)     |
        // | origin X,Y,Z (3float, 12bytes)  |
        // | max_depth  (3float, 12bytes)    |
        // | en_has_color (0 or 1, 1char)|
        // | Quantization step (float)       |
        // +-----------------------------    +
        // TODO header ignored maxdepth, size
        unsigned char en_resolution[sizeof(double)];
        unsigned char en_origin[3 * sizeof(float)];
        unsigned char en_max_depth[sizeof(int)];
        unsigned char en_has_color;
        unsigned char en_QStep[sizeof(float)];

        memcpy(&en_resolution, &(p->resolution), sizeof(p->resolution));
        memcpy(en_origin, &p->origin.x, sizeof(en_origin) / 3);
        memcpy(&en_origin[4], &p->origin.y, sizeof(en_origin) / 3);
        memcpy(&en_origin[8], &p->origin.z, sizeof(en_origin) / 3);
        memcpy(en_max_depth, &p->maxDepth, sizeof(en_max_depth));
        en_has_color = (unsigned char) p->hasColor;
        memcpy(&en_QStep, &QStep, sizeof(QStep));

        // push additional info
        for (unsigned char &i: en_resolution)
            raw_data.header.push_back(i);

        for (unsigned char &i: en_origin)
            raw_data.header.push_back(i);

        for (unsigned char &i: en_max_depth)
            raw_data.header.push_back(i);

        raw_data.header.push_back(en_has_color);

        for (unsigned char &i: en_QStep)
            raw_data.header.push_back(i);

        // TODO header shouldn't encode, big table！
        encodeCharVectorToStream(raw_data.header, outputStream);
        encodeCharVectorToStream(raw_data.occ_codes, outputStream);
        encodeCharVectorToStream(raw_data.dcm_flags, outputStream);
        encodeCharVectorToStream(raw_data.dcm_codes, outputStream);
        encodeCharVectorToStream(raw_data.occ_lookup_table, outputStream);
        encodeCharVectorToStream(raw_data.dcm_lookup_table, outputStream);
        if (p->hasColor)
            encodeCharVectorToStream(raw_data.color_codes, outputStream);

    }

    void OctreeCompress::reStore(std::istream &inputStream) {

        OctreeCompressData raw_data;

        decodeStreamToCharVector(inputStream, raw_data.header);
        decodeStreamToCharVector(inputStream, raw_data.occ_codes);
        decodeStreamToCharVector(inputStream, raw_data.dcm_flags);
        decodeStreamToCharVector(inputStream, raw_data.dcm_codes);
        decodeStreamToCharVector(inputStream, raw_data.occ_lookup_table);
        decodeStreamToCharVector(inputStream, raw_data.dcm_lookup_table);

        this->clear();
        unsigned char de_resolution[sizeof(double)];
        unsigned char de_origin[3 * sizeof(float)];
        unsigned char de_max_depth[sizeof(int)];
        unsigned char de_has_color;
        unsigned char de_QStep[sizeof(float)];
        float x, y, z;
        double new_res;
        int new_max_d;
        int dcm_max_depth;
        float QStep;

        dcm_max_depth = (int) (raw_data.header.front());
        raw_data.header.erase(raw_data.header.begin());

        for (unsigned char &i: de_resolution) {
            i = raw_data.header.front();
            raw_data.header.erase(raw_data.header.begin());
        }
        for (unsigned char &i: de_origin) {
            i = raw_data.header.front();
            raw_data.header.erase(raw_data.header.begin());
        }
        for (unsigned char &i: de_max_depth) {
            i = raw_data.header.front();
            raw_data.header.erase(raw_data.header.begin());
        }
        de_has_color = raw_data.header.front();
        raw_data.header.erase(raw_data.header.begin());

        for (unsigned char &i: de_QStep) {
            i = raw_data.header.front();
            raw_data.header.erase(raw_data.header.begin());
        }

        memcpy(&new_res, &de_resolution, sizeof(new_res));
        memcpy(&x, &de_origin[0], sizeof(x));
        memcpy(&y, &de_origin[4], sizeof(y));
        memcpy(&z, &de_origin[8], sizeof(z));
        memcpy(&new_max_d, &de_max_depth, sizeof(new_max_d));
        memcpy(&QStep, &de_QStep, sizeof(QStep));
        Point3f new_ori(x, y, z);
        p->origin = new_ori;
        p->resolution = new_res;
        p->maxDepth = new_max_d;
        p->hasColor = de_has_color & 1;


        p->rootNode = new OctreeCompressNode();
        restoreOctree(raw_data, *p->rootNode, p->maxDepth, dcm_max_depth);

        std::cout << "Restored before color" << std::endl;
        // decode color attribute
        if (p->hasColor) {
            decodeStreamToCharVector(inputStream, raw_data.color_codes);
            std::cout << "Decode stream" << std::endl;
            decodeColor(raw_data, *p->rootNode, QStep);
        }
    }

    void
    OctreeCompress::getPointCloudByOctree(std::vector<Point3f> &restorePointCloud, std::vector<Point3f> &restoreColor) {
        Ptr<OctreeCompressNode> root = p->rootNode;
        double resolution = p->resolution;

        getPointRecurse(restorePointCloud, restoreColor, 0, 0, 0, root, resolution, p->origin, p->hasColor);
    }

    void OctreeCompress::setMaxDepth(int _maxDepth) {
        if (_maxDepth)
            this->p->maxDepth = _maxDepth;
    }

    void OctreeCompress::setSize(double _size) {
        this->p->size = _size;
    };

    void OctreeCompress::setOrigin(const Point3f &_origin) {
        this->p->origin = _origin;
    }

    void OctreeCompress::clear() {
        if (!p->rootNode.empty()) {
            p->rootNode.release();
        }

        p->size = 0;
        p->maxDepth = -1;
        p->origin = Point3f(0, 0, 0); // origin coordinate
        p->resolution = 0;
        p->depthMask = 0;
    }

    bool OctreeCompress::empty() const {
        return p->rootNode.empty();
    }

    bool OctreeCompress::isPointInBound(const Point3f &_point) const {
        return _isPointInBound(_point, p->origin, p->size);
    }

    bool
    insertPointRecurse(Ptr<OctreeCompressNode> &_node, const Point3f &point, const Point3f &color, int maxDepth,
                       const OctreeCompressKey &key,
                       size_t depthMask) {
        OctreeCompressNode &node = *_node;

        if (node.depth == maxDepth) {
            if (node.pointNum == 0) {
                node.color = color;
                node.isLeaf = true;
                node.pointNum++;
                return true;
            }
            return false;
        }

        size_t childIndex = key.findChildIdxByMask(depthMask);
        //std::cout<<childIndex<<std::endl;
        if (node.children[childIndex].empty()) {
            node.children[childIndex] = new OctreeCompressNode(node.depth + 1, 0, Point3f(0, 0, 0), Point3f(0, 0, 0),
                                                               int(childIndex), 0);
            node.children[childIndex]->parent = _node;
        }

        bool result = insertPointRecurse(node.children[childIndex], point, color, maxDepth, key, depthMask >> 1);
        node.pointNum += result;
        return result;
    }

    /* one-to-one Map 6N descriptor to idx of lookup table:
        (001, 0) -> 001
        (001, 1) -> 011
        (010, 0) -> 010
        (010, 1) -> 110
        (100, 0) -> 100
        (100, 1) -> 110
        idx 000 and 111 are reserved

       6N descriptor explain: see setNeighbour()
    */
    unsigned char mapNeighPos2Idx(unsigned char dim_mask, unsigned char sign) {
        return dim_mask | (sign << (dim_mask & 3));
    }

    // Set node's neighbour vector pointing at 6 adjacent nodes(cubes) at same level.
    void setNeighbour(OctreeCompressNode &node) {
        if (node.parent == nullptr) {
            // root should stop here
            return;
        }

        // 6N neighbour position descriptor explain:
        //   dim_mask: the 6N neighbour dimension, two on x(001), two on y(010), tow on z(100)
        //   sign direction (0 negative, 1 positive)
        // these can fully identify 6 neighbours
        unsigned char dim_mask = 1;
        bool sign = false;

        // neigh_6N_idx: where is neigh's 6N position according to curr node
        // neigh_parent_idx: what is neigh's octreeNode position according to parent
        unsigned char neigh_6N_idx;
        unsigned char neigh_parent_idx;

        // iterate neighbours position(dim_mask, sign), get neigh[1..6].
        while (dim_mask <= 4) {
            // neigh_parent_idx only depend on parentIndex and dim direction
            neigh_parent_idx = (unsigned char) node.parentIndex ^ dim_mask;
            neigh_6N_idx = mapNeighPos2Idx(dim_mask, (unsigned char) sign);

            if (bool(dim_mask & (unsigned char) node.parentIndex) == sign) {
                // neigh inside curr node's parent's neighbour
                if (!node.parent->neigh[neigh_6N_idx]) {
                    node.neigh[neigh_6N_idx] = nullptr;
                } else {
                    node.neigh[neigh_6N_idx] = node.parent->neigh[neigh_6N_idx]->children[neigh_parent_idx];
                }
            } else {
                // neigh share same parent with curr node
                node.neigh[neigh_6N_idx] = node.parent->children[neigh_parent_idx];
            }

            // next neighbours position
            if (sign) {
                dim_mask <<= 1;
                sign = false;
            } else sign = true;
        }
    }

    void traverseByLevel(OctreeCompressData &raw_data_out,
                         OctreeCompressNode &root) {

        try {
            // max depth before all become dcm
            int dcm_max_depth = -1;
            bool dcm_max_depth_full = true;

            // Prediction Tables
            int look_up_table[64][256];
            unsigned char weight_table[64][256];
            int look_up_DCM_table[64][8];
            unsigned char weight_DCM_table[64][8];
            memset(look_up_table, 0, sizeof(look_up_table));
            memset(weight_table, 0, sizeof(weight_table));
            memset(look_up_DCM_table, 0, sizeof(look_up_DCM_table));
            memset(weight_DCM_table, 0, sizeof(weight_DCM_table));

            // Pre-Traverse
            std::queue<OctreeCompressNode *> nodeQueue;
            nodeQueue.push(&root);

            while (!nodeQueue.empty()) {

                OctreeCompressNode &node = *(nodeQueue.front());
                nodeQueue.pop();

                // Stop at last leaf level, no need to encode leaf node
                if (node.isLeaf) {
                    break;
                }

                // set neighbours for curr node
                setNeighbour(node);

                // Direct coding mode (DCM) for lookup

                if (node.parent != nullptr) {
                    // check if it is single point
                    if (node.pointNum == 1) {
                        if (node.depth > dcm_max_depth) {
                            dcm_max_depth = node.depth;
                            dcm_max_depth_full = true;
                        }

                        // record DCM branching, refer to neighbour
                        unsigned char neigh = OctreeCompressKey::getNeighPattern(node);
                        OctreeCompressNode pNode = node;
                        while (!pNode.isLeaf) {
                            for (unsigned char i = 0; i < pNode.children.size(); i++) {
                                if (!pNode.children[i].empty()) {
                                    look_up_DCM_table[neigh][i]++;
                                    pNode = *(pNode.children[i]);
                                    break;
                                }
                            }
                        }
                        // DCM complete
                        continue;
                    } else if (dcm_max_depth_full) {
                        dcm_max_depth_full = false;
                    }
                }

                // Octree mode (further branching)
                for (unsigned char i = 0; i < 8; i++) {
                    if (!node.children[i].empty()) {
                        nodeQueue.push(node.children[i]);
                    }
                }
                // record Octree branching, refer to neighbour
                look_up_table[(int) OctreeCompressKey::getNeighPattern(node)]
                [(int) OctreeCompressKey::getBitPattern(node)]++;
            }

            if (!dcm_max_depth_full) {
                dcm_max_depth++;
            }

            raw_data_out.header.push_back((unsigned char) dcm_max_depth);

            // Sort lookup to get weight table
            for (int i = 0; i < 64; i++) {
                std::vector<std::pair<int, int>> temp_vec;
                std::vector<std::pair<int, int>> temp_DCM_vec;

                for (int j = 0; j < 256; j++) {
                    temp_vec.emplace_back(look_up_table[i][j], j);
                }
                for (int j = 0; j < 8; j++) {
                    temp_DCM_vec.emplace_back(look_up_DCM_table[i][j], j);
                }

                std::sort(temp_vec.rbegin(), temp_vec.rend());
                std::sort(temp_DCM_vec.rbegin(), temp_DCM_vec.rend());

                for (int w = 0; w < 256; w++) {
                    weight_table[i][temp_vec[w].second] = (unsigned char) w;
                    raw_data_out.occ_lookup_table.push_back((unsigned char) (temp_vec[w].second));
                }
                for (int w = 0; w < 8; w++) {
                    weight_DCM_table[i][temp_DCM_vec[w].second] = (unsigned char) w;
                    raw_data_out.dcm_lookup_table.push_back((unsigned char) (temp_DCM_vec[w].second));
                }
            }

            // True Traverse
            nodeQueue = std::queue<OctreeCompressNode *>();
            nodeQueue.push(&root);

            while (!nodeQueue.empty()) {

                OctreeCompressNode &node = *(nodeQueue.front());
                nodeQueue.pop();

                // Stop at last leaf level, no need to encode leaf node
                if (node.isLeaf) {
                    break;
                }

                setNeighbour(node);
                // neigh: the core of prediction
                unsigned char neigh = OctreeCompressKey::getNeighPattern(node);

                // Direct coding mode (DCM)

                if (node.parent != nullptr) {
                    if (node.pointNum == 1) {
                        if (node.depth < dcm_max_depth) raw_data_out.dcm_flags.push_back(true);

                        // Apply DCM
                        OctreeCompressNode pNode = node;
                        while (!pNode.isLeaf) {
                            for (unsigned char i = 0; i < pNode.children.size(); i++) {
                                if (!pNode.children[i].empty()) {
                                    // push back DCM weighted Prediction loss to output vector

                                    // Pred
                                    raw_data_out.dcm_codes.push_back(weight_DCM_table[neigh][i]);
                                    // Direct
                                    // raw_data_out.dcm_codes.push_back(i);

                                    pNode = *(pNode.children[i]);
                                    break;
                                }
                            }
                        }
                        continue;
                    } else {
                        if (node.depth < dcm_max_depth) raw_data_out.dcm_flags.push_back(false);
                    }
                }

                // Octree mode (further branching)
                // push back weighted Prediction loss to output vector

                // Pred
                raw_data_out.occ_codes.push_back(weight_table[neigh][OctreeCompressKey::getBitPattern(node)]);
                // Direct
                // raw_data_out.occ_codes.push_back(OctreeCompressKey::getBitPattern(node));

                for (unsigned char i = 0; i < 8; i++) {
                    if (!node.children[i].empty()) {
                        nodeQueue.push(node.children[i]);
                    }
                }
            }
        }
        catch (std::bad_alloc) {

        }

    }

    void restoreOctree(OctreeCompressData &raw_data_in,
                       OctreeCompressNode &root,
                       int max_depth, int dcm_max_depth) {

        std::queue<OctreeCompressNode *> nodeQueue;
        nodeQueue.push(&root);

        size_t index = 0;
        size_t index_dcm_flag = 0;
        size_t index_dcm = 0;
        size_t index_bound = raw_data_in.occ_codes.size();

        try {

            // restore lookup table
            unsigned char occ_table[64][256];
            unsigned char dcm_table[64][8];
            memset(occ_table, (unsigned char) 0, sizeof(occ_table));
            memset(dcm_table, (unsigned char) 0, sizeof(dcm_table));
            for (int i = 0; i < 64; i++) {
                for (int j = 0; j < 256; j++) {
                    occ_table[i][j] = raw_data_in.occ_lookup_table[i * 256 + j];
                }
            }
            for (int i = 0; i < 64; i++) {
                for (int j = 0; j < 8; j++) {
                    dcm_table[i][j] = raw_data_in.dcm_lookup_table[i * 8 + j];
                }
            }

            // Restore tree
            while (!nodeQueue.empty()) {

                OctreeCompressNode &node = *(nodeQueue.front());
                nodeQueue.pop();

                setNeighbour(node);
                // neigh: the core of prediction
                unsigned char neigh = OctreeCompressKey::getNeighPattern(node);

                // Direct coding mode (DCM)
                if (node.parent != nullptr) {

                    if (node.depth >= dcm_max_depth || raw_data_in.dcm_flags[index_dcm_flag++]) {
                        //Read DCM
                        OctreeCompressNode *pNode = &node;
                        while (pNode->depth < max_depth) {

                            // Pred
                            unsigned char i = dcm_table[neigh][raw_data_in.dcm_codes[index_dcm++]];
                            // Direct
                            // unsigned char i = raw_data_in.dcm_codes[index_dcm++];

                            // TODO Except depth and parent index, nothing set, including color(include Octree mode)
                            pNode->children[i] = new OctreeCompressNode(pNode->depth + 1, 0, Point3f(0, 0, 0),
                                                                        Point3f(0, 0, 0), int(i), 0);
                            pNode->children[i]->parent = pNode;
                            pNode = pNode->children[i];
                        }
                        pNode->isLeaf = true;
                        while (pNode != nullptr) {
                            ++(pNode->pointNum);
                            pNode = pNode->parent;
                        }

                        continue;
                    }
                }

                // Octree mode
                if (index >= index_bound) {
                    node.isLeaf = true;
                    OctreeCompressNode *pNode = &node;
                    while (pNode != nullptr) {
                        ++(pNode->pointNum);
                        pNode = pNode->parent;
                    }
                    continue;
                }
                unsigned char mask = 1;

                // Pred
                unsigned char occup_code = occ_table[neigh][raw_data_in.occ_codes[index++]];
                // Direct
                // unsigned char occup_code = raw_data_in.occ_codes[index++];

                for (unsigned char i = 0; i < 8; i++) {
                    if (!!(occup_code & mask)) {
                        node.children[i] = new OctreeCompressNode(node.depth + 1, 0, Point3f(0, 0, 0),
                                                                  Point3f(0, 0, 0), int(i), 0);
                        node.children[i]->parent = &node;
                        nodeQueue.push(node.children[i]);
                    }
                    mask = mask << 1;
                }
            }
        }
        catch (std::bad_alloc) {

        }

    }

    void
    getPointRecurse(std::vector<Point3f> &restorePointCloud, std::vector<Point3f> &restoreColor, unsigned long x_key,
                    unsigned long y_key,
                    unsigned long z_key, Ptr<OctreeCompressNode> &_node, double resolution, Point3f ori,
                    bool hasColor) {
        OctreeCompressNode node = *_node;
        if (node.isLeaf) {
            restorePointCloud.emplace_back(
                    (float) (resolution * x_key) + (float) (resolution * 0.5) + ori.x,
                    (float) (resolution * y_key) + (float) (resolution * 0.5) + ori.y,
                    (float) (resolution * z_key) + (float) (resolution * 0.5) + ori.z);
            if (hasColor) {
                restoreColor.emplace_back(node.color);
            }
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
                // TODO Debug
                if ((z_copy << 1) < z_copy || (y_copy << 1) < y_copy || (x_copy << 1) < x_copy) {
                    std::cout << "!!!";
                }
                size_t x_offSet = !!(x_mask & i);
                size_t y_offSet = !!(y_mask & i);
                size_t z_offSet = !!(z_mask & i);
                x_copy = (x_copy << 1) | x_offSet;
                y_copy = (y_copy << 1) | y_offSet;
                z_copy = (z_copy << 1) | z_offSet;
                getPointRecurse(restorePointCloud, restoreColor, x_copy, y_copy, z_copy, node.children[i], resolution,
                                ori, hasColor);
            }
        }
    };

// Range Coding methods based on implementation by Julius Kammerl (julius@kammerl.de) from PCL
    void encodeCharVectorToStream(const std::vector<unsigned char> &inputCharVector_arg,
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

        auto input_size = static_cast<size_t> (inputCharVector_arg.size());

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
            // read symbol
            symbol = static_cast<std::uint8_t>(inputCharVector_arg[readPos++]);

            // map to range
            low += part_idx[symbol] * (range /= part_idx[256]);
            range *= part_idx[symbol + 1] - part_idx[symbol];

            // renormalization
            // first case: range is completely inside a block of size adjust_limit
            // second case: range is too small while first case continuously miss, preform resize to bottom_limit
            while ((low ^ (low + range)) < adjust_limit ||
                   ((range < bottom_limit) && ((range = -int(low) & (bottom_limit - 1)), 1))) {
                auto out = static_cast<unsigned char> (low >> 24);
                range <<= 8;
                low <<= 8;
                outputCharVector_.push_back(out);
            }

        }

        // flush remaining data
        for (int i = 0; i < 4; i++) {
            auto out = static_cast<unsigned char> (low >> 24);
            outputCharVector_.push_back(out);
            low <<= 8;
        }

        // write encoded data to stream
        const size_t vec_len = inputCharVector_arg.size();
        outputByteStream_arg.write(reinterpret_cast<const char *> (&vec_len), sizeof(vec_len));
        outputByteStream_arg.write(reinterpret_cast<const char *> (&outputCharVector_[0]), outputCharVector_.size());

    }

    void decodeStreamToCharVector(std::istream &inputByteStream_arg,
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

        outputPos = 0;

        // read cumulative frequency table
        inputByteStream_arg.read(reinterpret_cast<char *> (&part_idx[0]), sizeof(part_idx));

        // read vec_size
        inputByteStream_arg.read(reinterpret_cast<char *> (&output_size), sizeof(output_size));

        outputCharVector_arg.clear();
        outputCharVector_arg.resize(output_size);

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

    void Haar3DRecursive(OctreeCompressNode *node, std::vector<Point3f> &haarCoefficients,
                         std::vector<OctreeCompressNode *> &cubes, size_t &N) {
        if (!node)
            return;
        if (node->isLeaf) {
            // convert RGB color to YUV
            node->RAHTCoefficient.x = 0.2126f * node->color.x + 0.7152f * node->color.y + 0.0722f * node->color.z;
            node->RAHTCoefficient.y = (node->color.z - node->RAHTCoefficient.x) / 1.85563f;
            node->RAHTCoefficient.z = (node->color.x - node->RAHTCoefficient.x) / 1.57480f;
            return;
        }

        for (const auto &child: node->children) {
            Haar3DRecursive(child, haarCoefficients, cubes, N);
        }

        std::vector<OctreeCompressNode *> prevCube(node->children.size());
        std::vector<OctreeCompressNode *> currCube(node->children.size());

        // use the pre-allocated object
        for (size_t idx = 0; idx < node->children.size(); ++idx) {
            prevCube[idx] = cubes[idx];
            currCube[idx] = cubes[node->children.size() + idx];
        }


        // copy node info from octree
        for (size_t idx = 0; idx < node->children.size(); ++idx) {
            if (!node->children[idx]) {
                prevCube[idx]->pointNum = 0;
                continue;
            }
            prevCube[idx]->RAHTCoefficient = node->children[idx]->RAHTCoefficient;
            prevCube[idx]->pointNum = node->children[idx]->pointNum;
        }

        size_t cubeSize = prevCube.size();
        size_t stepSize = 2;

        // start doing transform in x then y then z direction
        while (true) {
            for (size_t x = 0; x < cubeSize; x += stepSize) {
                OctreeCompressNode *node1 = prevCube[x];
                OctreeCompressNode *node2 = prevCube[x + 1];

                if (!node1->pointNum && !node2->pointNum) {
                    currCube[x / stepSize]->pointNum = 0;
                    continue;
                }

                // transform under this condition
                if (node1->pointNum && node2->pointNum) {
                    currCube[x / stepSize] = new OctreeCompressNode;
                    auto w1 = (float) node1->pointNum;
                    auto w2 = (float) node2->pointNum;
                    float w = w1 + w2;
                    float a1 = sqrt(w1) / sqrt(w);
                    float a2 = sqrt(w2) / sqrt(w);

                    currCube[x / stepSize]->pointNum = (int) w;

                    // YUV
                    float YLowPass = a1 * node1->RAHTCoefficient.x + a2 * node2->RAHTCoefficient.x;
                    float ULowPass = a1 * node1->RAHTCoefficient.y + a2 * node2->RAHTCoefficient.y;
                    float VLowPass = a1 * node1->RAHTCoefficient.z + a2 * node2->RAHTCoefficient.z;

                    currCube[x / stepSize]->RAHTCoefficient = Point3f(YLowPass, ULowPass, VLowPass);

                    float YHighPass = a1 * node2->RAHTCoefficient.x - a2 * node1->RAHTCoefficient.x;
                    float UHighPass = a1 * node2->RAHTCoefficient.y - a2 * node1->RAHTCoefficient.y;
                    float VHighPass = a1 * node2->RAHTCoefficient.z - a2 * node1->RAHTCoefficient.z;

                    haarCoefficients[N++] = Point3f(YHighPass, UHighPass, VHighPass);
                    continue;
                }
                // if no partner to transform, then directly use the value
                currCube[x / stepSize]->pointNum = node1->pointNum ? node1->pointNum : node2->pointNum;
                currCube[x / stepSize]->RAHTCoefficient = node1->pointNum ? node1->RAHTCoefficient
                                                                          : node2->RAHTCoefficient;
            }

            cubeSize >>= 1;
            if (cubeSize < 2)
                break;

            // swap prevCube and currCube
            for (size_t k = 0; k < prevCube.size(); ++k) {
                prevCube[k]->pointNum = currCube[k]->pointNum;
                prevCube[k]->RAHTCoefficient = currCube[k]->RAHTCoefficient;
            }
        }

        // update selected node's coefficient in the octree
        node->RAHTCoefficient = currCube[0]->RAHTCoefficient;
    }

    // RAHT main - post-order traversal to generate RAHT coefficients in x,y,z directions
    // QStep: quantization step
    void encodeColor(OctreeCompressData &raw_data_out, OctreeCompressNode &root, float QStep) {

        std::vector<Point3f> haarCoefficients;

        size_t N = 0;

        size_t pointNum = root.pointNum;
        size_t colorNum = 3 * pointNum;

        haarCoefficients.resize(pointNum);

        std::vector<OctreeCompressNode *> cubes(root.children.size() << 1);
        for (auto &cube: cubes)
            cube = new OctreeCompressNode;


        // Obtain RAHT coefficients through 3D Haar Transform
        Haar3DRecursive(&root, haarCoefficients, cubes, N);
        haarCoefficients[N++] = root.RAHTCoefficient;

        // Init array for quantization
        assert(QStep > 0.0f);
        std::vector<int32_t> quantizedCoefficients(colorNum);

        // Quantization
        for (size_t i = 0; i < N; ++i) {
            quantizedCoefficients[i] = (int32_t) std::round(haarCoefficients[i].x / QStep);
            quantizedCoefficients[N + i] = (int32_t) std::round(haarCoefficients[i].y / QStep);
            quantizedCoefficients[(N << 1) + i] = (int32_t) std::round(haarCoefficients[i].z / QStep);
        }

        // save coefficients to raw_data_out for encoding
        raw_data_out.color_codes.resize(colorNum * 4, '\0');
        size_t cursor = 0;

        for (auto val: quantizedCoefficients) {
            // skip 0s
            if (!val) {
                cursor += 4;
                continue;
            }

            val = val > 0 ? (val << 1) : (((-val) << 1) - 1);
            raw_data_out.color_codes[cursor++] = static_cast<unsigned char>(val & 0xFF);
            raw_data_out.color_codes[cursor++] = static_cast<unsigned char>((val >> 8) & 0xFF);
            raw_data_out.color_codes[cursor++] = static_cast<unsigned char>((val >> 16) & 0xFF);
            raw_data_out.color_codes[cursor++] = static_cast<unsigned char>((val >> 24) & 0xFF);
        }

        for (auto &p: cubes)
            delete p;

        // end
    }

    // adaptive recursive preorder traversal
    // TODO: use memcpy for better performance?
    void invHaar3DRecursive(OctreeCompressNode *node, std::vector<Point3f> &haarCoefficients, std::vector<OctreeCompressNode *> &cubes, size_t &N) {
        if (!node)
            return;
        if (node->isLeaf) {
            // restore leaf nodes' RGB color
            float y, u, v, r, g, b;
            y = node->RAHTCoefficient.x + 0.5f;
            u = node->RAHTCoefficient.y;
            v = node->RAHTCoefficient.z;

            r = y + 1.5748f * v;
            g = y - 0.18733f * u - 0.46813f * v;
            b = y + 1.85563f * u;

            // Clipping from 0 to 255
            r = (r < 0.0f) ? 0.0f : ((r > 255.0f) ? 255.0f : r);
            g = (g < 0.0f) ? 0.0f : ((g > 255.0f) ? 255.0f : g);
            b = (b < 0.0f) ? 0.0f : ((b > 255.0f) ? 255.0f : b);

            node->color = Point3f(r, g, b);
            return;
        }

        // actual size of currCube in the loop
        size_t iterSize = 2;

        std::vector<OctreeCompressNode *> prevCube(8);
        std::vector<OctreeCompressNode *> currCube(8);

        for (size_t idx = 0; idx < node->children.size(); ++idx) {
            prevCube[idx] = cubes[idx];
            currCube[idx] = cubes[node->children.size() + idx];
        }

        // node = 1 -> 2 -> 4 -> 8 = child nodes
        // first set prev = node, then insert weight values into currCube by traversing children
        // then update currCube's HaarCoefficients using node's low-pass coefficient and high-pass coefficients from input
        // after that set prev = curr, then enter next loop

        prevCube[0]->RAHTCoefficient = node->RAHTCoefficient;
        prevCube[0]->pointNum = node->pointNum;

        while (true) {
            // sum stepSize number of nodes' pointNum values for currCube
            size_t stepSize = 8 / iterSize;

            // 1st loop: i = 0, 1   j = i*stepSize, j+1, ... , i+1 * stepSize
            for (size_t i = 0; i < iterSize; ++i) {
                currCube[i] = new OctreeCompressNode;
                for (size_t j = i * stepSize; j < i * stepSize + stepSize; ++j) {
                    if (node->children[j])
                        currCube[i]->pointNum += node->children[j]->pointNum;
                }
            }

            for (int i = (int) iterSize - 2; i >= 0; i -= 2) {
                OctreeCompressNode *fatherNode = prevCube[i / 2];
                OctreeCompressNode *node1 = currCube[i];
                OctreeCompressNode *node2 = currCube[i + 1];

                if (!node1->pointNum && !node2->pointNum)
                    continue;
                if (node1->pointNum && node2->pointNum) {
                    auto w1 = static_cast<float>(node1->pointNum);
                    auto w2 = static_cast<float>(node2->pointNum);
                    float w = w1 + w2;

                    float a1 = sqrt(w1) / sqrt(w);
                    float a2 = sqrt(w2) / sqrt(w);

                    // read coefficients from input array
                    Point3f lowPassCoefficient = fatherNode->RAHTCoefficient;
                    Point3f highPassCoefficient = haarCoefficients[N--];

                    // calculate YUV color value

                    node1->RAHTCoefficient = a1 * lowPassCoefficient - a2 * highPassCoefficient;
                    node2->RAHTCoefficient = a1 * highPassCoefficient + a2 * lowPassCoefficient;
                    continue;
                }
                node1->RAHTCoefficient = fatherNode->RAHTCoefficient;
                node2->RAHTCoefficient = fatherNode->RAHTCoefficient;
            }
            iterSize <<= 1;
            if (iterSize > 8)
                break;

//            for (auto p: prevCube)
//                delete p;

//            prevCube = currCube;

            for (size_t k = 0; k < prevCube.size(); ++k) {
                prevCube[k]->pointNum = currCube[k]->pointNum;
                prevCube[k]->RAHTCoefficient = currCube[k]->RAHTCoefficient;
            }

//            currCube.resize(iterSize);
        }

        for (int i = 0; i < 8; ++i)
            if (node->children[i])
                node->children[i]->RAHTCoefficient = currCube[i]->RAHTCoefficient;

//        for (auto p: prevCube)
//            delete p;

        // traverse child nodes
        for (int i = 7; i >= 0; --i)
            invHaar3DRecursive(node->children[i], haarCoefficients, cubes, N);

        // end
    }


    void decodeColor(OctreeCompressData &raw_data_in, OctreeCompressNode &root, float QStep) {

        size_t pointNum = root.pointNum;
        size_t colorNum = 3 * pointNum;
        size_t i, j, k;

        std::vector<int32_t> quantizedCoefficients(colorNum);
        // decode uchar vector
        for (i = 0, j = 0; i < colorNum; ++i) {
            int32_t dVal = 0;
            dVal |= static_cast<::int32_t>(raw_data_in.color_codes[j++]);
            dVal |= (static_cast<::int32_t>(raw_data_in.color_codes[j++]) << 8);
            dVal |= (static_cast<::int32_t>(raw_data_in.color_codes[j++]) << 16);
            dVal |= (static_cast<::int32_t>(raw_data_in.color_codes[j++]) << 24);
            // unsigned to signed
            quantizedCoefficients[i] = dVal & 0x1 ? -(dVal >> 1) - 1 : (dVal >> 1);
        }

        // de-quantization
        std::vector<Point3f> Haar3DCoefficients(pointNum);
        for (i = 0, j = i + pointNum, k = j + pointNum; i < pointNum; ++i) {
            Haar3DCoefficients[i].x = static_cast<float>(quantizedCoefficients[i]) * QStep;
            Haar3DCoefficients[i].y = static_cast<float>(quantizedCoefficients[j++]) * QStep;
            Haar3DCoefficients[i].z = static_cast<float>(quantizedCoefficients[k++]) * QStep;
        }

        size_t N = Haar3DCoefficients.size() - 1;
        root.RAHTCoefficient = Haar3DCoefficients[N--];

        std::vector<OctreeCompressNode *> cubes(root.children.size() << 1);
        for (auto &cube: cubes)
            cube = new OctreeCompressNode;


        invHaar3DRecursive(&root, Haar3DCoefficients, cubes, N);

        for (auto &cube: cubes)
            delete cube;

    }
}