// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html

#include "precomp.hpp"
#include "octree_compression.hpp"

#include <fstream>

#define OCTREE_CHILD_NUM 8

namespace cv {

    void Haar3DRecursive(OctreeCompressNode *node, std::vector<Point3f> &haarCoefficients, size_t &N);

    static bool _isPointInBound(const Point3f &_point, const Point3f &_origin, double _size);

    static bool
    insertPointRecurse(Ptr<OctreeCompressNode> &node, const Point3f &point, const Point3f &color, int maxDepth,
                       const OctreeCompressKey &key,
                       size_t depthMask);

    void traverseByLevel(std::vector<unsigned char> &binary_tree_out_arg, OctreeCompressNode &root);

    void restoreOctree(const std::vector<unsigned char> &binary_tree_out_arg, OctreeCompressNode &root, size_t len);

    void getPointRecurse(std::vector<Point3f> &restorePointCloud, unsigned long x_key, unsigned long y_key,
                         unsigned long z_key, Ptr<OctreeCompressNode> &_node, double resolution, Point3f ori);

    OctreeCompressNode::OctreeCompressNode() : children(OCTREE_CHILD_NUM, nullptr), depth(0), size(0), origin(0, 0, 0),
                                               parentIndex(-1), pointNum(0) {
    }

    OctreeCompressNode::OctreeCompressNode(int _depth, double _size, const Point3f &_origin, const Point3f &_color,
                                           int _parentIndex,
                                           int _pointNum) : children(OCTREE_CHILD_NUM), depth(_depth), size(_size),
                                                            origin(_origin), color(_color), parentIndex(_parentIndex),
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
        bool hasColor;
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

        CV_LOG_IF_WARNING(NULL, cnt != 0, "OverAll " << cnt
                                                     << " points has been ignored! The number of point clouds contained in the current octree is "
                                                     << pointCloud.size() - cnt);
        return true;
    }

    bool OctreeCompress::create(const std::vector<Point3f> &pointCloud, double resolution) {
        std::vector<Point3f> v;
        return this->create(pointCloud, v, resolution);
    }


    void OctreeCompress::traverse(std::vector<unsigned char> &bit_out) {

        // traverse Octree nodes
        traverseByLevel(bit_out, *p->rootNode);

        // TODO change additional info to header not tail
        // additional information (at tail)
        // +-----------------------------   +
        // | resolution (double, 8bytes)    |
        // | origin X,Y,Z (3float, 12bytes) |
        // | bit_out len (size_t, 8bytes)   |
        // +-----------------------------   +
        //TODO use sizeof() instead of hard coding size

        //calc additional info
        size_t length = bit_out.size();
        unsigned char len_out[8];
        memcpy(len_out, &length, sizeof(len_out));

        unsigned char ori[12];
        memcpy(ori, &p->origin.x, sizeof(ori) / 3);
        memcpy(&ori[4], &p->origin.y, sizeof(ori) / 3);
        memcpy(&ori[8], &p->origin.z, sizeof(ori) / 3);

        unsigned char res[8];
        memcpy(&res, &(p->resolution), sizeof(p->resolution));

        // push additional info
        for (unsigned char &i: res) {
            bit_out.push_back(i);
        }
        for (unsigned char &i: ori) {
            bit_out.push_back(i);
        }
        for (unsigned char &i: len_out) {
            bit_out.push_back(i);
        }
    }

    void OctreeCompress::reStore(const std::vector<unsigned char> &bit_out) {

        this->clear();
        size_t bit_out_len = bit_out.size();
        size_t len;
        Point3f ori(0, 0, 0);
        memcpy(&len, &bit_out[bit_out_len - 8], sizeof(len));
        memcpy(&(p->resolution), &bit_out[len], sizeof(double));
        memcpy(&ori.x, &bit_out[len + 8], sizeof(float));
        memcpy(&ori.y, &bit_out[len + 8 + 4], sizeof(float));
        memcpy(&ori.z, &bit_out[len + 8 + 8], sizeof(float));
        p->rootNode = new OctreeCompressNode(0, p->size, Point3f(0, 0, 0), Point3f(0, 0, 0), -1, 0);
        p->origin = ori;
        restoreOctree(bit_out, *p->rootNode, len);
    }

    void OctreeCompress::getPointCloudByOctree(std::vector<Point3f> &restorePointCloud) {
        Ptr<OctreeCompressNode> root = p->rootNode;
        double resolution = p->resolution;

        getPointRecurse(restorePointCloud, 0, 0, 0, root, resolution, p->origin);
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
                // convert color from RGB to YUV
                Point3f YUVColor;
                YUVColor.x = 0.299f * color.x + 0.587f * color.y + 0.114f * color.z;
                YUVColor.y = 0.492f * (color.y - YUVColor.x);
                YUVColor.z = 0.877f * (color.x - YUVColor.x);

                node.color = YUVColor;

                // node.color = color;
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

    void traverseByLevel(std::vector<unsigned char> &binary_tree_out_arg,
                         OctreeCompressNode &root) {

        std::queue<OctreeCompressNode *> nodeQueue;
        nodeQueue.push(&root);

        try {

            while (!nodeQueue.empty()) {

                OctreeCompressNode &node = *(nodeQueue.front());

                // stop at last level, no need to encode leaf node
                if (node.isLeaf) {
                    break;
                }

                // Octree mode (further branching)

                binary_tree_out_arg.push_back(OctreeCompressKey::getBitPattern(node));
                nodeQueue.pop();
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

    void restoreOctree(const std::vector<unsigned char> &binary_tree_out_arg, OctreeCompressNode &root, size_t len) {
        std::queue<OctreeCompressNode *> nodeQueue;
        nodeQueue.push(&root);
        size_t index = 0;
        try {
            while (!nodeQueue.empty()) {
                OctreeCompressNode &node = *(nodeQueue.front());
                nodeQueue.pop();
                unsigned char mask = 1;
                if (index < len) {

                    // Octree mode
                    for (unsigned char i = 0; i < 8; i++) {
                        if (!!(binary_tree_out_arg[index] & mask)) {
                            node.children[i] = new OctreeCompressNode(node.depth + 1, 0, Point3f(0, 0, 0),
                                                                      Point3f(0, 0, 0), int(i), -1);
                            node.children[i]->parent = &node;
                            nodeQueue.push(node.children[i]);
                        }
                        mask = mask << 1;
                    }
                } else {
                    node.isLeaf = true;
                }
                index++;
            }
        }
        catch (std::bad_alloc) {

        }

    }

    void getPointRecurse(std::vector<Point3f> &restorePointCloud, unsigned long x_key, unsigned long y_key,
                         unsigned long z_key, Ptr<OctreeCompressNode> &_node, double resolution, Point3f ori) {
        OctreeCompressNode node = *_node;
        if (node.isLeaf) {
            restorePointCloud.emplace_back(
                    (float) (resolution * x_key) + (float) (resolution * 0.5) + ori.x,
                    (float) (resolution * y_key) + (float) (resolution * 0.5) + ori.y,
                    (float) (resolution * z_key) + (float) (resolution * 0.5) + ori.z);
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
                getPointRecurse(restorePointCloud, x_copy, y_copy, z_copy, node.children[i], resolution, ori);
            }
        }
    };

// Range Coding methods based on implementation by Julius Kammerl (julius@kammerl.de) from PCL
    void OctreeCompress::encodeCharVectorToStream(const std::vector<unsigned char> &inputCharVector_arg,
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

        // write encoded data to stream
        outputByteStream_arg.write(reinterpret_cast<const char *> (&outputCharVector_[0]), outputCharVector_.size());

    }

    void OctreeCompress::decodeStreamToCharVector(std::istream &inputByteStream_arg,
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

    void Haar3DRecursive(OctreeCompressNode *node, std::vector<Point3f> &haarCoefficients, size_t &N) {
        if (!node)
            return;
        if (node->isLeaf) {
            node->RAHTCoefficient = node->color;
            return;
        }

        for (const auto &child: node->children) {
            Haar3DRecursive(child, haarCoefficients, N);
        }

        std::vector<OctreeCompressNode *> prevCube(node->children.size());
        std::vector<OctreeCompressNode *> currCube(node->children.size() >> 1);

        // generate a new array and copy data from current node
        for (size_t idx = 0; idx < prevCube.size(); ++idx) {
            if (!node->children[idx]) {
                prevCube[idx] = nullptr;
                continue;
            }
            prevCube[idx] = new OctreeCompressNode;
            prevCube[idx]->RAHTCoefficient = node->children[idx]->RAHTCoefficient;
            prevCube[idx]->pointNum = node->children[idx]->pointNum;
        }

        size_t cubeSize = prevCube.size();
        size_t stepSize = 2;

        // start doing transform x then y then z
        while (true) {
            for (size_t x = 0; x < cubeSize; x += stepSize) {
                OctreeCompressNode *node1 = prevCube[x];
                OctreeCompressNode *node2 = prevCube[x + 1];

                if (!node1 && !node2) {
                    currCube[x / stepSize] = nullptr;
                    continue;
                }
                // transform under this condition
                if (node1 && node2) {
                    currCube[x / stepSize] = new OctreeCompressNode;
                    // TODO: Here the pointNum is not correctly matched with weight value in RAHT, need further fix
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

                    float YHighPass = a1 * node1->RAHTCoefficient.x - a2 * node2->RAHTCoefficient.x;
                    float UHighPass = a1 * node1->RAHTCoefficient.y - a2 * node2->RAHTCoefficient.y;
                    float VHighPass = a1 * node1->RAHTCoefficient.z - a2 * node2->RAHTCoefficient.z;

                    haarCoefficients[N++] = Point3f(YHighPass, UHighPass, VHighPass);

                    delete node1, delete node2;
                    continue;
                }
                // if no partner to transform, then directly use the value
                currCube[x / stepSize] = node1 ? node1 : node2;
            }

            cubeSize >>= 1;
            if (cubeSize < 2)
                break;

            // swap prevCube and currCube
            prevCube = currCube;
            currCube.resize(cubeSize >> 1);
        }

        // update selected node's coefficient in the octree
        node->RAHTCoefficient = currCube[0]->RAHTCoefficient;

        // free memory
        delete currCube[0];
    }

    // RAHT main - post-order traversal to generate RAHT coefficients in x,y,z directions
    // QStep: quantization step
    void OctreeCompress::encodeColor(std::vector<Point3f> &haarCoefficients, int QStep) {

        // measure time
//        auto start = std::chrono::high_resolution_clock::now();


        size_t N = 0;
        OctreeCompressNode *root = p->rootNode;

        size_t pointNum = root->pointNum;

        haarCoefficients.resize(pointNum);

        Haar3DRecursive(root, haarCoefficients, N);

        haarCoefficients[N] = root->RAHTCoefficient;

        // TODO: Apply Quantization, then encode coefficients

        // Quantization
        assert(QStep > 0);
        std::vector<Point3i> quantizedCoefficients(N);
        // store by: YYY UUU VVV
        for (size_t i = 0; i < N; ++i) {
            quantizedCoefficients[i].x = (int) round(haarCoefficients[i].x) / QStep;
            quantizedCoefficients[i].y = (int) round(haarCoefficients[i].y) / QStep;
            quantizedCoefficients[i].z = (int) round(haarCoefficients[i].z) / QStep;
        }

        // measure time
//        auto stop = std::chrono::high_resolution_clock::now();
//        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
//        std::cout << "Time taken by function: "
//             << duration.count() << " microseconds" << std::endl;

        std::ofstream myfile;
        myfile.open("/home/jeffery/Downloads/example.csv");
        for (size_t j = 0; j < quantizedCoefficients.size(); ++j) {
            myfile << quantizedCoefficients[j].x << "," << quantizedCoefficients[j].y << ","
                   << quantizedCoefficients[j].z << "," << "\n";
        }
        myfile.close();

        int a = 1;
    }



}
