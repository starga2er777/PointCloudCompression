// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html

#include "precomp.hpp"
#include "octree_compression.hpp"

#include <fstream>
#include <bitset>
#include <algorithm>

#define OCTREE_CHILD_NUM 8
#define OCTREE_NEIGH_SIZE 8

namespace cv{

    static bool _isPointInBound(const Point3f& _point, const Point3f& _origin, double _size);
    static bool insertPointRecurse( Ptr<OctreeCompressNode>& node,  const Point3f& point, int maxDepth, const OctreeCompressKey &key,size_t depthMask);
    void traverseByLevel(OctreeCompressData &raw_data_out, OctreeCompressNode &node);
    void restoreOctree(const std::vector<unsigned char> &bit_out,OctreeCompressNode &root, size_t len);
    void getPointRecurse(std::vector<Point3f> &restorePointCloud,unsigned long x_key,unsigned long y_key,unsigned long z_key, Ptr<OctreeCompressNode>& _node,double resolution,Point3f ori);
    void decodeStreamToCharVector(std::istream& inputByteStream_arg, std::vector<unsigned char>& outputCharVector_arg);
    void encodeCharVectorToStream(const std::vector<unsigned char>& inputCharVector_arg, std::ostream& outputByteStream_arg);


    OctreeCompressNode::OctreeCompressNode():children(OCTREE_CHILD_NUM, nullptr), neigh(OCTREE_NEIGH_SIZE, nullptr), depth(0), size(0), origin(0,0,0), parentIndex(-1), pointNum(0)
    {
    }

    OctreeCompressNode::OctreeCompressNode(int _depth, double _size, const Point3f& _origin, int _parentIndex, int _pointNum):children(OCTREE_CHILD_NUM), neigh(OCTREE_NEIGH_SIZE), depth(_depth), size(_size), origin(_origin), parentIndex(_parentIndex), pointNum(_pointNum)
    {
    }

    bool OctreeCompressNode::isPointInBound(const Point3f& _point) const
    {
        return isPointInBound(_point, origin, size);
    }

    bool OctreeCompressNode::isPointInBound(const Point3f& _point, const Point3f& _origin, double _size) const
    {
        return _isPointInBound(_point, _origin, _size);
    }

    bool _isPointInBound(const Point3f& _point, const Point3f& _origin, double _size)
    {
        float epsX = std::numeric_limits<float>::epsilon() * std::max(std::abs(_point.x), std::abs(_origin.x));
        float epsY = std::numeric_limits<float>::epsilon() * std::max(std::abs(_point.y), std::abs(_origin.y));
        float epsZ = std::numeric_limits<float>::epsilon() * std::max(std::abs(_point.z), std::abs(_origin.z));

        if((_point.x + epsX >= _origin.x && _point.y + epsY >= _origin.y && _point.z + epsZ >= _origin.z) &&
           (_point.x <= _origin.x + _size + epsX && _point.y <= _origin.y + _size + epsY && _point.z <= _origin.z + _size + epsZ))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    struct OctreeCompress::Impl
    {
    public:
        Impl():maxDepth(-1), size(0), origin(0,0,0),resolution(0),depthMask(0)
        {}

        ~Impl()
        {}

        // The pointer to Octree root node.
        Ptr <OctreeCompressNode> rootNode = nullptr;
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

    OctreeCompress::OctreeCompress() : p(new Impl)
    {
        p->maxDepth = -1;
        p->size = 0;
        p->origin = Point3f(0,0,0);
    }

    OctreeCompress::OctreeCompress(int _maxDepth, double _size, const Point3f& _origin ) : p(new Impl)
    {
        p->maxDepth = _maxDepth;
        p->size = _size;
        p->origin = _origin;
    }

    OctreeCompress::OctreeCompress(const std::vector<Point3f>& _pointCloud, int _maxDepth) : p(new Impl)
    {
        CV_Assert( _maxDepth > -1 );
        this->create(_pointCloud, _maxDepth);
    }

    OctreeCompress::OctreeCompress(int _maxDepth) : p(new Impl)
    {
        p->maxDepth = _maxDepth;
        p->size = 0;
        p->origin = Point3f(0,0,0);
    }

    OctreeCompress::~OctreeCompress(){}

    bool OctreeCompress::insertPoint(const Point3f& point,double resolution,size_t depthMask)
    {
        if(p->rootNode.empty())
        {
            p->rootNode = new OctreeCompressNode( 0, p->size, p->origin, -1, 0);
        }

        OctreeCompressKey key(floor((point.x-this->p->origin.x)/resolution),
                              floor((point.y-this->p->origin.y)/resolution),
                              floor((point.z-this->p->origin.z)/resolution));
        //std::cout<<"x="<<key.x_key<<" y="<<key.y_key<<" z="<<key.z_key<<std::endl;

        bool result = insertPointRecurse(p->rootNode, point, p->maxDepth,key,depthMask);
        p->rootNode->pointNum += result;
        return result;
    }


    bool OctreeCompress::create(const std::vector<Point3f> &pointCloud, double resolution)
    {
        if(resolution > 0)
        {
            p->resolution=resolution;
        }

        //CV_Assert( p->maxDepth > -1 && !pointCloud.empty());

        if(pointCloud.empty())
            return false;

        Point3f maxBound(pointCloud[0]);
        Point3f minBound(pointCloud[0]);
        Point3f center, temp;

        // Find center coordinate of PointCloud data.
        for(size_t idx = 0; idx <pointCloud.size(); idx++)
        {
            maxBound.x = max(pointCloud[idx].x, maxBound.x);
            maxBound.y = max(pointCloud[idx].y, maxBound.y);
            maxBound.z = max(pointCloud[idx].z, maxBound.z);

            minBound.x = min(pointCloud[idx].x, minBound.x);
            minBound.y = min(pointCloud[idx].y, minBound.y);
            minBound.z = min(pointCloud[idx].z, minBound.z);
        }

        double maxSize=max(max(maxBound.x-minBound.x,maxBound.y-minBound.y),maxBound.z-minBound.z);
        p->maxDepth= ceil(log2(maxSize/resolution));
        p->depthMask=1<<(p->maxDepth-1);


        center = (maxBound + minBound) * 0.5f;

        temp = center - minBound;
        float halfSize = std::max(temp.x, std::max(temp.y, temp.z));
        this->p->origin =Point3f (float(floor(minBound.x/resolution)*resolution),
                                  float(floor(minBound.y/resolution)*resolution),
                                  float(floor(minBound.z/resolution)*resolution));

        this->p->size = 2 * halfSize;

        // Insert every point in PointCloud data.
        int cnt=0;
        for(size_t idx = 0; idx < pointCloud.size(); idx++ )
        {
            if(!insertPoint(pointCloud[idx],resolution,p->depthMask)){
                cnt++;
            }
        }

        CV_LOG_IF_WARNING(NULL,cnt!=0,"OverAll "<<cnt<<" points has been ignored! The number of point clouds contained in the current octree is "<<pointCloud.size()-cnt);
        return true;
    }

    void OctreeCompress::traverse(std::ostream& outputStream){
        OctreeCompressData raw_data;

        // traverse Octree nodes
        traverseByLevel(raw_data, *p->rootNode);

        // Set header
        // +-----------------------------   +
        // | resolution (double, 8bytes)    |
        // | origin X,Y,Z (3float, 12bytes) |
        // | occ_codes len (size_t, 8bytes) |
        // | dcm_flags len (size_t, 8bytes) |
        // | dcm_codes len (size_t, 8bytes) |
        // +-----------------------------   +
        //TODO use sizeof() instead of hard coding size
        unsigned char res[8];
        unsigned char ori[12];
        unsigned char len_occ_codes[8];
        unsigned char len_dcm_flags[8];
        unsigned char len_dcm_codes[8];

        size_t occ_codes_size = raw_data.occ_codes.size();
        size_t dcm_flags_size = raw_data.dcm_flags.size();
        size_t dcm_codes_size = raw_data.dcm_codes.size();

        memcpy(&res, &(p->resolution), sizeof(p->resolution));
        memcpy(ori,&p->origin.x,sizeof(ori)/3);
        memcpy(&ori[4],&p->origin.y,sizeof(ori)/3);
        memcpy(&ori[8],&p->origin.z,sizeof(ori)/3);
        memcpy(len_occ_codes,&occ_codes_size,sizeof(len_occ_codes));
        memcpy(len_dcm_flags,&dcm_flags_size,sizeof(len_dcm_flags));
        memcpy(len_dcm_codes,&dcm_codes_size,sizeof(len_dcm_codes));

        // push additional info
        for(unsigned char & i : res){
            raw_data.header.push_back(i);
        }
        for(unsigned char & i : ori){
            raw_data.header.push_back(i);
        }
        for(unsigned char & i : len_occ_codes){
            raw_data.header.push_back(i);
        }
        for(unsigned char & i : len_dcm_flags){
            raw_data.header.push_back(i);
        }
        for(unsigned char & i : len_dcm_codes){
            raw_data.header.push_back(i);
        }

        encodeCharVectorToStream(raw_data.header,outputStream);
        encodeCharVectorToStream(raw_data.occ_codes,outputStream);
        encodeCharVectorToStream(raw_data.dcm_flags,outputStream);
        encodeCharVectorToStream(raw_data.dcm_codes,outputStream);
        encodeCharVectorToStream(raw_data.occ_lookup_table,outputStream);
        encodeCharVectorToStream(raw_data.dcm_lookup_table,outputStream);
    }

    void OctreeCompress::reStore(const std::vector<unsigned char> &bit_out){

        this->clear();
        size_t bit_out_len=bit_out.size();
        size_t len;
        Point3f ori(0,0,0);
        memcpy(&len,&bit_out[bit_out_len-8],sizeof(len));
        memcpy(&(p->resolution),&bit_out[len],sizeof(double));
        memcpy(&ori.x,&bit_out[len+8],sizeof(float));
        memcpy(&ori.y,&bit_out[len+8+4],sizeof(float));
        memcpy(&ori.z,&bit_out[len+8+8],sizeof(float));
        p->rootNode = new OctreeCompressNode( 0, p->size, Point3f (0,0,0), -1, 0);
        p->origin=ori;
        restoreOctree(bit_out,*p->rootNode, len);
    }

    void OctreeCompress::getPointCloudByOctree(std::vector<Point3f> &restorePointCloud){
        Ptr <OctreeCompressNode> root=p->rootNode;
        double resolution=p->resolution;

        getPointRecurse(restorePointCloud,0,0,0,root,resolution,p->origin);
    }

    void OctreeCompress::setMaxDepth(int _maxDepth)
    {
        if(_maxDepth )
            this->p->maxDepth = _maxDepth;
    }

    void OctreeCompress::setSize(double _size)
    {
        this->p->size = _size;
    };

    void OctreeCompress::setOrigin(const Point3f& _origin)
    {
        this->p->origin = _origin;
    }

    void OctreeCompress::clear()
    {
        if(!p->rootNode.empty())
        {
            p->rootNode.release();
        }

        p->size = 0;
        p->maxDepth = -1;
        p->origin = Point3f (0,0,0); // origin coordinate
        p->resolution=0;
        p->depthMask=0;
    }

    bool OctreeCompress::empty() const
    {
        return p->rootNode.empty();
    }

    bool OctreeCompress::isPointInBound(const Point3f& _point) const
    {
        return _isPointInBound(_point, p->origin, p->size);
    }

    bool insertPointRecurse( Ptr<OctreeCompressNode>& _node,  const Point3f& point, int maxDepth, const OctreeCompressKey &key, size_t depthMask)
    {
        OctreeCompressNode& node = *_node;

        if(node.depth == maxDepth)
        {
            if (node.pointNum == 0) {
                node.isLeaf = true;
                node.pointNum++;
                return true;
            }
            return false;
        }

        size_t childIndex= key.findChildIdxByMask(depthMask);
        if(node.children[childIndex].empty())
        {
            node.children[childIndex] = new OctreeCompressNode(node.depth + 1,0, Point3f(0,0,0), int(childIndex), 0);
            node.children[childIndex]->parent = _node;
        }

        bool result = insertPointRecurse(node.children[childIndex], point, maxDepth, key,depthMask>>1);
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
            neigh_parent_idx = (unsigned char)node.parentIndex ^ dim_mask;
            neigh_6N_idx = mapNeighPos2Idx(dim_mask, (unsigned char)sign);

            if (bool(dim_mask & (unsigned char)node.parentIndex) == sign) {
                // neigh inside curr node's parent's neighbour
                if (!node.parent->neigh[neigh_6N_idx]) {
                    node.neigh[neigh_6N_idx]= nullptr;
                }
                else {
                    node.neigh[neigh_6N_idx] = node.parent->neigh[neigh_6N_idx]->children[neigh_parent_idx];
                }
            }
            else {
                // neigh share same parent with curr node
                node.neigh[neigh_6N_idx] = node.parent->children[neigh_parent_idx];
            }

            // next neighbours position
            if (sign) {
                dim_mask <<= 1;
                sign = false;
            }
            else sign = true;
        }
    }

    void traverseByLevel(OctreeCompressData &raw_data_out,
                         OctreeCompressNode &root){

        try{
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
            std::queue<OctreeCompressNode*> nodeQueue;
            nodeQueue.push(&root);

            while (!nodeQueue.empty()) {

                OctreeCompressNode &node = *(nodeQueue.front());
                nodeQueue.pop();

                // Stop at last leaf level, no need to encode leaf node
                if (node.isLeaf) {
                    break;
                }

                // set neighbours for all children
                for (int i = 0; i < 8; i++) {
                    if (!node.children[i]) continue;
                    setNeighbour(*(node.children[i]));
                }

                // Direct coding mode (DCM) for lookup

                if (node.parent != nullptr) {
                    // check if it is single point
                    if (node.pointNum == 1){

                        // record DCM branching, refer to neighbour
                        unsigned char neigh = OctreeCompressKey::getNeighPattern(node);
                        OctreeCompressNode pNode = node;
                        while (!pNode.isLeaf) {
                            for (unsigned char i=0; i<pNode.children.size();i++){
                                if (!pNode.children[i].empty()) {
                                    look_up_DCM_table[neigh][i]++;
                                    pNode = *(pNode.children[i]);
                                    break;
                                }
                            }
                        }
                        // DCM complete
                        continue;
                    }
                }

                // Octree mode (further branching)
                for (unsigned char i = 0; i < 8; i++) {
                    if (!node.children[i].empty()) {
                        nodeQueue.push(node.children[i]);
                    }
                }
                // record Octree branching, refer to neighbour
                look_up_table[(int) OctreeCompressKey::getNeighPattern(node)][(int) OctreeCompressKey::getBitPattern(node)]++;
            }

            // Sort lookup to get weight table
            for (int i = 0; i < 64; i++)
            {
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
                    weight_table[i][temp_vec[w].second] = (unsigned char)w;
                    raw_data_out.occ_lookup_table.push_back((unsigned char)(temp_vec[w].second));
                }
                for (int w = 0; w < 8; w++) {
                    weight_DCM_table[i][temp_DCM_vec[w].second] = (unsigned char)w;
                    raw_data_out.dcm_lookup_table.push_back((unsigned char)(temp_DCM_vec[w].second));
                }
            }

            // True Traverse
            nodeQueue = std::queue<OctreeCompressNode*>();
            nodeQueue.push(&root);

            while (!nodeQueue.empty()) {

                OctreeCompressNode &node = *(nodeQueue.front());
                nodeQueue.pop();

                // Stop at last leaf level, no need to encode leaf node
                if (node.isLeaf) {
                    break;
                }

                // neigh: the core of prediction
                unsigned char neigh = OctreeCompressKey::getNeighPattern(node);

                // Direct coding mode (DCM)

                if (node.parent != nullptr) {
                    // eligible: check if parent only have one child
                    bool eligible = false;

                    for (unsigned char i=0; i<node.children.size();i++){
                        if (!node.children[i].empty()) {
                            if (eligible) {
                                eligible = false;
                                break;
                            }
                            eligible = true;
                        }
                    }

                    if (eligible){
                        if (node.pointNum == 1){
                            raw_data_out.dcm_flags.push_back(true);

                            // Apply DCM
                            OctreeCompressNode pNode = node;
                            while (!pNode.isLeaf) {
                                for (unsigned char i=0; i<pNode.children.size();i++){
                                    if (!pNode.children[i].empty()) {
                                        // push back DCM weighted Prediction loss to output vector
                                        raw_data_out.dcm_codes.push_back(weight_DCM_table[neigh][i]);
                                        pNode = *(pNode.children[i]);
                                        break;
                                    }
                                }
                            }
                            continue;
                        }
                        else {
                            raw_data_out.dcm_flags.push_back(false);
                        }
                    }
                }

                // Octree mode (further branching)
                // push back weighted Prediction loss to output vector
                raw_data_out.occ_codes.push_back(weight_table[neigh][OctreeCompressKey::getBitPattern(node)]);
                for (unsigned char i = 0; i < 8; i++) {
                    if (!node.children[i].empty()) {
                        nodeQueue.push(node.children[i]);
                    }
                }
            }
        }
        catch (std::bad_alloc){

        }

    }


    void restoreOctree(const std::vector<unsigned char> &binary_tree_out_arg,OctreeCompressNode &root, size_t len){
        std::queue<OctreeCompressNode *> nodeQueue;
        nodeQueue.push(&root);
        size_t index=0;
        try{
            while(!nodeQueue.empty()){
                OctreeCompressNode &node=*(nodeQueue.front());
                nodeQueue.pop();
                unsigned char mask=1;
                if (index < len){

                    // Octree mode
                    for(unsigned char i=0;i<8;i++){
                        if(!!(binary_tree_out_arg[index]&mask)){
                            node.children[i] = new OctreeCompressNode(node.depth + 1, 0, Point3f(0, 0, 0), int(i), -1);
                            node.children[i]->parent = &node;
                            nodeQueue.push(node.children[i]);
                        }
                        mask=mask<<1;
                    }
                }
                else{
                    node.isLeaf=true;
                }
                index++;
            }
        }
        catch(std::bad_alloc){

        }

    }

    void getPointRecurse(std::vector<Point3f> &restorePointCloud,unsigned long x_key,unsigned long y_key,unsigned long z_key, Ptr<OctreeCompressNode>& _node,double resolution,Point3f ori){
        OctreeCompressNode node=*_node;
        if(node.isLeaf){
            restorePointCloud.emplace_back(
                    (float)(resolution*x_key)+(float)(resolution*0.5)+ori.x,
                    (float)(resolution*y_key)+(float)(resolution*0.5)+ori.y,
                    (float)(resolution*z_key)+(float)(resolution*0.5)+ori.z);
            return;
        }
        unsigned char x_mask=1;
        unsigned char y_mask=2;
        unsigned char z_mask=4;
        for(unsigned char i=0;i<8;i++){
            unsigned long x_copy=x_key;
            unsigned long y_copy=y_key;
            unsigned long z_copy=z_key;
            if(!node.children[i].empty()){
                size_t x_offSet=!!(x_mask&i);
                size_t y_offSet=!!(y_mask&i);
                size_t z_offSet=!!(z_mask&i);
                x_copy=(x_copy<<1)|x_offSet;
                y_copy=(y_copy<<1)|y_offSet;
                z_copy=(z_copy<<1)|z_offSet;
                getPointRecurse(restorePointCloud,x_copy,y_copy,z_copy,node.children[i],resolution,ori);
            }
        }
    };

// Range Coding methods based on implementation by Julius Kammerl (julius@kammerl.de) from PCL
    void encodeCharVectorToStream(const std::vector<unsigned char>& inputCharVector_arg,
                                           std::ostream& outputByteStream_arg) {

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
        while (readPos < input_size)
        {
            symbol = static_cast<std::uint8_t> (inputCharVector_arg[readPos++]);
            hist[symbol + 1]++;
        }
        part_idx[0] = 0;
        for (int i = 1; i <= 256; i++)
        {
            // partition must have at least 1 space for each symbol
            if (hist[i] <= 0) {
                part_idx[i] = part_idx[i - 1] + 1;
                continue;
            }
            part_idx[i] = part_idx[i - 1] + static_cast<std::uint32_t> (hist[i]);
        }

        // rescale if least partition range exceeds bottom_limit
        while (part_idx[256] >= bottom_limit)
        {
            for (int i = 1; i <= 256; i++)
            {
                part_idx[i] >>= 1;
                if (part_idx[i] <= part_idx[i - 1]) {
                    part_idx[i] = part_idx[i - 1] + 1;
                }
            }
        }

        // write cumulative frequency table to output stream
        outputByteStream_arg.write(reinterpret_cast<const char*> (&part_idx[0]), sizeof(part_idx));

        // start encoding, range initialize to maximum
        readPos = 0;
        low = 0;
        range = static_cast<std::uint32_t> (-1);

        while (readPos < input_size)
        {
            // read symbol
            symbol = static_cast<std::uint8_t>(inputCharVector_arg[readPos++]);

            // map to range
            low += part_idx[symbol] * (range /= part_idx[256]);
            range *= part_idx[symbol + 1] - part_idx[symbol];

            // renormalization
            // first case: range is completely inside a block of size adjust_limit
            // second case: range is too small while first case continuously miss, preform resize to bottom_limit
            while ((low ^ (low + range)) < adjust_limit || ((range < bottom_limit) && ((range = -int(low) & (bottom_limit - 1)), 1)))
            {
                unsigned char out = static_cast<unsigned char> (low >> 24);
                range <<= 8;
                low <<= 8;
                outputCharVector_.push_back(out);
            }

        }

        // flush remaining data
        for (int i = 0; i < 4; i++)
        {
            unsigned char out = static_cast<unsigned char> (low >> 24);
            outputCharVector_.push_back(out);
            low <<= 8;
        }

        // write encoded data to stream
        outputByteStream_arg.write(reinterpret_cast<const char*> (&outputCharVector_[0]), outputCharVector_.size());

    }

    void decodeStreamToCharVector(std::istream& inputByteStream_arg,
                                           std::vector<unsigned char>& outputCharVector_arg)
    {

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
        inputByteStream_arg.read(reinterpret_cast<char*> (&part_idx[0]), sizeof(part_idx));

        // read code
        code = 0;
        for (size_t i = 0; i < 4; i++)
        {
            std::uint8_t out;
            inputByteStream_arg.read(reinterpret_cast<char*> (&out), sizeof(unsigned char));
            code = (code << 8) | out;
        }

        low = 0;
        range = static_cast<std::uint32_t> (-1);

        // decoding
        for (size_t i = 0; i < output_size; i++)
        {
            // symbol lookup in cumulative frequency table
            std::uint32_t count = (code - low) / (range /= part_idx[256]);

            // finding symbol using Jump search
            std::uint8_t symbol = 0;
            std::uint8_t step = 128;
            while (step > 0)
            {
                if (part_idx[symbol + step] <= count)
                {
                    symbol = static_cast<std::uint8_t> (symbol + step);
                }
                step /= 2;
            }

            // write symbol to output stream
            outputCharVector_arg[outputPos++] = symbol;

            low += part_idx[symbol] * range;
            range *= part_idx[symbol + 1] - part_idx[symbol];

            // check range limits
            while ((low ^ (low + range)) < adjust_limit || ((range < bottom_limit) && ((range = -int(low) & (bottom_limit - 1)), 1)))
            {
                std::uint8_t out;
                inputByteStream_arg.read(reinterpret_cast<char*> (&out), sizeof(unsigned char));
                code = code << 8 | out;
                range <<= 8;
                low <<= 8;
            }

        }

    }

}
