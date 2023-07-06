// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html

#include "precomp.hpp"
#include "octree.hpp"
#include "opencv2/3d.hpp"


#define OCTREE_CHILD_NUM 8
#define OCTREE_NEIGH_SIZE 8

namespace cv{

// Locate the OctreeNode corresponding to the input point from the given OctreeNode.
static Ptr<OctreeNode> index(const Point3f& point, Ptr<OctreeNode>& node,OctreeKey& key,size_t depthMask);

static bool _isPointInBound(const Point3f& _point, const Point3f& _origin, double _size);
static bool insertPointRecurse( Ptr<OctreeNode>& node, const Point3f& point, const Point3f &color, int maxDepth
                                ,const OctreeKey &key, size_t depthMask);
bool deletePointRecurse( Ptr<OctreeNode>& node);

// For Nearest neighbor search.
template<typename T> struct PQueueElem; // Priority queue
static void radiusNNSearchRecurse(const Ptr<OctreeNode>& node, const Point3f& query, float squareRadius,
                           std::vector<PQueueElem<Point3f> >& candidatePoint);
static void KNNSearchRecurse(const Ptr<OctreeNode>& node, const Point3f& query, const int K,
                      float& smallestDist, std::vector<PQueueElem<Point3f> >& candidatePoint);

OctreeNode::OctreeNode():children(OCTREE_CHILD_NUM, nullptr),neigh(OCTREE_NEIGH_SIZE, nullptr), depth(0), size(0), origin(0,0,0), parentIndex(-1),pointNum(0)
{
}

OctreeNode::OctreeNode(int _depth, double _size, const Point3f &_origin, const Point3f &_color,
                       int _parentIndex, int _pointNum) : children(OCTREE_CHILD_NUM),
                                                          neigh(OCTREE_NEIGH_SIZE), depth(_depth),
                                                          size(_size), origin(_origin),
                                                          color(_color), parentIndex(_parentIndex),
                                                          pointNum(_pointNum) {
}

bool OctreeNode::empty() const
{
    if(this->isLeaf)
    {
        if(this->pointList.empty())
            return true;
        else
            return false;
    }
    else
    {
        for(size_t i = 0; i< OCTREE_CHILD_NUM; i++)
        {
            if(!this->children[i].empty())
            {
                return false;
            }
        }
        return true;
    }
}

bool OctreeNode::isPointInBound(const Point3f& _point) const
{
    return isPointInBound(_point, origin, size);
}

bool OctreeNode::isPointInBound(const Point3f& _point, const Point3f& _origin, double _size) const
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

struct Octree::Impl
{
public:
    Impl():maxDepth(-1), size(0), origin(0,0,0),resolution(0)
    {}

    ~Impl()
    {}

    //! The pointer to Octree root node.
    Ptr <OctreeNode> rootNode = nullptr;
    //! Max depth of the Octree. And depth must be greater than zero
    int maxDepth;
    //! The size of the cube of the .
    double size;
    //! The origin coordinate of root node.
    Point3f origin;
    //! The size of the leaf node.
    double resolution;
    //! Whether the point cloud has a color attribute.
    bool hasColor{};
};

Octree::Octree() : p(new Impl)
{
    p->maxDepth = -1;
    p->size = 0;
    p->origin = Point3f(0,0,0);
}

Octree::Octree(int _maxDepth, double _size, const Point3f& _origin ) : p(new Impl)
{
    p->maxDepth = _maxDepth;
    p->size = _size;
    p->origin = _origin;
}

Octree::Octree(const std::vector<Point3f>& _pointCloud, double resolution) : p(new Impl)
{
    std::vector<Point3f> v;
    this->create(_pointCloud,v, resolution);
}

Octree::Octree(int _maxDepth) : p(new Impl)
{
    p->maxDepth = _maxDepth;
    p->size = 0;
    p->origin = Point3f(0,0,0);
}

Octree::~Octree(){}

bool Octree::insertPoint(const Point3f& point,const Point3f &color)
{
    double resolution=p->resolution;
    size_t depthMask=1 << (p->maxDepth - 1);
    if(p->rootNode.empty())
    {
        p->rootNode = new OctreeNode( 0, p->size, p->origin,  color, -1, 0);
    }
    bool pointInBoundFlag = p->rootNode->isPointInBound(point, p->rootNode->origin, p->rootNode->size);
    if(p->rootNode->depth==0 && !pointInBoundFlag)
    {
        CV_Error(Error::StsBadArg, "The point is out of boundary!");
    }
    OctreeKey key(floor((point.x - this->p->origin.x) / resolution),
                          floor((point.y - this->p->origin.y) / resolution),
                          floor((point.z - this->p->origin.z) / resolution));

    bool result = insertPointRecurse(p->rootNode, point, color, p->maxDepth, key, depthMask);
    return result;
}


bool Octree::create(const std::vector<Point3f> &pointCloud, const std::vector<Point3f> &colorAttribute, double resolution)
{

    if (resolution > 0) {
        p->resolution = resolution;
    }
    else{
        CV_Error(Error::StsBadArg, "The resolution must be greater than 0!");
    }

    if (pointCloud.empty())
        return false;

    Point3f maxBound(pointCloud[0]);
    Point3f minBound(pointCloud[0]);

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
    maxSize=double(1<<int(ceil(log2(maxSize))));
    p->maxDepth = ceil(log2(maxSize / resolution));
    this->p->size = (1<<p->maxDepth)*resolution;
    this->p->origin = Point3f(float(floor(minBound.x / resolution) * resolution),
                              float(floor(minBound.y / resolution) * resolution),
                              float(floor(minBound.z / resolution) * resolution));

    p->hasColor = !colorAttribute.empty();

    // Insert every point in PointCloud data.
    int cnt = 0;
    for (size_t idx = 0; idx < pointCloud.size(); idx++) {
        Point3f insertColor = p->hasColor ? colorAttribute[idx] : Point3f(0.0f, 0.0f, 0.0f);
        if (!insertPoint(pointCloud[idx], insertColor)) {
            cnt++;
        }
    }

    CV_LOG_IF_WARNING(nullptr, cnt != 0, "OverAll " << cnt
                                                    << " points has been ignored! The number of point clouds contained in the current octree is "
                                                    << pointCloud.size() - cnt)
    return true;
}

bool Octree::create(const std::vector<Point3f> &pointCloud, double resolution) {
    std::vector<Point3f> v;
    return this->create(pointCloud, v, resolution);
}

void Octree::setMaxDepth(int _maxDepth)
{
    if(_maxDepth )
        this->p->maxDepth = _maxDepth;
}

void Octree::setSize(double _size)
{
    this->p->size = _size;
};

void Octree::setOrigin(const Point3f& _origin)
{
    this->p->origin = _origin;
}

void Octree::clear()
{
    if(!p->rootNode.empty())
    {
        p->rootNode.release();
    }

    p->size = 0;
    p->maxDepth = -1;
    p->origin = Point3f (0,0,0); // origin coordinate
}

bool Octree::empty() const
{
    return p->rootNode.empty();
}

Ptr<OctreeNode> index(const Point3f& point, Ptr<OctreeNode>& _node,OctreeKey &key,size_t depthMask)
{
    OctreeNode &node = *_node;

    if(node.empty())
    {
        return Ptr<OctreeNode>();
    }

    if(node.isLeaf)
    {
        for(size_t i = 0; i < node.pointList.size(); i++ )
        {
            if((point.x == node.pointList[i].x) &&
               (point.y == node.pointList[i].y) &&
               (point.z == node.pointList[i].z)
                    )
            {
                return _node;
            }
        }
        return Ptr<OctreeNode>();
    }


    size_t childIndex = key.findChildIdxByMask(depthMask);
    if(!node.children[childIndex].empty())
    {
        return index(point, node.children[childIndex],key,depthMask>>1);
    }
    return Ptr<OctreeNode>();
}

bool Octree::isPointInBound(const Point3f& _point) const
{
    return _isPointInBound(_point, p->origin, p->size);
}

bool Octree::deletePoint(const Point3f& point)
{
    OctreeKey key=OctreeKey(floor((point.x - this->p->origin.x) / p->resolution),
                               floor((point.y - this->p->origin.y) / p->resolution),
                               floor((point.z - this->p->origin.z) / p->resolution));
    size_t depthMask=1 << (p->maxDepth - 1);
    Ptr<OctreeNode> node = index(point, p->rootNode,key,depthMask);

    if(!node.empty())
    {
        size_t i = 0;
        while (!node->pointList.empty() && i < node->pointList.size() )
        {
            if((point.x == node->pointList[i].x) &&
               (point.y == node->pointList[i].y) &&
               (point.z == node->pointList[i].z)
                    )
            {
                node->pointList.erase(node->pointList.begin() + i);
            } else{
                i++;
            }
        }

        // If it is the last point cloud in the OctreeNode, recursively delete the node.
        return deletePointRecurse(node);
    }
    else
    {
        return false;
    }
}

bool deletePointRecurse(Ptr<OctreeNode>& _node)
{
    OctreeNode& node = *_node;

    if(_node.empty())
        return false;

    if(node.isLeaf)
    {
        if( !node.pointList.empty())
        {
            Ptr<OctreeNode> parent = node.parent;
            parent->children[node.parentIndex] = nullptr;
            _node.release();

            return deletePointRecurse(parent);
        }
        else
        {
            return true;
        }
    }
    else
    {
        bool deleteFlag = true;

        // Only all children was deleted, can we delete the tree node.
        for(size_t i = 0; i< OCTREE_CHILD_NUM; i++)
        {
            if(!node.children[i].empty())
            {
                deleteFlag = false;
                break;
            }
        }

        if(deleteFlag)
        {
            Ptr<OctreeNode> parent = node.parent;
            _node.release();
            return deletePointRecurse(parent);
        }
        else
        {
            return true;
        }
    }
}

bool insertPointRecurse( Ptr<OctreeNode>& _node,  const Point3f& point,const Point3f &color, int maxDepth,const OctreeKey &key,
                         size_t depthMask)
{
    OctreeNode &node = *_node;
    if (node.depth == maxDepth) {
        node.isLeaf = true;
        node.color = color;
        node.pointNum++;
        node.pointList.push_back(point);
        return true;
    }

    double childSize = node.size * 0.5;
    size_t childIndex = key.findChildIdxByMask(depthMask);
    size_t xIndex=childIndex&1?1:0;
    size_t yIndex=childIndex&2?1:0;
    size_t zIndex=childIndex&4?1:0;
    Point3f childOrigin = node.origin + Point3f(xIndex * float(childSize), yIndex * float(childSize), zIndex * float(childSize));

    if (node.children[childIndex].empty()) {

        node.children[childIndex] = new OctreeNode(node.depth + 1, childSize, childOrigin, Point3f(0, 0, 0),
                                                           int(childIndex), 0);
        node.children[childIndex]->parent = _node;
    }

    bool result = insertPointRecurse(node.children[childIndex], point, color, maxDepth, key, depthMask >> 1);
    node.pointNum += result;
    return result;
}

// For Nearest neighbor search.
template<typename T>
struct PQueueElem
{
    PQueueElem() : dist(0), t(0) {}
    PQueueElem(float _dist, T _t) : dist(_dist), t(_t) {}
    float dist;
    T t;

    bool
    operator<(const PQueueElem<T> p1) const
    {
        return (this->dist < p1.dist);
    }
};

static float SquaredDistance(const Point3f& query, const Point3f& origin)
{
    Point3f diff = query - origin;
    return diff.dot(diff);
}

static bool overlap(const OctreeNode& node, const Point3f& query, float squareRadius)
{
    float halfSize = float(node.size * 0.5);
    Point3f center = node.origin + Point3f( halfSize, halfSize, halfSize );

    float dist = SquaredDistance(center, query);
    float temp = float(node.size) * float(node.size) * 3.0f;

    return ( dist + dist * std::numeric_limits<float>::epsilon() ) <= float(temp * 0.25f + squareRadius + sqrt(temp * squareRadius)) ;
}

void radiusNNSearchRecurse(const Ptr<OctreeNode>& node, const Point3f& query, float squareRadius,
                           std::vector<PQueueElem<Point3f> >& candidatePoint)
{
    float dist;
    Ptr<OctreeNode> child;

    // iterate eight children.
    for(size_t i = 0; i< OCTREE_CHILD_NUM; i++)
    {
        if( !node->children[i].empty()&& overlap(*node->children[i], query, squareRadius))
        {
            if(!node->children[i]->isLeaf)
            {
                // Reach the branch node.
                radiusNNSearchRecurse(node->children[i], query, squareRadius, candidatePoint);
            }
            else
            {
                // Reach the leaf node.
                child = node->children[i];

                for(size_t j = 0; j < child->pointList.size(); j++)
                {
                    if(abs(child->pointList[j].x-(-8.88461112976f))<0.001){
                        int kkk=0;
                    }
                    dist = SquaredDistance(child->pointList[j], query);
                    if(dist + dist * std::numeric_limits<float>::epsilon() <= squareRadius )
                    {
                        candidatePoint.emplace_back(dist, child->pointList[j]);
                    }
                }
            }
        }
    }
}

int Octree::radiusNNSearch(const Point3f& query, float radius,
                           std::vector<Point3f>& pointSet, std::vector<float>& squareDistSet) const
{
    if(p->rootNode.empty())
        return 0;
    float squareRadius = radius * radius;

    PQueueElem<Point3f> elem;
    std::vector<PQueueElem<Point3f> > candidatePoint;

    radiusNNSearchRecurse(p->rootNode, query, squareRadius, candidatePoint);

    for(size_t i = 0; i < candidatePoint.size(); i++)
    {
        pointSet.push_back(candidatePoint[i].t);
        squareDistSet.push_back(candidatePoint[i].dist);
    }
    return int(pointSet.size());
}

void KNNSearchRecurse(const Ptr<OctreeNode>& node, const Point3f& query, const int K,
                      float& smallestDist, std::vector<PQueueElem<Point3f> >& candidatePoint)
{
    std::vector<PQueueElem<int> > priorityQue;
    Ptr<OctreeNode> child;
    float dist = 0;
    Point3f center; // the OctreeNode Center

    // Add the non-empty OctreeNode to priorityQue.
    for(size_t i = 0; i < OCTREE_CHILD_NUM; i++)
    {
        if(!node->children[i].empty())
        {
            float halfSize = float(node->children[i]->size * 0.5);

            center = node->children[i]->origin + Point3f(halfSize, halfSize, halfSize);

            dist = SquaredDistance(query, center);
            priorityQue.emplace_back(dist, int(i));
        }
    }

    std::sort(priorityQue.rbegin(), priorityQue.rend());
    child = node->children[priorityQue.back().t];

    while (!priorityQue.empty()&&overlap(*child, query, smallestDist))
    {
        if (!child->isLeaf)
        {
            KNNSearchRecurse(child, query, K, smallestDist, candidatePoint);
        } else {
            for (size_t i = 0; i < child->pointList.size(); i++) {
                dist = SquaredDistance(child->pointList[i], query);

                if ( dist + dist * std::numeric_limits<float>::epsilon() <= smallestDist ) {
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
        if(!priorityQue.empty())
            child = node->children[priorityQue.back().t];
    }
}

void Octree::KNNSearch(const Point3f& query, const int K, std::vector<Point3f>& pointSet, std::vector<float>& squareDistSet) const
{
    if(p->rootNode.empty())
        return;

    PQueueElem<Ptr<Point3f> > elem;
    std::vector<PQueueElem<Point3f> > candidatePoint;
    float smallestDist = std::numeric_limits<float>::max();

    KNNSearchRecurse(p->rootNode, query, K, smallestDist, candidatePoint);

    for(size_t i = 0; i < candidatePoint.size(); i++)
    {
        pointSet.push_back(candidatePoint[i].t);
        squareDistSet.push_back(candidatePoint[i].dist);
    }
}

}