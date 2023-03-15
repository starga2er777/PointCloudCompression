#ifndef OPENCV_OCTREE_KEY_H
#define OPENCV_OCTREE_KEY_H

namespace cv{
    class OctreeKey{
    public:
        size_t x_key;
        size_t y_key;
        size_t z_key;

    public:
        OctreeKey():x_key(0),y_key(0),z_key(0){};
        OctreeKey(size_t x,size_t y,size_t z):x_key(x),y_key(y),z_key(z){};
        inline unsigned char findChildIdxByMask(size_t mask) const{
            return static_cast<unsigned char>((!!(z_key&mask))<<2)|((!!(y_key&mask))<<1)|(!!(x_key&mask));
        }

        static inline unsigned char getBitPattern(OctreeNode &node) {
            unsigned char res=0;
            for (unsigned char i=0; i<node.children.size();i++){
                res|=static_cast<unsigned char>((!node.children[i].empty()) << i);
            }
            return res;
        }
    };
}

#endif //OPENCV_OCTREE_KEY_H
