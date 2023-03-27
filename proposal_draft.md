# Google Summer of Code 2023: OpenCV Point Cloud Compression



## Personal Profile:

First/Last Name: Yuhang Wang

Email: yuhangwang0012@gmail.com

School/University: Southern University of Science and Technology

Graduation Date: June 2024

Major: Computer Science and Technology

Location: Shenzhen,China

Timezone: China Standard Time(CST), UTC+8

Git-hub Profile: [TsingYiPainter (Yuhang Wang) (github.com)](https://github.com/TsingYiPainter)

Open Source Experience:

​	-[OpenCV](https://github.com/opencv/opencv/pull/22682): Handling cases where the depth of Octree setting is too large



## About me

- I am a third-year undergraduate student at Southern University of Science and Technology. Under the guidance of Professor Shiqi Yu, I have accumulated experience in point cloud compression for a semester. This is also my research direction for this semester. I believe I have enough time and ability to complete this project.
- For this project, I plan to utilize an octree as the data structure to store the point cloud data. Currently, I have gained sufficient knowledge on the implementation of an `octree`in the `3d` module of `OpenCV5.x` and have made improvement suggestions that were accepted by the official team. Furthermore, I have studied the source code of Point Cloud Library (`PCL`) regarding point cloud compression and have acquired knowledge on some of the dynamic point cloud compression techniques they adopt, such as `double-buffered octree` compression.
- I have perused numerous scholarly articles concerning the compression of point clouds, which involve various compression techniques such as direct encoding, predictive encoding, and color compression. Furthermore, I have devised a corresponding solution for direct encoding. Additionally, I intend to abide by the `GPCC` encoding standard for the compression of point cloud data.
- I am adept at seeking help. In my research on Computer Version, Professor Shiqi Yu, Research assistants Zihao Mu, Wanli Zhong have provided me with valuable advice. When encountering bottlenecks, will seek advice in a timely manner. Additionally, when there are interim achievements, I will promptly report and listen to their feedback.
- I possess a solid foundation in C++ programming and am proficient in the use of version control tools such as Git and Github. Moreover, I have the experience of contributing to open-source communities.



## Project Info

### Background

Recent advances in computer graphics have made it possible to create realistic digital representations of 3D objects and surroundings. Such representations use various mathematical models to describe both geometry and attribute information of the objects. Among them, 3D point cloud has gained popularity and is extensively used in many industries. This simple data structure consists of an unorganized set of points in 3D space, usually represented in a Cartesian coordinate system with (𝑥, 𝑦, 𝑧). It can carry attributes such as colors, normals, and reflectance, making it suitable for representing both static and dynamic 3D objects.

Real-world applications require point clouds with high densities, often up to millions of points. This demand raises significant requirements in terms of computational and memory resources. Compression algorithms play a crucial role in the computer vision field, especially in handling and processing large amounts of 3D point cloud data. They can help to reduce storage and memory requirements for applications, and make it easier to store, transmit, and process large amounts of visual data. Thus, the development of an efficient point cloud compression algorithm that considers various application constraints has become a challenge.

![Rabbit](https://user-images.githubusercontent.com/84237574/227936502-be4a3df7-7910-4bcf-a7bc-bb5fc52602b9.png)

*From Cao C, Preda M, Zaharia T. (2019). 3D Point Cloud Compression: A Survey. 1-9. 10.1145/3329714.3338130.*

Before delving in, I have reviewed related documents and codes of some existing approaches:

* **G-PCC** (Geometry based Point Cloud Compression) is a method for compressing both static and dynamic 3D point cloud data. This method divides the point cloud into a set of non-overlapping cubic cells, and then encodes the occupancy state of each cell using an octree-based coding scheme. Additionally, a predictive coding technique is employed to further reduce the amount of data needed to represent the point cloud. It has been standardized by MPEG and is widely used in various applications, such as 3D imaging, virtual reality, and autonomous driving. 

* The **PCL**(Point Cloud Library) also provides a point cloud compression functionality. It allows for encoding all kinds of point clouds including “unorganized” point clouds that are characterized by non-existing point references, varying point size, resolution, density and/or point ordering. Furthermore, the underlying octree data structure enables to efficiently merge point cloud data from several sources.

I believe that being familiar with a few mature approaches is beneficial to this project because it provides a foundation for designing and developing our compression algorithm. By analyzing the advantages of current techniques, I can identify areas for improvement and then further enhance the efficiency and effectiveness of our compression algorithm.

### Peer Demonstration and Comparison

// Todo



## Overview

![1](https://user-images.githubusercontent.com/83380147/226165706-193b043f-98cc-45b9-a3ae-0f7883602ceb.png)

*Procedure of Compressing and Decompressing the Point Cloud*

The procedure can be generally divided into five steps:

1. **Preprocessing**: In order to avoid precision error, discretization is necessary since the data is composed by continuous float numbers. The data is thus mapped to discrete space.
2. **Construction**: An octree is constructed to store the data. User can specify the precision according to actual demand, the precision argument decides the depth of the octree.
3. **Serialization**: The complete information of octree is encoded by bit stream. Techniques like occupancy code, direct coding mode are applied to improve compression rate. 
4. **Entropy Coding**: Since the zero-one distribution in the bit stream is not uniform, entropy encoding is applied for further improvement in compression rate.
5. **Output**: Generate a bit stream as output.



### Building The Octree

The **Octree** is a tree-like structure which can divide a 3D space recursively into eight subspaces and assign each subspace into eight child nodes.

![2](https://user-images.githubusercontent.com/83380147/226165712-ac322b0b-517d-4939-9507-bd8b4a4d0deb.png)

When building the octree, the point cloud data is firstly voxelized/discretized: by inserting all the points into a voxel coordinate system.

For example, when precision is set to `0.01`, a point with coordinate `Point3f(0.251,0.502,0.753)` would be transformed by the process:  `Point3i(0.251/0.01,0.502/0.01,0.753/0.01) = Point3i(25,50,75)`.

Then, we calculate the arguments of the octree:

```C++
/**
 * Construction of important parameters for an octree.
 *
 * @param resolution 	Precision of point cloud compression specify by user.
 * @param maxSize	    The maximum span of a point cloud data in the x, y, and z directions.
 * @param maxDepth      The max depth of octree
 * @param depthMask     The length of the side of the cube represented by the octet root node after the point cloud data is discretized
 */        
double maxSize = max(max(maxBound.x - minBound.x, maxBound.y - minBound.y), maxBound.z - 
                             minBound.z);
octree->maxDepth = ceil(log2(maxSize / resolution));
octree->depthMask = 1 << (p->maxDepth - 1);
```

For example, we assume the `maxSize` is 1, then the `maxDepth` will be 7, and the `depthMask` will be 64 (binary is `100_0000`)

Now we insert `Point3i(25,50,75)`(binary is `1_1001,11_0010,100_1011`):

| Round | Depth Mask | x&Depth Mask | y&Depth Mask | z&Depth Mask | Child Index(0-7) |
| ----- | ---------- | ------------ | ------------ | ------------ | ---------------- |
| 1     | 100_0000   | 0            | 1            | 1            | 6                |
| 2     | 10_0000    | 0            | 1            | 0            | 2                |
| 3     | 1_0000     | 1            | 1            | 0            | 3                |
| 4     | 1000       | 1            | 0            | 1            | 5                |
| 5     | 100        | 0            | 0            | 0            | 0                |
| 6     | 10         | 0            | 1            | 1            | 6                |
| 7     | 1          | 1            | 0            | 1            | 5                |

We have already accomplished the insertion of a point cloud data, avoiding precision issues, while improving efficiency by utilizing bitwise operations.



### Encoding/Decoding Octree

![occupancy code](https://user-images.githubusercontent.com/84237574/227980966-dcdf196d-495b-46d0-86ce-cdc7b3e4072d.png)

*Illustration of Octree and Occupancy Code*

The **Occupancy code** is the simplest representation of octree. An occupancy code has the size of one byte (8 bits), and each bit indicates whether a child node exists at the corresponding position.

Encoding is performed by serializing the octree to a byte vector. Each node of an octree can be represented by an occupancy code. The byte vector is a sequence of occupancy codes generated by traversing in **BFS**(Breadth-first search) order starting from the root. This guarantees that we can build the exact same octree structure reversely when decoding.

By using **arithmetic entropy coding** (1), the byte stream can be further compressed due to its non-uniform probability distribution. Our entropy encoder is able to encode/decode the byte stream effectively.
$$
\begin{aligned}
&H = -\sum^{k}_{i=1}p_i log_2(p_i)  &(1)
\end{aligned}
$$
*p_i : The occurrence probability of symbol s_i in the input string.*





## Methodology of Further Enhancement

In order to further enhance the functionality and compression rate of the algorithm, the following methods can be implemented:

### Direct Coding Mode & Predictive Coding Mode

Further achieving compression rate requires fully exploring spatial correlation (neighbor entropy coding), or choosing different coding methods. 

#### Direct Coding Mode



![3](https://user-images.githubusercontent.com/83380147/226165724-d5349ba8-b5eb-427a-9e59-a3f6cd60a1e5.png)

*From MPEG 3DG, G-PCC codec description v12, ISO/IEC JTC 1/SC 29/WG 7 N0151, 2021.*

In practice, when octree's depth increases, a large number of octree nodes are only occupied by a single child, i.e. degenerate from subtree to a linked list, because at that depth the space is finely separated by tiny octree nodes(like dense voxels), and points are relatively isolated, so no spatial correlation is taken into advantage. In G-PCC, Direct Coding Mode(DCM) is applied to handle those nodes. In DCM, only the coordinate information of each node are encoded. 



#### Predictive Coding Mode

The predictive coding model is based on the spatial position relation of octree nodes. Specifically, the predictive encoding mode predicts the value of a node's occupancy code according to its position in the octree as well as the information of its neighbors. This approach takes advantage of the correlation between adjacent nodes in the space to reduce the number of bits required for entropy coding, thus achieving better compression outcome. 

![Occupancy Code](https://user-images.githubusercontent.com/84237574/227981345-eaa47e3b-1629-4dae-9fc6-711368c49dbe.png)

*Child node is likely to have the same occupancy distribution as its parent node*

Through statistics, we found that given the specific neighborhood node information, the node's occupancy code is often predictable. The placeholders of this node will tend to form a plane with the neighbors. 

The prediction methods can be implemented based on statistics or MLS(Moving Least-squares) surface fitting. In order to use the predicted information in compression, the predictive coding mode will send the residual difference between the predicted value and the real value of the encoded occupancy code into the entropy coding module for compression.

![Different Coding Approaches](https://user-images.githubusercontent.com/84237574/227980041-ab9a3d0a-e33e-46f3-89fe-6e5a5cd27321.png)

*Different Coding Approaches for Different Cases*

In practice, we have to select different coding method based on the node's geometry information. If the node shows certain regularity, predictive coding mode can help. If the node is isolated, direct coding mode has better performance.



### Color Attribute Encoding

###### For static point cloud:

Haar wavelet transform is a mathematical technique used in image compression to reduce the size of digital images while preserving their essential features. The transform works by decomposing an image into a series of wavelet coefficients that capture the image's high-frequency components.

![2D Haar Wavelet Example](https://www.researchgate.net/profile/Essam-Houssein/publication/308881224/figure/fig1/AS:413604256993280@1475622347362/2D-Haar-Wavelet-Transform-Example.png)

*2D Haar Wavelet Transform Example*

The **RAHT(Region-Adaptive Hierarchical Transform)** is a hierarchical sub-band transform that resembles an adaptive variation of a Haar wavelet in 3D point clouds. It is inspired by using the colors associated with a node in a lower level of the octree to predict the colors of the nodes in the next level. It is implemented by following backwards the octree scan, from individual voxels to the entire space, at each step grouping voxels of the same level into larger ones in each direction until reaching the root. It quantizes the transform coefficients using a uniform scalar quantizer, and then entropy codes each quantized coefficient using an arithmetic coder. The decoder operates in the reverse manner.

###### For dynamic point cloud:

Double-buffering octree can make full use of the correlation between point cloud frames to complete the compression of point cloud. 

![double-buffering](https://user-images.githubusercontent.com/84237574/227980250-6755e16a-7d65-419f-b587-03ce9d7d38cc.png)

*Illustration of differential encoding technique when using double-buffering octree*

For each frame, only the difference between it and the previous frame needs to be coded. It is worth noting that the size of the difference between frames is negligible. Therefore, in a selected interval, if the first frame at this interval is taken as the initial frame, the difference between each subsequent frame can be encoded. This method can greatly improve the compression rate of dynamic point cloud compression.



### Inter-frame Encoding

Dynamic point clouds has not only spatial continuity but also temporal redundancy. When compressing the adjacent frames of dynamic point cloud data, the current frame can refer to the previous frame. 

![inter-frame](https://user-images.githubusercontent.com/84237574/227980389-92335b73-6d55-4c7f-a5bc-c0930dd941b8.png)

*Inter-frame Encoding*

After encoding both current and previous frames with octree, the octree nodes are compared in the octree structure to identify the nodes that have changed. Then the encoding results of the two frames are concatenated with the occupancy codes of the nodes that have not changed from the previous frame to output the compression result of the current frame. During this process, due to the utilization of temporal redundancy, the occupancy codes of the previous frame can be directly copied, thus reducing the storage space cost.



## Expected Deliverables

- **A point cloud compression algorithm with optional precision support**. 
- The outcome will support **color attribute compression** and utilize methods such as **DCM**, **predictive coding**, and **entropy coding** to improve compression rate. For dynamic point clouds, **inter-frame encoding** will be applied to further improve compression rate.
- The relevant functions in the algorithm **adhere to the OpenCV code style**. A clear **explanation document** will be provided.
- The algorithm will be evaluated based on compression rate, speed, and reconstruction quality. Bits per point(bpp) will be used as the evaluation metric for compression rate and Peak Signal-to-Noise Ratio (PSNR) will be used as the evaluation metric for reconstruction quality.
- The algorithm will be tested for correctness based on [existing dataset](http://graphics.stanford.edu/data/3Dscanrep/) and real life point cloud data captured by my depth camera.



## Expected Schedule

Prior - May 29

- Reading papers of point cloud compression algorithms
- Discussion with mentor about specific aspects of the project

June 1-14

- Implement direct coding mode & predictive coding mode
- Discussion of the plan for encoding color attribute

June 16- July 10

- Implement color attribute encoding
- Discussion of inter-frame encoding

July 10 - 28

- Finish double buffering octree	

August 1-14

- Test and improve the algorithm
- Rewrite based on OpenCV standard
- Write a document for program 

August 15-28

- Write a summary report throughout the project

