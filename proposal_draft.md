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
- I possess a solid foundation in C++ programming and am proficient in the use of version control tools such as Git and Github. Moreover, I am familiar with the process of contributing to open-source communities.



## Project Info

### Background

Recent advances in computer graphics have made it possible to create realistic digital representations of 3D objects and surroundings. Such representations use various mathematical models to describe both geometry and attribute information of the objects. Among them, 3D point cloud has gained popularity and is extensively used in many industries. This simple data structure consists of an unorganized set of points in 3D space, usually represented in a Cartesian coordinate system with (𝑥, 𝑦, 𝑧). It can carry attributes such as colors, normals, and reflectance, making it suitable for representing both static and dynamic 3D objects.

Real-world applications require point clouds with high densities, often up to millions of points. This demand raises significant requirements in terms of computational and memory resources. Compression algorithms play a crucial role in the computer vision field, especially in handling and processing large amounts of 3D point cloud data. They can help to reduce storage and memory requirements for applications, and make it easier to store, transmit, and process large amounts of visual data. Thus, the development of an efficient point cloud compression algorithm that considers various application constraints has become a challenge.

Before delving in, I have reviewed related documents and code of some existing approaches:

* G-PCC (Geometry based Point Cloud Compression) is a method for compressing both static and dynamic 3D point cloud data. This method divides the point cloud into a set of non-overlapping cubic cells, and then encodes the occupancy state of each cell using an octree-based coding scheme. Additionally, a predictive coding technique is employed to further reduce the amount of data needed to represent the point cloud. It has been standardized by MPEG and is widely used in various applications, such as 3D imaging, virtual reality, and autonomous driving. 

* The PCL(Point Cloud Library) also provides a point cloud compression functionality. It allows for encoding all kinds of point clouds including “unorganized” point clouds that are characterized by non-existing point references, varying point size, resolution, density and/or point ordering. Furthermore, the underlying octree data structure enables to efficiently merge point cloud data from several sources.

I believe that being familiar with a few mature approaches is beneficial to this project because it provides a foundation for designing and developing our compression algorithm. By analyzing the advantages of current techniques, I can identify areas for improvement and then further enhance the efficiency and effectiveness of my compression algorithm.

### Field Study



### Peer Demonstration and Comparison





## Design

### Overview

![1](https://user-images.githubusercontent.com/83380147/226165706-193b043f-98cc-45b9-a3ae-0f7883602ceb.png)


The above figure demonstrates the procedure of compression and decompression of point cloud data.

The procedure consists of five steps:

1. Preprocessing: In order to avoid the precision error, discretization is necessary since the data is composed by continuous float numbers. The data is thus mapped to discrete space.
2. Construction of octree: Octree is applied to store the data. User can specify the precision according to actual demand, the precision argument decides the depth of the octree.
3. Serialization: The complete information of octree is encoded by bit stream. Techniques like occupancy code, direct coding mode are applied to improve compression rate. 
4. Entropy Coding: Since the zero-one distribution in the bit stream is not uniform, entropy encoding is applied for further improvement.
5. Outputs: Output a bit stream.



### Building The Octree

![2](https://user-images.githubusercontent.com/83380147/226165712-ac322b0b-517d-4939-9507-bd8b4a4d0deb.png)


First, the point cloud data is discretized: we insert all the points into a voxel coordinate system.

For example, when precision is set to `0.01`, a point with coordinate `Point3f(0.251,0.502,0.753)`  would transform to  `Point3i(0.251/0.01,0.502/0.01,0.753/0.01)=Point3i(25,50,75)`.

Then construct the arguments of the octree

```C++
/**
 * Construction of important parameters for an octree.
 *
 * @param resolution 	Precision of point cloud compression specify by user.
 * @param maxSize	The maximum span of a point cloud data in the x, y, and z directions.
 * @param maxDepth      The max depth of octree
 * @param depthMask     The length of the side of the cube represented by the octet root node after the point cloud data is discretized
 */        
double maxSize = max(max(maxBound.x - minBound.x, maxBound.y - minBound.y), maxBound.z - 
                             minBound.z);
octree->maxDepth = ceil(log2(maxSize / resolution));
octree->depthMask = 1 << (p->maxDepth - 1);
```

Assume the `maxSize`is 1, then the `maxDepth` is 7, the `depthMask` is 64(binary is `100_0000`)

Now we insert `Point3i(25,50,75)`, binary is `(1_1001,11_0010,100_1011)`

| round | depthMask | x&depthMask | y&depthMask | z&depthMask | which child(0-7) |
| ----- | --------- | ----------- | ----------- | ----------- | ---------------- |
| 1     | 100_0000  | 0           | 1           | 1           | 6                |
| 2     | 10_0000   | 0           | 1           | 0           | 2                |
| 3     | 1_0000    | 1           | 1           | 0           | 3                |
| 4     | 1000      | 1           | 0           | 1           | 5                |
| 5     | 100       | 0           | 0           | 0           | 0                |
| 6     | 10        | 0           | 1           | 1           | 6                |
| 7     | 1         | 1           | 0           | 1           | 5                |

Well, we have accomplished the insertion of a point cloud data, avoiding precision issues, while improving efficiency by utilizing bitwise operations.



### Methods to Encode/Decode Octree

Encoding is performed by serializing the octree to a byte vector. Each node of an octree can be represented by an occupancy code of eight bits, each bit representing whether a specific child is occupied (has a point in its volume). The byte vector is a sequence of occupancy codes generated by traversing in BFS order starting from the root. This order guarantees that we can reverse-build the same octree structure when decoding.

By using arithmetic entropy coding, the byte stream can be further compressed due to its non-uniform probability distribution. Our entropy encoder encodes and decodes the byte stream effectively.

Further achieving compression rate requires exploring spatial correlation (neighbor entropy coding), or choosing different coding methods. In practice, we find out that when octree's depth is deepened (to improve precision), most octree nodes is only occupied by one child, i.e. degenerate from subtree to a chain, because at that depth the space is finely separated by tiny octree nodes(like dense voxels), and points are isolated to each node, so no spatial correlation is taken into advantage. Thus, directly encoding the isolated points' positions is more efficient than using occupancy codes. This method is called "Direct Coding Mode" (DCM).Here is a diagram of the algorithm



![3](https://user-images.githubusercontent.com/83380147/226165724-d5349ba8-b5eb-427a-9e59-a3f6cd60a1e5.png)




### Methods to Encode Color

Similar to a method commonly used in image compression, the region-adaptive hierarchical transform (RAHT) is an ideal method for encoding the color attribute of point cloud data. 

The RAHT is a hierarchical sub-band transform that resembles an adaptive variation of a Haar wavelet. It is implemented by following backwards the octree scan, from individual voxels to the entire space, at each step grouping voxels of the same level into larger ones in each direction until reaching the root. It quantizes the transform coefficients using a uniform scalar quantizer, and then entropy codes each quantized coefficient using an arithmetic coder (AC). The decoder operates in the reverse manner.

[Compression of 3D Point Clouds Using a Region-Adaptive Hierarchical Transform](https://ieeexplore.ieee.org/abstract/document/7482691)



## Expected Deliverables

- I will develop a point cloud compression algorithm with optional precision support. The algorithm will include color attribute compression and utilize methods such as `DCM`, `predictive coding`, and `entropy coding` to improve compression rate. To address dynamic point clouds, a `double-buffered octree` structure can be employed to further improve compression rate.
- The relevant functions in the algorithm adhere to the style of the `OpenCV` library and provide clear documentation explanations.
- The algorithm will be evaluated based on compression rate, speed, and reconstruction quality. `Bits per point (bpp)` will be used as the evaluation metric for compression rate, while `Peak Signal-to-Noise Ratio (PSNR)` will be used as the evaluation metric for reconstruction quality.
- Data set:http://graphics.stanford.edu/data/3Dscanrep/. Also, I will use the depth camera to capture real life point cloud data







## Expected Schedule

Prior - May 29

- Reading papers of point cloud compression algorithms
- Discussion with mentor about specific aspects of project

June 1-14

- Finish direct coding mode
- Discuss the completed plan for encoding color

June 16- July 10

- Finish encode color
- Discuss the completed plan for double buffering octree

July 10 - 28

- Finish double buffering octree	

August 1-14

- Test and improve the program
- Write a document for program 
- start working on Checkstyle check

August 15-28

- Write a summary article throughout the project
- some flexible time

