// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "test_precomp.hpp"
#include <fstream>


namespace opencv_test { namespace {

        using namespace cv;

        class OctreeCompressTest: public testing::Test
        {
        protected:
            void SetUp() override
            {
                //load .ply file
                String loadFileName=R"(D:\mydoc\CS_Resources\PCL_1.12.1\share\doc\pcl-1.12\tutorials\sources\cloud_viewer\cmake-build-debug-visual-studio\Debug\face\myface.ply)";
                loadPointCloud(loadFileName,pointcloud);

                // Generate OctreeCompress From PointCloud,resolution is the minimum precision that can be specified and must be 10^x
                treeTest.create(pointcloud, 10);

                //traverse OctreeCompress by level order.Get the char array representation of the tree
                std::vector<unsigned char> treeToVector;
                treeTest.traverse(treeToVector);

                //compressed char array to byte stream
                std::ofstream vectorToStream;
                //std::stringstream vectorToStream;
                //vectorToStream.open(R"(A_myPly\output\stream_01.bin)", std::ios_base::binary);
                vectorToStream.open(
                        R"(D:\mydoc\CS_Resources\PCL_1.12.1\share\doc\pcl-1.12\tutorials\sources\cloud_viewer\cmake-build-debug-visual-studio\Debug\face\face_cp_10.bin)", std::ios_base::binary);
                treeTest.encodeCharVectorToStream(treeToVector,vectorToStream);
                vectorToStream.close();

                std::ifstream streamToVector;
                streamToVector.open(
                        R"(D:\mydoc\CS_Resources\PCL_1.12.1\share\doc\pcl-1.12\tutorials\sources\cloud_viewer\cmake-build-debug-visual-studio\Debug\face\face_cp_10.bin)", std::ios_base::binary);
                //restore char array from byte stream
                std::vector<unsigned char> vectorToTree(treeToVector.size());
                treeTest.decodeStreamToCharVector(streamToVector,vectorToTree);
                streamToVector.close();

                //restore OctreeCompress from char array
                treeTest.reStore(vectorToTree);

                //restore PointCloud from OctreeCompress
                treeTest.getPointCloudByOctree(restorePointCloud);
                //save .ply file
                String saveFileName=R"(D:\mydoc\CS_Resources\PCL_1.12.1\share\doc\pcl-1.12\tutorials\sources\cloud_viewer\cmake-build-debug-visual-studio\Debug\face\myface_10.ply)";
                savePointCloud(saveFileName,restorePointCloud);
            }

        public:
            std::vector<Point3f> pointcloud;
            std::vector<Point3f> restorePointCloud;
            Point3f restPoint;
            OctreeCompress treeTest;

        private:
            int maxDepth;
        };
        TEST_F(OctreeCompressTest, BasicFunctionTest)
        {
            EXPECT_TRUE(true);
        }
    } // namespace
} // opencv_test
