// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.

#include "test_precomp.hpp"
#include <fstream>


namespace opencv_test {
    namespace {

        using namespace cv;

        class OctreeCompressTest : public testing::Test {
        protected:
            void SetUp() override {
                // SETTING (Filebase is the path of pointcloud with the file name, but no ".ply")
                String FileBase = R"(D:\mydoc\CS_Resources\PCL_1.12.1\share\doc\pcl-1.12\tutorials\sources\cloud_viewer\cmake-build-debug-visual-studio\Debug\dress\dress_raw)";
                // TODO lew resolution(big number) causes error!!
                float resolution = 5;
                String label = "Test_Color_";
                String res_str = std::to_string(resolution);
                res_str.erase(res_str.find_last_not_of('0') + 1);
                res_str.erase(res_str.find('.'), 1);


                // measure time
                auto start = std::chrono::high_resolution_clock::now();

                //load .ply file
                String loadFileName = FileBase + ".ply";
                // color: rgb
                loadPointCloud(loadFileName, pointcloud, normal_placeholder, color_attribute);

                // Generate OctreeCompress From PointCloud,resolution is the minimum precision that can be specified and must be 10^x
                treeTest.create(pointcloud, color_attribute, resolution);


                //compressed char array to byte stream
                std::ofstream vectorToStream;
                vectorToStream.open(FileBase + label + res_str + ".bin", std::ios_base::binary);
                treeTest.traverse(vectorToStream);
                vectorToStream.close();

                // measure time
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                std::cout << "Time taken by Encode: "
                          << duration.count() << " microseconds" << std::endl;

                // measure time
                start = std::chrono::high_resolution_clock::now();

                std::ifstream streamToTree;
                streamToTree.open(FileBase + label + res_str + ".bin", std::ios_base::binary);

                //restore OctreeCompress from char array
                treeTest.reStore(streamToTree);
                std::cout << "Restored " << std::endl;

                //restore PointCloud from OctreeCompress
                treeTest.getPointCloudByOctree(restorePointCloud, restore_color_attribute);
                //save .ply file
                String saveFileName= FileBase + label + res_str + ".ply";
                savePointCloud(saveFileName,restorePointCloud, normal_placeholder, restore_color_attribute);

                // measure time
                stop = std::chrono::high_resolution_clock::now();
                duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
                std::cout << "Time taken by Decode: "
                          << duration.count() << " microseconds" << std::endl;
            }

        public:
            std::vector<Point3f> pointcloud;
            std::vector<Point3f> normal_placeholder;
            std::vector<Point3f> color_attribute;
            std::vector<Point3f> restorePointCloud;
            std::vector<Point3f> restore_color_attribute;
            Point3f restPoint;
            OctreeCompress treeTest;

        private:
            int maxDepth;
        };

        TEST_F(OctreeCompressTest, BasicFunctionTest) {
            EXPECT_TRUE(true);
        }
    } // namespace
} // opencv_test
