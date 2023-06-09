/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "test_precomp.hpp"

namespace opencv_test { namespace {

BIGDATA_TEST(Imgproc_DistanceTransform, large_image_12218)
{
    const int lls_maxcnt = 79992000;   // labels's maximum count
    const int lls_mincnt = 1;          // labels's minimum count
    int i, j, nz;
    Mat src(8000, 20000, CV_8UC1), dst, labels;
    for( i = 0; i < src.rows; i++ )
        for( j = 0; j < src.cols; j++ )
            src.at<uchar>(i, j) = (j > (src.cols / 2)) ? 0 : 255;

    distanceTransform(src, dst, labels, cv::DIST_L2, cv::DIST_MASK_3, DIST_LABEL_PIXEL);

    double scale = (double)lls_mincnt / (double)lls_maxcnt;
    labels.convertTo(labels, CV_32SC1, scale);
    Size size = labels.size();
    nz = cv::countNonZero(labels);
    EXPECT_EQ(nz, (size.height*size.width / 2));
}

}} // namespace
