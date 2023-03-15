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
#include <opencv2/highgui.hpp>

namespace opencv_test { namespace {

//rotate/flip a quadrant appropriately
static void rot(int n, int *x, int *y, int rx, int ry)
{
    if (ry == 0) {
        if (rx == 1) {
            *x = n-1 - *x;
            *y = n-1 - *y;
        }

        //Swap x and y
        int t  = *x;
        *x = *y;
        *y = t;
    }
}

static void d2xy(int n, int d, int *x, int *y)
{
    int rx, ry, s, t=d;
    *x = *y = 0;
    for (s=1; s<n; s*=2)
    {
        rx = 1 & (t/2);
        ry = 1 & (t ^ rx);
        rot(s, x, y, rx, ry);
        *x += s * rx;
        *y += s * ry;
        t /= 4;
    }
}

TEST(Imgproc_FindContours, hilbert)
{
    int n = 64, n2 = n*n, scale = 10, w = (n + 2)*scale;
    Point ofs(scale, scale);
    Mat img(w, w, CV_8U);
    img.setTo(Scalar::all(0));

    Point p(0,0);
    for( int i = 0; i < n2; i++ )
    {
        Point q(0,0);
        d2xy(n2, i, &q.x, &q.y);
        line(img, p*scale + ofs, q*scale + ofs, Scalar::all(255));
        p = q;
    }
    dilate(img, img, Mat());
    vector<vector<Point> > contours;
    findContours(img, contours, noArray(), RETR_LIST, CHAIN_APPROX_SIMPLE);
    printf("ncontours = %d, contour[0].npoints=%d\n", (int)contours.size(), (int)contours[0].size());
    img.setTo(Scalar::all(0));

    drawContours(img, contours, 0, Scalar::all(255), 1);

    ASSERT_EQ(1, (int)contours.size());
    ASSERT_EQ(9832, (int)contours[0].size());
}

TEST(Imgproc_FindContours, border)
{
    Mat img;
    cv::copyMakeBorder(Mat::zeros(8, 10, CV_8U), img, 1, 1, 1, 1, BORDER_CONSTANT, Scalar(1));

    std::vector<std::vector<cv::Point> > contours;
    findContours(img, contours, RETR_LIST, CHAIN_APPROX_NONE);

    Mat img_draw_contours = Mat::zeros(img.size(), CV_8U);
    for (size_t cpt = 0; cpt < contours.size(); cpt++)
    {
      drawContours(img_draw_contours, contours, static_cast<int>(cpt), cv::Scalar(1));
    }

    ASSERT_EQ(0, cvtest::norm(img, img_draw_contours, NORM_INF));
}

TEST(Imgproc_FindContours, regression_4363_shared_nbd)
{
    // Create specific test image
    Mat1b img(12, 69, (const uchar&)0);

    img(1, 1) = 1;

    // Vertical rectangle with hole sharing the same NBD
    for (int r = 1; r <= 10; ++r) {
        for (int c = 3; c <= 5; ++c) {
            img(r, c) = 1;
        }
    }
    img(9, 4) = 0;

    // 124 small CCs
    for (int r = 1; r <= 7; r += 2) {
        for (int c = 7; c <= 67; c += 2) {
            img(r, c) = 1;
        }
    }

    // Last CC
    img(9, 7) = 1;

    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(img, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

    bool found = false;
    size_t index = 0;
    for (vector< vector<Point> >::const_iterator i = contours.begin(); i != contours.end(); ++i)
    {
        const vector<Point>& c = *i;
        if (!c.empty() && c[0] == Point(7, 9))
        {
            found = true;
            index = (size_t)(i - contours.begin());
            break;
        }
    }
    EXPECT_TRUE(found) << "Desired result: point (7,9) is a contour - Actual result: point (7,9) is not a contour";

    if (found)
    {
        EXPECT_LT(hierarchy[index][3], 0) << "Desired result: (7,9) has no parent - Actual result: parent of (7,9) is another contour. index = " << index;
    }
}

TEST(Imgproc_PointPolygonTest, regression_10222)
{
    vector<Point> contour;
    contour.push_back(Point(0, 0));
    contour.push_back(Point(0, 100000));
    contour.push_back(Point(100000, 100000));
    contour.push_back(Point(100000, 50000));
    contour.push_back(Point(100000, 0));

    const Point2f point(40000, 40000);
    const double result = cv::pointPolygonTest(contour, point, false);
    EXPECT_GT(result, 0) << "Desired result: point is inside polygon - actual result: point is not inside polygon";
}

}} // namespace
/* End of file. */
