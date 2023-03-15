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

class CV_ImgWarpBaseTest : public cvtest::ArrayTest
{
public:
    CV_ImgWarpBaseTest( bool warp_matrix );

protected:
    int read_params( const cv::FileStorage& fs );
    int prepare_test_case( int test_case_idx );
    void get_test_array_types_and_sizes( int test_case_idx, vector<vector<Size> >& sizes, vector<vector<int> >& types );
    void get_minmax_bounds( int i, int j, int type, Scalar& low, Scalar& high );
    void fill_array( int test_case_idx, int i, int j, Mat& arr );

    int interpolation;
    int max_interpolation;
    double spatial_scale_zoom, spatial_scale_decimate;
};


CV_ImgWarpBaseTest::CV_ImgWarpBaseTest( bool warp_matrix )
{
    test_array[INPUT].push_back(NULL);
    if( warp_matrix )
        test_array[INPUT].push_back(NULL);
    test_array[INPUT_OUTPUT].push_back(NULL);
    test_array[REF_INPUT_OUTPUT].push_back(NULL);
    max_interpolation = 5;
    interpolation = 0;
    element_wise_relative_error = false;
    spatial_scale_zoom = 0.01;
    spatial_scale_decimate = 0.005;
}


int CV_ImgWarpBaseTest::read_params( const cv::FileStorage& fs )
{
    int code = cvtest::ArrayTest::read_params( fs );
    return code;
}


void CV_ImgWarpBaseTest::get_minmax_bounds( int i, int j, int type, Scalar& low, Scalar& high )
{
    cvtest::ArrayTest::get_minmax_bounds( i, j, type, low, high );
    if( CV_MAT_DEPTH(type) == CV_32F )
    {
        low = Scalar::all(-10.);
        high = Scalar::all(10);
    }
}


void CV_ImgWarpBaseTest::get_test_array_types_and_sizes( int test_case_idx,
                                                vector<vector<Size> >& sizes, vector<vector<int> >& types )
{
    RNG& rng = ts->get_rng();
    int depth = cvtest::randInt(rng) % 3;
    int cn = cvtest::randInt(rng) % 3 + 1;
    cvtest::ArrayTest::get_test_array_types_and_sizes( test_case_idx, sizes, types );
    depth = depth == 0 ? CV_8U : depth == 1 ? CV_16U : CV_32F;

    types[INPUT][0] = types[INPUT_OUTPUT][0] = types[REF_INPUT_OUTPUT][0] = CV_MAKETYPE(depth, cn);
    if( test_array[INPUT].size() > 1 )
        types[INPUT][1] = cvtest::randInt(rng) & 1 ? CV_32FC1 : CV_64FC1;

    interpolation = cvtest::randInt(rng) % max_interpolation;
}


void CV_ImgWarpBaseTest::fill_array( int test_case_idx, int i, int j, Mat& arr )
{
    if( i != INPUT || j != 0 )
        cvtest::ArrayTest::fill_array( test_case_idx, i, j, arr );
}

int CV_ImgWarpBaseTest::prepare_test_case( int test_case_idx )
{
    int code = cvtest::ArrayTest::prepare_test_case( test_case_idx );
    Mat& img = test_mat[INPUT][0];
    int i, j, cols = img.cols;
    int type = img.type(), depth = CV_MAT_DEPTH(type), cn = CV_MAT_CN(type);
    double scale = depth == CV_16U ? 1000. : 255.*0.5;
    double space_scale = spatial_scale_decimate;
    vector<float> buffer(img.cols*cn);

    if( code <= 0 )
        return code;

    if( test_mat[INPUT_OUTPUT][0].cols >= img.cols &&
        test_mat[INPUT_OUTPUT][0].rows >= img.rows )
        space_scale = spatial_scale_zoom;

    for( i = 0; i < img.rows; i++ )
    {
        uchar* ptr = img.ptr(i);
        switch( cn )
        {
        case 1:
            for( j = 0; j < cols; j++ )
                buffer[j] = (float)((sin((i+1)*space_scale)*sin((j+1)*space_scale)+1.)*scale);
            break;
        case 2:
            for( j = 0; j < cols; j++ )
            {
                buffer[j*2] = (float)((sin((i+1)*space_scale)+1.)*scale);
                buffer[j*2+1] = (float)((sin((i+j)*space_scale)+1.)*scale);
            }
            break;
        case 3:
            for( j = 0; j < cols; j++ )
            {
                buffer[j*3] = (float)((sin((i+1)*space_scale)+1.)*scale);
                buffer[j*3+1] = (float)((sin(j*space_scale)+1.)*scale);
                buffer[j*3+2] = (float)((sin((i+j)*space_scale)+1.)*scale);
            }
            break;
        case 4:
            for( j = 0; j < cols; j++ )
            {
                buffer[j*4] = (float)((sin((i+1)*space_scale)+1.)*scale);
                buffer[j*4+1] = (float)((sin(j*space_scale)+1.)*scale);
                buffer[j*4+2] = (float)((sin((i+j)*space_scale)+1.)*scale);
                buffer[j*4+3] = (float)((sin((i-j)*space_scale)+1.)*scale);
            }
            break;
        default:
            CV_Assert(0);
        }

        /*switch( depth )
        {
        case CV_8U:
            for( j = 0; j < cols*cn; j++ )
                ptr[j] = (uchar)cvRound(buffer[j]);
            break;
        case CV_16U:
            for( j = 0; j < cols*cn; j++ )
                ((ushort*)ptr)[j] = (ushort)cvRound(buffer[j]);
            break;
        case CV_32F:
            for( j = 0; j < cols*cn; j++ )
                ((float*)ptr)[j] = (float)buffer[j];
            break;
        default:
            CV_Assert(0);
        }*/
        cv::Mat src(1, cols*cn, CV_32F, &buffer[0]);
        cv::Mat dst(1, cols*cn, depth, ptr);
        src.convertTo(dst, dst.type());
    }

    return code;
}

/////////////////////////

static void test_remap( const Mat& src, Mat& dst, const Mat& mapx, const Mat& mapy,
                        Mat* mask=0, int interpolation=cv::INTER_LINEAR )
{
    int x, y, k;
    int drows = dst.rows, dcols = dst.cols;
    int srows = src.rows, scols = src.cols;
    const uchar* sptr0 = src.ptr();
    int depth = src.depth(), cn = src.channels();
    int elem_size = (int)src.elemSize();
    int step = (int)(src.step / CV_ELEM_SIZE(depth));
    int delta;

    if( interpolation != cv::INTER_CUBIC )
    {
        delta = 0;
        scols -= 1; srows -= 1;
    }
    else
    {
        delta = 1;
        scols = MAX(scols - 3, 0);
        srows = MAX(srows - 3, 0);
    }

    int scols1 = MAX(scols - 2, 0);
    int srows1 = MAX(srows - 2, 0);

    if( mask )
        *mask = Scalar::all(0);

    for( y = 0; y < drows; y++ )
    {
        uchar* dptr = dst.ptr(y);
        const float* mx = mapx.ptr<float>(y);
        const float* my = mapy.ptr<float>(y);
        uchar* m = mask ? mask->ptr(y) : 0;

        for( x = 0; x < dcols; x++, dptr += elem_size )
        {
            float xs = mx[x];
            float ys = my[x];
            int ixs = cvFloor(xs);
            int iys = cvFloor(ys);

            if( (unsigned)(ixs - delta - 1) >= (unsigned)scols1 ||
                (unsigned)(iys - delta - 1) >= (unsigned)srows1 )
            {
                if( m )
                    m[x] = 1;
                if( (unsigned)(ixs - delta) >= (unsigned)scols ||
                    (unsigned)(iys - delta) >= (unsigned)srows )
                    continue;
            }

            xs -= ixs;
            ys -= iys;

            switch( depth )
            {
            case CV_8U:
                {
                const uchar* sptr = sptr0 + iys*step + ixs*cn;
                for( k = 0; k < cn; k++ )
                {
                    float v00 = sptr[k];
                    float v01 = sptr[cn + k];
                    float v10 = sptr[step + k];
                    float v11 = sptr[step + cn + k];

                    v00 = v00 + xs*(v01 - v00);
                    v10 = v10 + xs*(v11 - v10);
                    v00 = v00 + ys*(v10 - v00);
                    dptr[k] = (uchar)cvRound(v00);
                }
                }
                break;
            case CV_16U:
                {
                const ushort* sptr = (const ushort*)sptr0 + iys*step + ixs*cn;
                for( k = 0; k < cn; k++ )
                {
                    float v00 = sptr[k];
                    float v01 = sptr[cn + k];
                    float v10 = sptr[step + k];
                    float v11 = sptr[step + cn + k];

                    v00 = v00 + xs*(v01 - v00);
                    v10 = v10 + xs*(v11 - v10);
                    v00 = v00 + ys*(v10 - v00);
                    ((ushort*)dptr)[k] = (ushort)cvRound(v00);
                }
                }
                break;
            case CV_32F:
                {
                const float* sptr = (const float*)sptr0 + iys*step + ixs*cn;
                for( k = 0; k < cn; k++ )
                {
                    float v00 = sptr[k];
                    float v01 = sptr[cn + k];
                    float v10 = sptr[step + k];
                    float v11 = sptr[step + cn + k];

                    v00 = v00 + xs*(v01 - v00);
                    v10 = v10 + xs*(v11 - v10);
                    v00 = v00 + ys*(v10 - v00);
                    ((float*)dptr)[k] = (float)v00;
                }
                }
                break;
            default:
                CV_Assert(0);
            }
        }
    }
}

/////////////////////////

class CV_RemapTest : public CV_ImgWarpBaseTest
{
public:
    CV_RemapTest();

protected:
    void get_test_array_types_and_sizes( int test_case_idx, vector<vector<Size> >& sizes, vector<vector<int> >& types );
    void run_func();
    int prepare_test_case( int test_case_idx );
    void prepare_to_validation( int /*test_case_idx*/ );
    double get_success_error_level( int test_case_idx, int i, int j );
    void fill_array( int test_case_idx, int i, int j, Mat& arr );
};


CV_RemapTest::CV_RemapTest() : CV_ImgWarpBaseTest( false )
{
    //spatial_scale_zoom = spatial_scale_decimate;
    test_array[INPUT].push_back(NULL);
    test_array[INPUT].push_back(NULL);

    spatial_scale_decimate = spatial_scale_zoom;
}


void CV_RemapTest::get_test_array_types_and_sizes( int test_case_idx, vector<vector<Size> >& sizes, vector<vector<int> >& types )
{
    CV_ImgWarpBaseTest::get_test_array_types_and_sizes( test_case_idx, sizes, types );
    types[INPUT][1] = types[INPUT][2] = CV_32FC1;
    interpolation = cv::INTER_LINEAR;
}


void CV_RemapTest::fill_array( int test_case_idx, int i, int j, Mat& arr )
{
    if( i != INPUT )
        CV_ImgWarpBaseTest::fill_array( test_case_idx, i, j, arr );
}


void CV_RemapTest::run_func()
{
    remap( cvarrToMat( test_array[INPUT][0] ), cvarrToMat( test_array[INPUT_OUTPUT][0] ),
             cvarrToMat( test_array[INPUT][1] ), cvarrToMat( test_array[INPUT][2] ), interpolation );
}


double CV_RemapTest::get_success_error_level( int /*test_case_idx*/, int /*i*/, int /*j*/ )
{
    int depth = test_mat[INPUT][0].depth();
    return depth == CV_8U ? 16 : depth == CV_16U ? 1024 : 5e-2;
}


int CV_RemapTest::prepare_test_case( int test_case_idx )
{
    RNG& rng = ts->get_rng();
    int code = CV_ImgWarpBaseTest::prepare_test_case( test_case_idx );
    const Mat& src = test_mat[INPUT][0];
    double a[9] = {0,0,0,0,0,0,0,0,1}, k[4];
    Mat _a( 3, 3, CV_64F, a );
    Mat _k( 4, 1, CV_64F, k );
    double sz = MAX(src.rows, src.cols);

    if( code <= 0 )
        return code;

    double aspect_ratio = cvtest::randReal(rng)*0.6 + 0.7;
    a[2] = (src.cols - 1)*0.5 + cvtest::randReal(rng)*10 - 5;
    a[5] = (src.rows - 1)*0.5 + cvtest::randReal(rng)*10 - 5;
    a[0] = sz/(0.9 - cvtest::randReal(rng)*0.6);
    a[4] = aspect_ratio*a[0];
    k[0] = cvtest::randReal(rng)*0.06 - 0.03;
    k[1] = cvtest::randReal(rng)*0.06 - 0.03;
    if( k[0]*k[1] > 0 )
        k[1] = -k[1];
    k[2] = cvtest::randReal(rng)*0.004 - 0.002;
    k[3] = cvtest::randReal(rng)*0.004 - 0.002;

    cvtest::initUndistortMap( _a, _k, Mat(), Mat(), test_mat[INPUT][1].size(), test_mat[INPUT][1], test_mat[INPUT][2], CV_32F );
    return code;
}


void CV_RemapTest::prepare_to_validation( int /*test_case_idx*/ )
{
    Mat& dst = test_mat[REF_INPUT_OUTPUT][0];
    Mat& dst0 = test_mat[INPUT_OUTPUT][0];
    Mat mask( dst.size(), CV_8U );
    test_remap(test_mat[INPUT][0], dst, test_mat[INPUT][1],
               test_mat[INPUT][2], &mask, interpolation );
    dst.setTo(Scalar::all(0), mask);
    dst0.setTo(Scalar::all(0), mask);
}

////////////////////////////// GetRectSubPix /////////////////////////////////

static void
test_getQuadrangeSubPix( const Mat& src, Mat& dst, double* a )
{
    int sstep = (int)(src.step / sizeof(float));
    int scols = src.cols, srows = src.rows;

    CV_Assert( src.depth() == CV_32F && src.type() == dst.type() );

    int cn = dst.channels();

    for( int y = 0; y < dst.rows; y++ )
        for( int x = 0; x < dst.cols; x++ )
        {
            float* d = dst.ptr<float>(y) + x*cn;
            float sx = (float)(a[0]*x + a[1]*y + a[2]);
            float sy = (float)(a[3]*x + a[4]*y + a[5]);
            int ix = cvFloor(sx), iy = cvFloor(sy);
            int dx = cn, dy = sstep;
            const float* s;
            sx -= ix; sy -= iy;

            if( (unsigned)ix >= (unsigned)(scols-1) )
                ix = ix < 0 ? 0 : scols - 1, sx = 0, dx = 0;
            if( (unsigned)iy >= (unsigned)(srows-1) )
                iy = iy < 0 ? 0 : srows - 1, sy = 0, dy = 0;

            s = src.ptr<float>(iy) + ix*cn;
            for( int k = 0; k < cn; k++, s++ )
            {
                float t0 = s[0] + sx*(s[dx] - s[0]);
                float t1 = s[dy] + sx*(s[dy + dx] - s[dy]);
                d[k] = t0 + sy*(t1 - t0);
            }
        }
}


class CV_GetRectSubPixTest : public CV_ImgWarpBaseTest
{
public:
    CV_GetRectSubPixTest();

protected:
    void get_test_array_types_and_sizes( int test_case_idx, vector<vector<Size> >& sizes, vector<vector<int> >& types );
    void run_func();
    int prepare_test_case( int test_case_idx );
    void prepare_to_validation( int /*test_case_idx*/ );
    double get_success_error_level( int test_case_idx, int i, int j );
    void fill_array( int test_case_idx, int i, int j, Mat& arr );

    CvPoint2D32f center;
    bool test_cpp;
};


CV_GetRectSubPixTest::CV_GetRectSubPixTest() : CV_ImgWarpBaseTest( false )
{
    //spatial_scale_zoom = spatial_scale_decimate;
    spatial_scale_decimate = spatial_scale_zoom;
    test_cpp = false;
}


void CV_GetRectSubPixTest::get_test_array_types_and_sizes( int test_case_idx, vector<vector<Size> >& sizes, vector<vector<int> >& types )
{
    RNG& rng = ts->get_rng();
    CV_ImgWarpBaseTest::get_test_array_types_and_sizes( test_case_idx, sizes, types );
    int src_depth = cvtest::randInt(rng) % 2, dst_depth;
    int cn = cvtest::randInt(rng) % 2 ? 3 : 1;
    Size src_size, dst_size;

    dst_depth = src_depth = src_depth == 0 ? CV_8U : CV_32F;
    if( src_depth < CV_32F && cvtest::randInt(rng) % 2 )
        dst_depth = CV_32F;

    types[INPUT][0] = CV_MAKETYPE(src_depth,cn);
    types[INPUT_OUTPUT][0] = types[REF_INPUT_OUTPUT][0] = CV_MAKETYPE(dst_depth,cn);

    src_size = sizes[INPUT][0];
    dst_size.width = cvRound(sqrt(cvtest::randReal(rng)*src_size.width) + 1);
    dst_size.height = cvRound(sqrt(cvtest::randReal(rng)*src_size.height) + 1);
    dst_size.width = MIN(dst_size.width,src_size.width);
    dst_size.height = MIN(dst_size.width,src_size.height);
    sizes[INPUT_OUTPUT][0] = sizes[REF_INPUT_OUTPUT][0] = dst_size;

    center.x = (float)(cvtest::randReal(rng)*src_size.width);
    center.y = (float)(cvtest::randReal(rng)*src_size.height);
    interpolation = cv::INTER_LINEAR;

    test_cpp = (cvtest::randInt(rng) & 256) == 0;
}


void CV_GetRectSubPixTest::fill_array( int test_case_idx, int i, int j, Mat& arr )
{
    if( i != INPUT )
        CV_ImgWarpBaseTest::fill_array( test_case_idx, i, j, arr );
}


void CV_GetRectSubPixTest::run_func()
{
    cv::Mat _out = cv::cvarrToMat(test_array[INPUT_OUTPUT][0]);
    cv::getRectSubPix( cv::cvarrToMat(test_array[INPUT][0]), _out.size(), center, _out, _out.type());
}


double CV_GetRectSubPixTest::get_success_error_level( int /*test_case_idx*/, int /*i*/, int /*j*/ )
{
    int in_depth = test_mat[INPUT][0].depth();
    int out_depth = test_mat[INPUT_OUTPUT][0].depth();

    return in_depth >= CV_32F ? 1e-3 : out_depth >= CV_32F ? 1e-2 : 1;
}


int CV_GetRectSubPixTest::prepare_test_case( int test_case_idx )
{
    return CV_ImgWarpBaseTest::prepare_test_case( test_case_idx );
}


void CV_GetRectSubPixTest::prepare_to_validation( int /*test_case_idx*/ )
{
    Mat& src0 = test_mat[INPUT][0];
    Mat& dst0 = test_mat[REF_INPUT_OUTPUT][0];
    Mat src = src0, dst = dst0;
    int ftype = CV_MAKETYPE(CV_32F,src0.channels());
    double a[] = { 1, 0, center.x - dst.cols*0.5 + 0.5,
                   0, 1, center.y - dst.rows*0.5 + 0.5 };
    if( src.depth() != CV_32F )
        src0.convertTo(src, CV_32F);

    if( dst.depth() != CV_32F )
        dst.create(dst0.size(), ftype);

    test_getQuadrangeSubPix( src, dst, a );

    if( dst.data != dst0.data )
        dst.convertTo(dst0, dst0.depth());
}

////////////////////////////// resizeArea /////////////////////////////////

template <typename T>
static void check_resize_area(const Mat& expected, const Mat& actual, double tolerance = 1.0)
{
    ASSERT_EQ(actual.type(), expected.type());
    ASSERT_EQ(actual.size(), expected.size());

    Mat diff;
    absdiff(actual, expected, diff);

    Mat one_channel_diff = diff; //.reshape(1);

    Size dsize = actual.size();
    bool next = true;
    for (int dy = 0; dy < dsize.height && next; ++dy)
    {
        const T* eD = expected.ptr<T>(dy);
        const T* aD = actual.ptr<T>(dy);

        for (int dx = 0; dx < dsize.width && next; ++dx)
            if (fabs(static_cast<double>(aD[dx] - eD[dx])) > tolerance)
            {
                cvtest::TS::ptr()->printf(cvtest::TS::SUMMARY, "Inf norm: %f\n", static_cast<float>(cvtest::norm(actual, expected, NORM_INF)));
                cvtest::TS::ptr()->printf(cvtest::TS::SUMMARY, "Error in : (%d, %d)\n", dx, dy);

                const int radius = 3;
                int rmin = MAX(dy - radius, 0), rmax = MIN(dy + radius, dsize.height);
                int cmin = MAX(dx - radius, 0), cmax = MIN(dx + radius, dsize.width);

                std::cout << "Abs diff:" << std::endl << diff << std::endl;
                std::cout << "actual result:\n" << actual(Range(rmin, rmax), Range(cmin, cmax)) << std::endl;
                std::cout << "expected result:\n" << expected(Range(rmin, rmax), Range(cmin, cmax)) << std::endl;

                next = false;
            }
    }

    ASSERT_EQ(0, cvtest::norm(one_channel_diff, cv::NORM_INF));
}

///////////////////////////////////////////////////////////////////////////

TEST(Imgproc_fitLine_vector_3d, regression)
{
    std::vector<Point3f> points_vector;

    Point3f p21(4,4,4);
    Point3f p22(8,8,8);

    points_vector.push_back(p21);
    points_vector.push_back(p22);

    std::vector<float> line;

    cv::fitLine(points_vector, line, DIST_L2, 0 ,0 ,0);

    ASSERT_EQ(line.size(), (size_t)6);

}

TEST(Imgproc_fitLine_vector_2d, regression)
{
    std::vector<Point2f> points_vector;

    Point2f p21(4,4);
    Point2f p22(8,8);
    Point2f p23(16,16);

    points_vector.push_back(p21);
    points_vector.push_back(p22);
    points_vector.push_back(p23);

    std::vector<float> line;

    cv::fitLine(points_vector, line, DIST_L2, 0 ,0 ,0);

    ASSERT_EQ(line.size(), (size_t)4);
}

TEST(Imgproc_fitLine_Mat_2dC2, regression)
{
    cv::Mat mat1 = Mat::zeros(3, 1, CV_32SC2);
    std::vector<float> line1;

    cv::fitLine(mat1, line1, DIST_L2, 0 ,0 ,0);

    ASSERT_EQ(line1.size(), (size_t)4);
}

TEST(Imgproc_fitLine_Mat_2dC1, regression)
{
    cv::Matx<int, 3, 2> mat2;
    std::vector<float> line2;

    cv::fitLine(mat2, line2, DIST_L2, 0 ,0 ,0);

    ASSERT_EQ(line2.size(), (size_t)4);
}

TEST(Imgproc_fitLine_Mat_3dC3, regression)
{
    cv::Mat mat1 = Mat::zeros(2, 1, CV_32SC3);
    std::vector<float> line1;

    cv::fitLine(mat1, line1, DIST_L2, 0 ,0 ,0);

    ASSERT_EQ(line1.size(), (size_t)6);
}

TEST(Imgproc_fitLine_Mat_3dC1, regression)
{
    cv::Mat mat2 = Mat::zeros(2, 3, CV_32SC1);
    std::vector<float> line2;

    cv::fitLine(mat2, line2, DIST_L2, 0 ,0 ,0);

    ASSERT_EQ(line2.size(), (size_t)6);
}

TEST(Imgproc_resize_area, regression)
{
    static ushort input_data[16 * 16] = {
         90,  94,  80,   3, 231,   2, 186, 245, 188, 165,  10,  19, 201, 169,   8, 228,
         86,   5, 203, 120, 136, 185,  24,  94,  81, 150, 163, 137,  88, 105, 132, 132,
        236,  48, 250, 218,  19,  52,  54, 221, 159, 112,  45,  11, 152, 153, 112, 134,
         78, 133, 136,  83,  65,  76,  82, 250,   9, 235, 148,  26, 236, 179, 200,  50,
         99,  51, 103, 142, 201,  65, 176,  33,  49, 226, 177, 109,  46,  21,  67, 130,
         54, 125, 107, 154, 145,  51, 199, 189, 161, 142, 231, 240, 139, 162, 240,  22,
        231,  86,  79, 106,  92,  47, 146, 156,  36, 207,  71,  33,   2, 244, 221,  71,
         44, 127,  71, 177,  75, 126,  68, 119, 200, 129, 191, 251,   6, 236, 247,  6,
        133, 175,  56, 239, 147, 221, 243, 154, 242,  82, 106,  99,  77, 158,  60, 229,
          2,  42,  24, 174,  27, 198,  14, 204, 246, 251, 141,  31, 114, 163,  29, 147,
        121,  53,  74,  31, 147, 189,  42,  98, 202,  17, 228, 123, 209,  40,  77,  49,
        112, 203,  30,  12, 205,  25,  19, 106, 145, 185, 163, 201, 237, 223, 247,  38,
         33, 105, 243, 117,  92, 179, 204, 248, 160,  90,  73, 126,   2,  41, 213, 204,
          6, 124, 195, 201, 230, 187, 210, 167,  48,  79, 123, 159, 145, 218, 105, 209,
        240, 152, 136, 235, 235, 164, 157,  9,  152,  38,  27, 209, 120,  77, 238, 196,
        240, 233,  10, 241,  90,  67,  12, 79,    0,  43,  58,  27,  83, 199, 190, 182};

    static ushort expected_data[5 * 5] = {
        120, 100, 151, 101, 130,
        106, 115, 141, 130, 127,
         91, 136, 170, 114, 140,
        104, 122, 131, 147, 133,
        161, 163,  70, 107, 182
    };

    cv::Mat src(16, 16, CV_16UC1, input_data);
    cv::Mat expected(5, 5, CV_16UC1, expected_data);
    cv::Mat actual(expected.size(), expected.type());

    cv::resize(src, actual, cv::Size(), 0.3, 0.3, INTER_AREA);

    check_resize_area<ushort>(expected, actual, 1.0);
}

TEST(Imgproc_resize_area, regression_half_round)
{
    static uchar input_data[32 * 32];
    for(int i = 0; i < 32 * 32; ++i)
        input_data[i] = (uchar)(i % 2 + 253 + i / (16 * 32));

    static uchar expected_data[16 * 16];
    for(int i = 0; i < 16 * 16; ++i)
        expected_data[i] = (uchar)(254 + i / (16 * 8));

    cv::Mat src(32, 32, CV_8UC1, input_data);
    cv::Mat expected(16, 16, CV_8UC1, expected_data);
    cv::Mat actual(expected.size(), expected.type());

    cv::resize(src, actual, cv::Size(), 0.5, 0.5, INTER_AREA);

    check_resize_area<uchar>(expected, actual, 0.5);
}

TEST(Imgproc_resize_area, regression_quarter_round)
{
    static uchar input_data[32 * 32];
    for(int i = 0; i < 32 * 32; ++i)
        input_data[i] = (uchar)(i % 2 + 253 + i / (16 * 32));

    static uchar expected_data[8 * 8];
    for(int i = 0; i < 8 * 8; ++i)
        expected_data[i] = 254;

    cv::Mat src(32, 32, CV_8UC1, input_data);
    cv::Mat expected(8, 8, CV_8UC1, expected_data);
    cv::Mat actual(expected.size(), expected.type());

    cv::resize(src, actual, cv::Size(), 0.25, 0.25, INTER_AREA);

    check_resize_area<uchar>(expected, actual, 0.5);
}


//////////////////////////////////////////////////////////////////////////

TEST(Imgproc_Remap, accuracy) { CV_RemapTest test; test.safe_run(); }
TEST(Imgproc_GetRectSubPix, accuracy) { CV_GetRectSubPixTest test; test.safe_run(); }

//////////////////////////////////////////////////////////////////////////

template <typename T, typename WT>
struct IntCast
{
    T operator() (WT val) const
    {
        return cv::saturate_cast<T>(val >> 2);
    }
};

template <typename T, typename WT>
struct FltCast
{
    T operator() (WT val) const
    {
        return cv::saturate_cast<T>(val * 0.25);
    }
};

template <typename T, typename WT, int one, typename CastOp>
void resizeArea(const cv::Mat & src, cv::Mat & dst)
{
    int cn = src.channels();
    CastOp castOp;

    for (int y = 0; y < dst.rows; ++y)
    {
        const T * sptr0 = src.ptr<T>(y << 1);
        const T * sptr1 = src.ptr<T>((y << 1) + 1);
        T * dptr = dst.ptr<T>(y);

        for (int x = 0; x < dst.cols * cn; x += cn)
        {
            int x1 = x << 1;

            for (int c = 0; c < cn; ++c)
            {
                WT sum = WT(sptr0[x1 + c]) + WT(sptr0[x1 + c + cn]);
                sum += WT(sptr1[x1 + c]) + WT(sptr1[x1 + c + cn]) + (WT)(one);

                dptr[x + c] = castOp(sum);
            }
        }
    }
}

TEST(Resize, Area_half)
{
    const int size = 1000;
    int types[] = { CV_8UC1, CV_8UC4,
                    CV_16UC1, CV_16UC4,
                    CV_16SC1, CV_16SC3, CV_16SC4,
                    CV_32FC1, CV_32FC4 };

    cv::RNG rng(17);

    for (int i = 0, _size = sizeof(types) / sizeof(types[0]); i < _size; ++i)
    {
        int type = types[i], depth = CV_MAT_DEPTH(type), cn = CV_MAT_CN(type);
        const float eps = depth <= CV_32S ? 0 : 7e-5f;

        SCOPED_TRACE(depth);
        SCOPED_TRACE(cn);

        cv::Mat src(size, size, type), dst_actual(size >> 1, size >> 1, type),
            dst_reference(size >> 1, size >> 1, type);

        rng.fill(src, cv::RNG::UNIFORM, -1000, 1000, true);

        if (depth == CV_8U)
            resizeArea<uchar, ushort, 2, IntCast<uchar, ushort> >(src, dst_reference);
        else if (depth == CV_16U)
            resizeArea<ushort, uint, 2, IntCast<ushort, uint> >(src, dst_reference);
        else if (depth == CV_16S)
            resizeArea<short, int, 2, IntCast<short, int> >(src, dst_reference);
        else if (depth == CV_32F)
            resizeArea<float, float, 0, FltCast<float, float> >(src, dst_reference);
        else
            CV_Assert(0);

        cv::resize(src, dst_actual, dst_actual.size(), 0, 0, cv::INTER_AREA);

        ASSERT_GE(eps, cvtest::norm(dst_reference, dst_actual, cv::NORM_INF));
    }
}

TEST(Resize, lanczos4_regression_16192)
{
    Size src_size(11, 17);
    Size dst_size(11, 153);
    Mat src(src_size, CV_8UC3, Scalar::all(128));
    Mat dst(dst_size, CV_8UC3, Scalar::all(255));

    cv::resize(src, dst, dst_size, 0, 0, INTER_LANCZOS4);

    Mat expected(dst_size, CV_8UC3, Scalar::all(128));
    EXPECT_EQ(cvtest::norm(dst, expected, NORM_INF), 0) << dst(Rect(0,0,8,8));
}

TEST(Resize, nearest_regression_15075)
{
    const int C = 5;
    const int i1 = 5, j1 = 5;
    Size src_size(12, 12);
    Size dst_size(11, 11);

    cv::Mat src = cv::Mat::zeros(src_size, CV_8UC(C)), dst;
    for (int j = 0; j < C; j++)
        src.col(i1).row(j1).data[j] = 1;

    cv::resize(src, dst, dst_size, 0, 0, INTER_NEAREST);
    EXPECT_EQ(C, cvtest::norm(dst, NORM_L1)) << src.size;
}

TEST(Imgproc_Warp, multichannel)
{
    static const int inter_types[] = {INTER_NEAREST, INTER_AREA, INTER_CUBIC,
                                      INTER_LANCZOS4, INTER_LINEAR};
    static const int inter_n = sizeof(inter_types) / sizeof(int);

    static const int border_types[] = {BORDER_CONSTANT, BORDER_DEFAULT,
                                       BORDER_REFLECT, BORDER_REPLICATE,
                                       BORDER_WRAP, BORDER_WRAP};
    static const int border_n = sizeof(border_types) / sizeof(int);

    RNG& rng = theRNG();
    for( int iter = 0; iter < 100; iter++ )
    {
        int inter = inter_types[rng.uniform(0, inter_n)];
        int border = border_types[rng.uniform(0, border_n)];
        int width = rng.uniform(3, 333);
        int height = rng.uniform(3, 333);
        int cn = rng.uniform(1, 15);
        if(inter == INTER_CUBIC || inter == INTER_LANCZOS4)
            cn = rng.uniform(1, 5);
        Mat src(height, width, CV_8UC(cn)), dst;
        //randu(src, 0, 256);
        src.setTo(0.);

        Mat rot = getRotationMatrix2D(Point2f(0.f, 0.f), 1.0, 1.0);
        warpAffine(src, dst, rot, src.size(), inter, border);
        ASSERT_EQ(0.0, cvtest::norm(dst, NORM_INF));
        Mat rot2 = Mat::eye(3, 3, rot.type());
        rot.copyTo(rot2.rowRange(0, 2));
        warpPerspective(src, dst, rot2, src.size(), inter, border);
        ASSERT_EQ(0.0, cvtest::norm(dst, NORM_INF));
    }
}


TEST(Imgproc_Warp, regression_19566)  // valgrind should detect problem if any
{
    const Size imgSize(8192, 8);

    Mat inMat = Mat::zeros(imgSize, CV_8UC4);
    Mat outMat = Mat::zeros(imgSize, CV_8UC4);

    warpAffine(
        inMat,
        outMat,
        getRotationMatrix2D(Point2f(imgSize.width / 2.0f, imgSize.height / 2.0f), 45.0, 1.0),
        imgSize,
        INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(0.0, 0.0, 0.0, 255.0)
    );
}


TEST(Imgproc_GetAffineTransform, singularity)
{
    Point2f A_sample[3];
    A_sample[0] = Point2f(8.f, 9.f);
    A_sample[1] = Point2f(40.f, 41.f);
    A_sample[2] = Point2f(47.f, 48.f);
    Point2f B_sample[3];
    B_sample[0] = Point2f(7.37465f, 11.8295f);
    B_sample[1] = Point2f(15.0113f, 12.8994f);
    B_sample[2] = Point2f(38.9943f, 9.56297f);
    Mat trans = getAffineTransform(A_sample, B_sample);
    ASSERT_EQ(0.0, cvtest::norm(trans, NORM_INF));
}

TEST(Imgproc_Remap, DISABLED_memleak)
{
    Mat src;
    const int N = 400;
    src.create(N, N, CV_8U);
    randu(src, 0, 256);
    Mat map_x, map_y, dst;
    dst.create( src.size(), src.type() );
    map_x.create( src.size(), CV_32FC1 );
    map_y.create( src.size(), CV_32FC1 );
    randu(map_x, 0., N+0.);
    randu(map_y, 0., N+0.);

    for( int iter = 0; iter < 10000; iter++ )
    {
        if(iter % 100 == 0)
        {
            putchar('.');
            fflush(stdout);
        }
        remap(src, dst, map_x, map_y, INTER_LINEAR);
    }
}

//** @deprecated */
TEST(Imgproc_linearPolar, identity)
{
    const int N = 33;
    Mat in(N, N, CV_8UC3, Scalar(255, 0, 0));
    in(cv::Rect(N/3, N/3, N/3, N/3)).setTo(Scalar::all(255));
    cv::blur(in, in, Size(5, 5));
    cv::blur(in, in, Size(5, 5));

    Mat src = in.clone();
    Mat dst;

    Rect roi = Rect(0, 0, in.cols - ((N+19)/20), in.rows);

    for (int i = 1; i <= 5; i++)
    {
        linearPolar(src, dst,
            Point2f((N-1) * 0.5f, (N-1) * 0.5f), N * 0.5f,
            cv::WARP_FILL_OUTLIERS | cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);

        linearPolar(dst, src,
            Point2f((N-1) * 0.5f, (N-1) * 0.5f), N * 0.5f,
            cv::WARP_FILL_OUTLIERS | cv::INTER_LINEAR);

        double psnr = cvtest::PSNR(in(roi), src(roi));
        EXPECT_LE(25, psnr) << "iteration=" << i;
    }

#if 0
    Mat all(N*2+2,N*2+2, src.type(), Scalar(0,0,255));
    in.copyTo(all(Rect(0,0,N,N)));
    src.copyTo(all(Rect(0,N+1,N,N)));
    src.copyTo(all(Rect(N+1,0,N,N)));
    dst.copyTo(all(Rect(N+1,N+1,N,N)));
    imwrite("linearPolar.png", all);
    imshow("input", in); imshow("result", dst); imshow("restore", src); imshow("all", all);
    cv::waitKey();
#endif
}

//** @deprecated */
TEST(Imgproc_logPolar, identity)
{
    const int N = 33;
    Mat in(N, N, CV_8UC3, Scalar(255, 0, 0));
    in(cv::Rect(N/3, N/3, N/3, N/3)).setTo(Scalar::all(255));
    cv::blur(in, in, Size(5, 5));
    cv::blur(in, in, Size(5, 5));

    Mat src = in.clone();
    Mat dst;

    Rect roi = Rect(0, 0, in.cols - ((N+19)/20), in.rows);

    double M = N/log(N * 0.5f);
    for (int i = 1; i <= 5; i++)
    {
        logPolar(src, dst,
            Point2f((N-1) * 0.5f, (N-1) * 0.5f), M,
            WARP_FILL_OUTLIERS | INTER_LINEAR | WARP_INVERSE_MAP);

        logPolar(dst, src,
            Point2f((N-1) * 0.5f, (N-1) * 0.5f), M,
            WARP_FILL_OUTLIERS | INTER_LINEAR);

        double psnr = cvtest::PSNR(in(roi), src(roi));
        EXPECT_LE(25, psnr) << "iteration=" << i;
    }

#if 0
    Mat all(N*2+2,N*2+2, src.type(), Scalar(0,0,255));
    in.copyTo(all(Rect(0,0,N,N)));
    src.copyTo(all(Rect(0,N+1,N,N)));
    src.copyTo(all(Rect(N+1,0,N,N)));
    dst.copyTo(all(Rect(N+1,N+1,N,N)));
    imwrite("logPolar.png", all);
    imshow("input", in); imshow("result", dst); imshow("restore", src); imshow("all", all);
    cv::waitKey();
#endif
}

TEST(Imgproc_warpPolar, identity)
{
    const int N = 33;
    Mat in(N, N, CV_8UC3, Scalar(255, 0, 0));
    in(cv::Rect(N / 3, N / 3, N / 3, N / 3)).setTo(Scalar::all(255));
    cv::blur(in, in, Size(5, 5));
    cv::blur(in, in, Size(5, 5));

    Mat src = in.clone();
    Mat dst;

    Rect roi = Rect(0, 0, in.cols - ((N + 19) / 20), in.rows);
    Point2f center = Point2f((N - 1) * 0.5f, (N - 1) * 0.5f);
    double radius = N * 0.5;
    int flags = WARP_FILL_OUTLIERS | INTER_LINEAR;
    // test linearPolar
    for (int ki = 1; ki <= 5; ki++)
    {
        warpPolar(src, dst, src.size(), center, radius, flags + WARP_POLAR_LINEAR + WARP_INVERSE_MAP);
        warpPolar(dst, src, src.size(), center, radius, flags + WARP_POLAR_LINEAR);

        double psnr = cv::PSNR(in(roi), src(roi));
        EXPECT_LE(25, psnr) << "iteration=" << ki;
    }
    // test logPolar
    src = in.clone();
    for (int ki = 1; ki <= 5; ki++)
    {
        warpPolar(src, dst, src.size(),center, radius, flags + WARP_POLAR_LOG + WARP_INVERSE_MAP );
        warpPolar(dst, src, src.size(),center, radius, flags + WARP_POLAR_LOG);

        double psnr = cv::PSNR(in(roi), src(roi));
        EXPECT_LE(25, psnr) << "iteration=" << ki;
    }

#if 0
    Mat all(N*2+2,N*2+2, src.type(), Scalar(0,0,255));
    in.copyTo(all(Rect(0,0,N,N)));
    src.copyTo(all(Rect(0,N+1,N,N)));
    src.copyTo(all(Rect(N+1,0,N,N)));
    dst.copyTo(all(Rect(N+1,N+1,N,N)));
    imwrite("linearPolar.png", all);
    imshow("input", in); imshow("result", dst); imshow("restore", src); imshow("all", all);
    cv::waitKey();
#endif
}

}} // namespace
/* End of file. */
