/*
 *   2021 Michael Hornacek
 *   michael.hornacek@gmail.com
 *   IMW-CPS TU Vienna, Austria
 *
 *   applyHomography <homographyPath> <imPath> <outImPath> [<angleDeg>] [<width> <height>]
 * 
 *   Note that providing angleDeg (even if 0) will first scale the image so that rotating by an
 *   angle of 45 degrees will not cut out any of the image contents, and then apply the input
 *   homography; not providing angleDeg will simply apply the given homography
 *
 *   Example invocations:
 *   applyHomography C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_4.yml C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\square_image.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\square_image.png 45 958 600
 */


#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

static const char* keys =
{
    "{@homographyPath | | ...}"
    "{@imPath | | ...}"
    "{@outImPath | | ...}"
    "{@angleDeg | | ...}"
    "{@width | | ...}"
    "{@height | | ...}"
};

void help()
{
    std::cout << "./applyHomography <homographyPath> <imPath> <outImPath> [<angleDeg>] [<width> <height>]\n"
        << std::endl;
}

int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv, keys);

    if (argc < 4)
    {
        help();
        return -1;
    }

    std::string homographyPath = parser.get<std::string>(0);
    std::string imPath = parser.get<std::string>(1);
    std::string outImPath = parser.get<std::string>(2);

    cv::Mat im = cv::imread(imPath);

    int srcHeight = im.rows;
    int srcWidth = im.cols;

    int dstHeight = srcHeight;
    int dstWidth = srcWidth;

    if (argc >= 5)
    {
        float angleDeg = parser.get<float>(3);

        if (argc == 7)
        {
            dstWidth = parser.get<int>(4);
            dstHeight = parser.get<int>(5);
        }

        std::cout << dstWidth << std::endl;
        std::cout << dstHeight << std::endl;

        float halfSrcWidth = 0.5 * srcWidth;
        float halfSrcHeight = 0.5 * srcHeight;
        float halfDstWidth = 0.5 * dstWidth;
        float halfDstHeight = 0.5 * dstHeight;

        float h = std::sqrt(halfSrcWidth * halfSrcWidth + halfSrcHeight * halfSrcHeight);
        double s = std::min(halfDstWidth, halfDstHeight) / h;

        float pi = 2 * std::acos(0.0);
        double angleRad = angleDeg / 180.0 * pi;

        cv::Mat H1 = cv::Mat::eye(cv::Size(3, 3), CV_64F);
        H1.at<double>(0, 2) = -halfSrcWidth;
        H1.at<double>(1, 2) = -halfSrcHeight;

        cv::Mat H2 = cv::Mat::eye(cv::Size(3, 3), CV_64F);
        H2.at<double>(0, 0) = s * std::cos(angleRad);
        H2.at<double>(1, 0) = s * std::sin(angleRad);
        H2.at<double>(0, 1) = s * -std::sin(angleRad);
        H2.at<double>(1, 1) = s * std::cos(angleRad);

        cv::Mat H3;
        cv::gemm(H2, H1, 1.0, cv::Mat(), 0.0, H3);

        cv::Mat H4 = cv::Mat::eye(cv::Size(3, 3), CV_64F);
        H4.at<double>(0, 2) = halfDstWidth;
        H4.at<double>(1, 2) = halfDstHeight;

        cv::Mat H5;
        cv::gemm(H4, H3, 1.0, cv::Mat(), 0.0, H5);

        cv::warpPerspective(im, im, H5, cv::Size(dstWidth, dstHeight));
    }

    cv::FileStorage fs;
    fs.open(homographyPath, cv::FileStorage::READ);

    cv::Mat H;
    fs["H"] >> H;

    cv::Mat outIm;

    // finally apply the input homography
    cv::warpPerspective(im, outIm, H, cv::Size(dstWidth, dstHeight));

    cv::imwrite(outImPath, outIm);
}
