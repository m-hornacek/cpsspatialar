/*
 *   2021 Michael Hornacek
 *   michael.hornacek@gmail.com
 *   IMW-CPS TU Vienna, Austria
 *
 *   prepareIm <angleDeg> <imPath> <outImPath>
 *
 *   Example invocations:
 *   prepareIm 0 C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\holodeck.png C:\Users\micha\Desktop\spatial-ar\in_out\prepareIm\holodeck.png
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
    "{@angleDeg | | ...}"
    "{@imPath | | ...}"
    "{@outImPath | | ...}"
};

void help()
{
    std::cout << "./prepareIm <imPath> <outImPath>\n"
        << std::endl;
}

int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv, keys);

    if (argc < 3)
    {
        help();
        return -1;
    }

    float angleDeg = parser.get<float>(0);
    std::string imPath = parser.get<std::string>(1);
    std::string outImPath = parser.get<std::string>(2);

    cv::Mat im = cv::imread(imPath);

    double halfHeight = 0.5 * im.rows;
    double halfWidth = 0.5 * im.cols;

    double h = std::sqrt(halfHeight * halfHeight + halfWidth * halfWidth);
    double s = halfHeight / h;

    double pi = 2 * std::acos(0.0);
    double angleRad = angleDeg / 180.0 * pi;

    cv::Mat H1 = cv::Mat::eye(cv::Size(3, 3), CV_64F);
    H1.at<double>(0, 2) = -halfWidth;
    H1.at<double>(1, 2) = -halfHeight;

    cv::Mat H2 = cv::Mat::eye(cv::Size(3, 3), CV_64F);
    H2.at<double>(0, 0) = s * std::cos(angleRad);
    H2.at<double>(1, 0) = s * std::sin(angleRad);
    H2.at<double>(0, 1) = s * -std::sin(angleRad);
    H2.at<double>(1, 1) = s * std::cos(angleRad);

    cv::Mat H3;
    cv::gemm(H2, H1, 1.0, cv::Mat(), 0.0, H3);

    cv::Mat H4 = cv::Mat::eye(cv::Size(3, 3), CV_64F);
    H4.at<double>(0, 2) = halfWidth;
    H4.at<double>(1, 2) = halfHeight;

    cv::Mat H5;
    cv::gemm(H4, H3, 1.0, cv::Mat(), 0.0, H5);

    cv::Mat outIm;
    cv::warpPerspective(im, outIm, H5, im.size());

    cv::imwrite(outImPath, outIm);
}
