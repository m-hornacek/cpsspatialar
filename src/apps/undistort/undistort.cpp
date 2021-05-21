/*
 *   2021 Michael Hornacek
 *   michael.hornacek@gmail.com
 *   IMW-CPS TU Vienna, Austria
 *
 *   undistort <camPath> <imPath> <outImPath>
 *
 *   Example invocations:
 *   undistort C:\Users\micha\Desktop\spatial-ar\in_out\calibrateCam\out\cam_0.yml C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\projCalib\outLeft\0.png C:\Users\micha\Desktop\spatial-ar\in_out\undistort\undistorted_0.png
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
    "{@camPath | | ...}"
    "{@imPath | | ...}"
    "{@outImPath | | ...}"
};

void help()
{
    std::cout << "undistort <camPath> <imPath> <outImPath>\n"
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

    std::string camPath = parser.get<std::string>(0);
    std::string imPath = parser.get<std::string>(1);
    std::string outImPath = parser.get<std::string>(2);

    cv::Mat im = cv::imread(imPath);

    cv::FileStorage fs;
    fs.open(camPath, cv::FileStorage::READ);

    cv::Mat K, distCoeffs;
    fs["K"] >> K;
    fs["distCoeffs"] >> distCoeffs;

    cv::Mat outIm;
    cv::undistort(im, outIm, K, distCoeffs);

    cv::imwrite(outImPath, outIm);
}
