/*
 *   2021 Michael Hornacek
 *   michael.hornacek@gmail.com
 *   IMW-CPS TU Vienna, Austria
 *
 *   rectify <camPath> <planePath> <imPath> <outImPath>
 *
 *   Example invocations:
 *   rectify C:\Users\micha\Desktop\spatial-ar\in_out\calibrateCam\out\cam_0.yml C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\plane_0.yml C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\projCalib\outLeft\0.png C:\Users\micha\Desktop\spatial-ar\in_out\rectify\rectified_0.png
 */


#include <iostream>
#include <fstream>
#include <sstream>

#include "Camera.h"
#include "Homography.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

static const char* keys =
{
    "{@camPath | | ...}"
    "{@planePath | | ...}"
    "{@imPath | | ...}"
    "{@outImPath | | ...}"
};

void help()
{
    std::cout << "./rectify <camPath> <planePath> <imPath> <outImPath>\n"
        << std::endl;
}

int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv, keys);

    if (argc < 5)
    {
        help();
        return -1;
    }

    std::string camPath = parser.get<std::string>(0);
    std::string planePath = parser.get<std::string>(1);
    std::string imPath = parser.get<std::string>(2);
    std::string outImPath = parser.get<std::string>(3);

    cv::Mat im = cv::imread(imPath);

    cv::FileStorage fsCam;
    fsCam.open(camPath, cv::FileStorage::READ);

    cv::Mat K, R, t, distCoeffs;
    fsCam["K"] >> K;
    fsCam["R"] >> R;
    fsCam["t"] >> t;
    fsCam["distCoeffs"] >> distCoeffs;

    cv::Mat undistIm;
    cv::undistort(im, undistIm, K, distCoeffs);

    Camera cam(K, R, t, im.cols, im.rows, 0.015);

    cv::FileStorage fsPlane;
    fsPlane.open(planePath, cv::FileStorage::READ);

    cv::Mat Rt;
    fsPlane["Rt"] >> Rt;

    Plane plane(Rt);

    // compute rectifying homography
    cv::Mat H, virtualR, virtualT;
    Homography::computeBirdsEyeViewHomography(plane, cam, H, virtualR, virtualT);

    cv::Mat outIm;
    cv::warpPerspective(undistIm, outIm, H, im.size());

    cv::imwrite(outImPath, outIm);
}
