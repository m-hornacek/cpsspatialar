/*
 *   2021 Michael Hornacek
 *   michael.hornacek@gmail.com
 *   IMW-CPS TU Vienna, Austria
 *
 *   applyHomography <homographyPath> <imPath> <outImPath>
 *
 *   Example invocations:
 *   applyHomography C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_4.yml C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\acircles_pattern_960x600.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\4_warped.png
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
};

void help()
{
    std::cout << "./applyHomography <homographyPath> <imPath> <outImPath>\n"
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

    cv::FileStorage fs;
    fs.open(homographyPath, cv::FileStorage::READ);

    cv::Mat H;
    fs["H"] >> H;

    cv::Mat outIm;
    cv::warpPerspective(im, outIm, H, im.size());

    cv::imwrite(outImPath, outIm);
}
