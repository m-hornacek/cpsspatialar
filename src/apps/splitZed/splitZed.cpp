/*
 *   2021 Michael Hornacek
 *   michael.hornacek@gmail.com
 *   IMW-CPS TU Vienna, Austria
 *
 *   splitZed <inPattern> <leftCamOutDir> <rightCamOutDir>
 *
 *   Example invocations:
 *   splitZed C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\stereoCalib\in\*.png C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\stereoCalib\outLeft C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\stereoCalib\outRight
 *   splitZed C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\projCalib\in\*.png C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\projCalib\outLeft C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\projCalib\outRight
 */


#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

#include "headers.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

int imWidth, imHeight;

int numPatterns;

static const char* keys =
{
    "{@inPattern | | ...}"
    "{@leftCamOutDir | | ...}"
    "{@rightCamOutDir | | ...}"
};

void help()
{
    std::cout << "./splitZed <inPattern> <leftCamOutDir> <rightCamOutDir>\n"
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

    String inPattern = parser.get<String>(0);
    String leftCamOutDir = parser.get<String>(1);
    String rightCamOutDir = parser.get<String>(2);

    std::vector<cv::String> inPaths;
    cv::glob(inPattern, inPaths, false);

    size_t count = inPaths.size();
    for (size_t i = 0; i < count; i++)
    {
        cv::Mat im = cv::imread(inPaths[i]);

        int width = im.cols * 0.5;
        int height = im.rows;

        cv::Mat imLeft = im(cv::Rect(0, 0, width, height));
        cv::Mat imRight = im(cv::Rect(width, 0, width, height));

        std::stringstream ssLeft;
        ssLeft << leftCamOutDir << "\\" << i << ".png";
        cv::imwrite(ssLeft.str(), imLeft);

        std::stringstream ssRight;
        ssRight << rightCamOutDir << "\\" << i << ".png";
        cv::imwrite(ssRight.str(), imRight);
    }
 }
