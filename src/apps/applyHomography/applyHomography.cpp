/*
 *   2021 Michael Hornacek
 *   michael.hornacek@gmail.com
 *   IMW-CPS TU Vienna, Austria
 *
 *   applyHomography <homographyPath> <inPath> <outPath> <scalingFactor> [<angleDeg> <width> <height>]
 *
 *   Note that providing angleDeg (even if 0) will first scale the image so that rotating by an
 *   angle of 45 degrees with respect to the target dimensions of width, height will not cut out
 *   any of the input image contents, and then apply the input homography; not providing angleDeg,
 *   width, and height will simply apply the given homography
 *
 *   Note additionally that if a video is provided, it must have extension .avi or .mp4 (case sensitive!)
 *
 *   Example invocations:
 *   applyHomography C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_4.yml C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\square_image.png C:\Users\micha\Desktop\spatial-ar\in_out\applyHomography\square_image.png 1.0 45 958 600
 *   applyHomography C:\Users\micha\Desktop\spatial-ar\in_out\calibrateProj\out\homography_4.yml C:\Users\micha\Desktop\02_Holographie_GIF\03_GIF_Final.avi C:\Users\micha\Desktop\02_Holographie_GIF\03_GIF_Final_warped.avi 3.0 45 958 600
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
    "{@inPath | | ...}"
    "{@outPath | | ...}"
    "{@scalingFactor | | ...}"
    "{@angleDeg | | ...}"
    "{@width | | ...}"
    "{@height | | ...}"
};

void help()
{
    std::cout << "applyHomography <homographyPath> <inPath> <outPath> <scalingFactor> [<angleDeg> <width> <height>]\n"
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
    std::string inPath = parser.get<std::string>(1);
    std::string outPath = parser.get<std::string>(2);
    float scalingFactor = parser.get<float>(3);

    std::string ext = inPath.substr(inPath.find_last_of("."));

    bool isVideo = false;
    if (ext == ".avi" || ext == ".mp4")
        isVideo = true;

    cv::VideoCapture cap;
    if (isVideo)
        cap = cv::VideoCapture(inPath);

    bool hasVideoWriter = false;
    cv::VideoWriter videoWriter;

    cv::Mat outIm;
    while (true)
    {
        cv::Mat im;
        if (isVideo)
            cap >> im;
        else
            im = cv::imread(inPath);

        if (im.empty())
            break;

        int srcHeight = im.rows;
        int srcWidth = im.cols;

        int dstHeight = srcHeight;
        int dstWidth = srcWidth;

        if (argc >= 6)
        {
            if (argc != 8)
            {
                help();
                return -1;
            }

            float angleDeg = parser.get<float>(4);

            dstWidth = parser.get<int>(5);
            dstHeight = parser.get<int>(6);

            float halfSrcWidth = 0.5 * srcWidth;
            float halfSrcHeight = 0.5 * srcHeight;
            float halfDstWidth = 0.5 * dstWidth;
            float halfDstHeight = 0.5 * dstHeight;

            float h = std::sqrt(halfSrcWidth * halfSrcWidth + halfSrcHeight * halfSrcHeight);
            double s = scalingFactor * std::min(halfDstWidth, halfDstHeight) / h;

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

        // finally apply the input homography
        cv::warpPerspective(im, outIm, H, cv::Size(dstWidth, dstHeight));

        if (isVideo)
        {
            if (!hasVideoWriter)
            {
                videoWriter = cv::VideoWriter(outPath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10,
                    cv::Size(dstWidth, dstHeight));
                hasVideoWriter = true;
            }

            videoWriter.write(outIm);
        }

        cv::imshow("warped", outIm);
        cv::waitKey(100);

        if (!isVideo)
            break;
    }

    if (isVideo)
    {
        cap.release();
        videoWriter.release();
    }
    else
        cv::imwrite(outPath, outIm);
}