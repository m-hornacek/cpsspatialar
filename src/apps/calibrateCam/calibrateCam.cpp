/*
 *   2021 Michael Hornacek
 *   michael.hornacek@gmail.com
 *   IMW-CPS TU Vienna, Austria
 *
 *   calibrateCam <boardSqSize> <boardDimsX> <boardDimsY> <outDir> <numIms> <cam0ImDir> [<cam1ImDir>]
 *
 *   Example invocation:
 *   calibrateCam 0.0565 4 6 C:\Users\micha\Desktop\spatial-ar\in_out\calibrateCam\out 15 C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\stereoCalib\outLeft C:\Users\micha\Desktop\spatial-ar\in_out\splitZed\stereoCalib\outRight
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
    "{@boardSqSize | | ...}"
    "{@boardDimsX | | ...}"
    "{@boardDimsY | | ...}"
    "{@outDir | | ...}"
    "{@numIms | | ...}"
    "{@cam0ImDir | | ...}"
    "{@cam1ImDir | | ...}"
};

void help()
{
    std::cout << "./calibrateCam <boardSqSize> <boardDimsX> <boardDimsY> <outDir> <numIms> <cam0ImDir> [<cam1ImDir>]\n"
        << std::endl;
}

void findChessboardImPts(cv::Mat& im, cv::Size chessboardPatternSize,
    std::vector<std::vector<cv::Point2f>>& outChessboardImPts, cv::Size& outImSize, cv::Mat& outImVis,
    cv::Mat& K = cv::Mat(), cv::Mat& distCoeffs = cv::Mat())
{
    cv::Mat imGray;

    if (outImVis.size != im.size)
        im.copyTo(outImVis);

    cv::cvtColor(im, imGray, cv::COLOR_BGR2GRAY);
    // cv::threshold(candidateIm, candidateImThresh, 100, 255, THRESH_BINARY);

    outImSize = cv::Size(im.cols, im.rows);

    std::vector<cv::Point2f> candidateChessboardImPts;
    if (findChessboardCorners(imGray, chessboardPatternSize, candidateChessboardImPts, cv::CALIB_CB_ADAPTIVE_THRESH))
    {
        cv::cornerSubPix(imGray, candidateChessboardImPts, cv::Size(5, 5), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001));

        drawChessboardCorners(outImVis, chessboardPatternSize, candidateChessboardImPts, true);
        outChessboardImPts.push_back(candidateChessboardImPts);
    }
}

void findChessboardImPts(String windowTitle, String inDir, int numIms, cv::Size chessboardPatternSize,
    std::vector<std::vector<cv::Point2f>>& outChessboardImPts, cv::Size& outImSize,
    cv::Mat& K = cv::Mat(), cv::Mat& distCoeffs = cv::Mat())
{
    for (int numIm = 0; numIm < numIms; numIm++)
    {
        stringstream ss1;
        ss1 << inDir << "\\" << numIm << ".png";

        cv::Mat im = cv::imread(ss1.str());

        cv::Mat imVis;
        findChessboardImPts(im, chessboardPatternSize,
            outChessboardImPts, outImSize, imVis,
            K, distCoeffs);

        std::stringstream ss;
        ss << windowTitle << " (50% resized)";

        cv::resize(imVis, imVis, cv::Size(imVis.cols * 0.5, imVis.rows * 0.5));

        cv::imshow(ss.str(), imVis);
        cv::waitKey(10);
    }
}

int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv, keys);

    if (argc < 7)
    {
        help();
        return -1;
    }

    float chessboardSqSize = parser.get<float>(0);
    int chessboardDimsX = parser.get<int>(1);
    int chessboardDimsY = parser.get<int>(2);
    String outDir = parser.get<String>(3);
    int numIms = parser.get<int>(4);
    String cam0ImDir = parser.get<String>(5);

    bool hasCam1 = false;
    String cam1ImDir;
    if (argc == 8)
    {
        hasCam1 = true;
        cam1ImDir = parser.get<String>(6);
    }

    cv::Size chessboardPatternSize(chessboardDimsX, chessboardDimsY);

    // compute chessboard object points
    std::vector<cv::Point3f> chessboardObjectPts_;
    for (int i = 0; i < chessboardPatternSize.height; i++)
        for (int j = 0; j < chessboardPatternSize.width; j++)
            chessboardObjectPts_.push_back(
                cv::Point3f(float(j * chessboardSqSize), float(i * chessboardSqSize), 0));

    // store copy of chessboard object points once per input image
    std::vector<std::vector<cv::Point3f>> chessboardObjectPts;
    for (int numIm = 0; numIm < numIms; numIm++)
        chessboardObjectPts.push_back(chessboardObjectPts_);

    std::vector<std::vector<cv::Point2f>> cam0ChessboardImPts;
    cv::Size cam0ImSize;
    findChessboardImPts("cam0", cam0ImDir, numIms, chessboardPatternSize,
        cam0ChessboardImPts, cam0ImSize);

    std::cout << "finished findChessboardImPts" << std::endl;

    cv::Mat cam0K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat cam0DistCoeffs = cv::Mat::zeros(8, 1, CV_64F);

    std::vector<cv::Mat> cam0Rs, cam0Ts;

    // compute 0 camera intrinsics
    std::cout << "Computing intrinsics of cam0" << std::endl;
    std::cout << "RMS: " << cv::calibrateCamera(chessboardObjectPts, cam0ChessboardImPts, cam0ImSize,
        cam0K, cam0DistCoeffs, cam0Rs, cam0Ts, cv::CALIB_FIX_ASPECT_RATIO) << std::endl << std::endl;

    cv::Mat cam0R = cv::Mat::eye(cv::Size(3, 3), CV_64F);
    cv::Mat cam0T = cv::Mat::zeros(cv::Size(3, 1), CV_64F);

    stringstream ssCam0;
    ssCam0 << outDir << "\\cam_0.yml";
    cv::FileStorage fsCam0(ssCam0.str(), cv::FileStorage::WRITE);

    fsCam0 << "K" << cam0K;
    fsCam0 << "R" << cam0R;
    fsCam0 << "t" << cam0T;
    fsCam0 << "width" << cam0ImSize.width;
    fsCam0 << "height" << cam0ImSize.height;
    fsCam0 << "distCoeffs" << cam0DistCoeffs;

    fsCam0.release();

    if (hasCam1)
    {
        vector<vector<Point2f>> cam1ChessboardImPts;
        cv::Size cam1ImSize;
        findChessboardImPts("cam1", cam1ImDir, numIms, chessboardPatternSize,
            cam1ChessboardImPts, cam1ImSize);

        cv::Mat cam1K = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat cam1DistCoeffs = cv::Mat::zeros(8, 1, CV_64F);

        std::vector<cv::Mat> cam1Rs, cam1Ts;

        // compute 1 camera intrinsics
        std::cout << "Computing intrinsics of cam1" << std::endl;
        std::cout << "RMS: " << calibrateCamera(chessboardObjectPts, cam1ChessboardImPts, cam1ImSize,
            cam1K, cam1DistCoeffs, cam1Rs, cam1Ts, CALIB_FIX_ASPECT_RATIO) << std::endl << std::endl;

        std::cout << cam1K << std::endl << std::endl;

        cv::Mat cam1FundamentalMatrix, cam1EssentialMatrix;
        cv::Mat cam1R, cam1T;

        // compute pose (cam1R, cam1T) of cam1 w.r.t. cam0
        std::cout << "Computing pose of cam1 relative to cam0" << std::endl;
        std::cout << "RMS: " << cv::stereoCalibrate(chessboardObjectPts,
            cam0ChessboardImPts, cam1ChessboardImPts,
            cam0K, cam0DistCoeffs,
            cam1K, cam1DistCoeffs,
            cam0ImSize,
            cam1R, cam1T,
            cam1FundamentalMatrix, cam1EssentialMatrix,
            cv::CALIB_USE_INTRINSIC_GUESS) << std::endl << std::endl;

        std::cout << cam1R << std::endl;
        std::cout << cam1T << std::endl;

        stringstream ssCam1;
        ssCam1 << outDir << "\\cam_1.yml";
        cv::FileStorage fsCam1(ssCam1.str(), cv::FileStorage::WRITE);

        fsCam1 << "K" << cam1K;
        fsCam1 << "R" << cam1R;
        fsCam1 << "t" << cam1T;
        fsCam1 << "width" << cam1ImSize.width;
        fsCam1 << "height" << cam1ImSize.height;
        fsCam1 << "distCoeffs" << cam1DistCoeffs;

        fsCam1.release();
    }
}
