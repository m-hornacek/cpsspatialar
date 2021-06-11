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
#include "CalibPattern.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

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
    std::cout << "calibrateCam <boardSqSize> <boardDimsX> <boardDimsY> <outDir> <numIms> <cam0ImDir> [<cam1ImDir>]\n"
        << std::endl;
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
    std::string outDir = parser.get<std::string>(3);
    int numIms = parser.get<int>(4);
    std::string cam0ImDir = parser.get<std::string>(5);

    bool hasCam1 = false;
    std::string cam1ImDir;
    if (argc == 8)
    {
        hasCam1 = true;
        cam1ImDir = parser.get<std::string>(6);
    }

    cv::Size chessboardPatternSize(chessboardDimsX, chessboardDimsY);

    std::vector<std::vector<cv::Point3f>> chessboardObjectPts;
    CalibPattern::computeChessboardObjPts(numIms, chessboardSqSize, chessboardPatternSize,
        chessboardObjectPts);

    std::vector<std::vector<cv::Point2f>> cam0ChessboardImPts;
    cv::Size cam0ImSize;
    CalibPattern::findChessboardImPts(cam0ImDir, numIms, chessboardPatternSize,
        cam0ChessboardImPts, cam0ImSize);

    cv::Mat cam0K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat cam0DistCoeffs = cv::Mat::zeros(8, 1, CV_64F);

    std::vector<cv::Mat> cam0Rs, cam0Ts;

    // compute 0 camera intrinsics
    std::cout << "Computing intrinsics of cam0" << std::endl;
    std::cout << "RMS (cam0): " << cv::calibrateCamera(chessboardObjectPts, cam0ChessboardImPts, cam0ImSize,
        cam0K, cam0DistCoeffs, cam0Rs, cam0Ts, cv::CALIB_FIX_ASPECT_RATIO) << std::endl << std::endl;

    cv::Mat cam0R = cv::Mat::eye(cv::Size(3, 3), CV_64F);
    cv::Mat cam0T = cv::Mat::zeros(cv::Size(3, 1), CV_64F);

    if (hasCam1)
    {
        std::vector<std::vector<cv::Point2f>> cam1ChessboardImPts;
        cv::Size cam1ImSize;
        CalibPattern::findChessboardImPts(cam1ImDir, numIms, chessboardPatternSize,
            cam1ChessboardImPts, cam1ImSize);

        cv::Mat cam1K = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat cam1DistCoeffs = cv::Mat::zeros(8, 1, CV_64F);

        std::vector<cv::Mat> cam1Rs, cam1Ts;

        // compute 1 camera intrinsics
        std::cout << "Computing intrinsics of cam1" << std::endl;
        std::cout << "RMS (cam1): " << calibrateCamera(chessboardObjectPts, cam1ChessboardImPts, cam1ImSize,
            cam1K, cam1DistCoeffs, cam1Rs, cam1Ts, cv::CALIB_FIX_ASPECT_RATIO) << std::endl << std::endl;

        cv::Mat cam1FundamentalMatrix, cam1EssentialMatrix;
        cv::Mat cam1R, cam1T;

        // compute pose (cam1R, cam1T) of cam1 w.r.t. cam0
        std::cout << "Computing stereo calibration beween cam0 and cam1" << std::endl;
        std::cout << "RMS (stereo): " << cv::stereoCalibrate(chessboardObjectPts,
            cam0ChessboardImPts, cam1ChessboardImPts,
            cam0K, cam0DistCoeffs,
            cam1K, cam1DistCoeffs,
            cam0ImSize,
            cam1R, cam1T,
            cam1FundamentalMatrix, cam1EssentialMatrix,
            cv::CALIB_USE_INTRINSIC_GUESS) << std::endl << std::endl;

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
}
