#ifndef CALIBPATTERN
#define CALIBPATTERN

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

class CalibPattern
{
public:
    static void computeChessboardObjPts(int numIms, float chessboardSquareSize, cv::Size chessboardPatternSize,
        std::vector<std::vector<cv::Point3f>>& outChessboardObjectPts);
    static void findChessboardImPts(cv::Mat& im, cv::Size chessboardPatternSize,
        std::vector<std::vector<cv::Point2f>>& outChessboardImPts, cv::Size& outImSize, cv::Mat& outImVis,
        bool applyIntrinsics = false, cv::Mat& K = cv::Mat(), cv::Mat& distCoeffs = cv::Mat());
    static void findChessboardImPts(std::string inDir, int numIms, cv::Size chessboardPatternSize,
        std::vector<std::vector<cv::Point2f>>& outChessboardImPts, cv::Size& outImSize,
        bool applyIntrinsics = false, cv::Mat& K = cv::Mat(), cv::Mat& distCoeffs = cv::Mat());
    static void findCirclesImPts(cv::Mat& im, cv::Size circlesPatternSize,
        std::vector<std::vector<cv::Point2f>>& outCirclesImPts, cv::Size& outImSize, cv::Mat& outImVis,
        bool applyIntrinsics = true, cv::Mat& K = cv::Mat(), cv::Mat& distCoeffs = cv::Mat());
    static void findChessboardAndCirclesImPts(std::string inDir, int numIms, cv::Size chessboardPatternSize, cv::Size circlesPatternSize,
        std::vector<std::vector<cv::Point2f>>& outChessboardImPts, std::vector<std::vector<cv::Point2f>>& outCirclesImPts, cv::Size& outImSize,
        bool applyIntrinsics = false, cv::Mat& K = cv::Mat(), cv::Mat& distCoeffs = cv::Mat(), cv::Mat& imVis = cv::Mat(), int visImIdx = 0);

private:

};

#endif
 