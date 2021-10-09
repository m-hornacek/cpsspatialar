 #include "CalibPattern.h"

void CalibPattern::computeChessboardObjPts(int numIms, float chessboardSquareSize, cv::Size chessboardPatternSize,
    std::vector<std::vector<cv::Point3f>>& outChessboardObjectPts)
{
    // compute chessboard object points
    std::vector<cv::Point3f> chessboardObjectPts_;
    for (int i = 0; i < chessboardPatternSize.height; i++)
        for (int j = 0; j < chessboardPatternSize.width; j++)
            chessboardObjectPts_.push_back(
                cv::Point3f(j * chessboardSquareSize, i * chessboardSquareSize, 0.0));

    // store copy of chessboard object points once per input image
    for (int numIm = 0; numIm < numIms; numIm++)
        outChessboardObjectPts.push_back(chessboardObjectPts_);
}

void CalibPattern::findChessboardImPts(cv::Mat& im, cv::Size chessboardPatternSize,
    std::vector<std::vector<cv::Point2f>>& outChessboardImPts, cv::Size& outImSize, cv::Mat& outImVis,
    bool applyIntrinsics, cv::Mat& K, cv::Mat& distCoeffs)
{
    cv::Mat imUndistort, imGrayUndistort;

    if (applyIntrinsics)
    {
        cv::undistort(im, imUndistort, K, distCoeffs);
    }
    else
        im.copyTo(imUndistort);

    if (outImVis.size != imUndistort.size)
        imUndistort.copyTo(outImVis);

    cv::cvtColor(imUndistort, imGrayUndistort, cv::COLOR_BGR2GRAY);
    // cv::threshold(candidateIm, candidateImThresh, 100, 255, THRESH_BINARY);

    //cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    //clahe->setClipLimit(4);
    //clahe->apply(imGrayUndistort, imGrayUndistort);

    outImSize = cv::Size(im.cols, im.rows);

    std::vector<cv::Point2f> candidateChessboardImPts;
    if (findChessboardCorners(imGrayUndistort, chessboardPatternSize, candidateChessboardImPts, cv::CALIB_CB_ADAPTIVE_THRESH))
    {
        cv::cornerSubPix(imGrayUndistort, candidateChessboardImPts, cv::Size(5, 5), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001));

        drawChessboardCorners(outImVis, chessboardPatternSize, candidateChessboardImPts, true);
        outChessboardImPts.push_back(candidateChessboardImPts);
    }
}

void CalibPattern::findChessboardImPts(std::string inDir, int numIms, cv::Size chessboardPatternSize,
    std::vector<std::vector<cv::Point2f>>& outChessboardImPts, cv::Size& outImSize,
    bool applyIntrinsics, cv::Mat& K, cv::Mat& distCoeffs)
{
    for (int numIm = 0; numIm < numIms; numIm++)
    {
        std::stringstream ss1;
        ss1 << inDir << "\\" << numIm << ".png";
        std::cout << ss1.str() << std::endl;

        cv::Mat imVis;
        findChessboardImPts(cv::imread(ss1.str()), chessboardPatternSize,
            outChessboardImPts, outImSize, imVis,
            applyIntrinsics, K, distCoeffs);

        std::stringstream ss2;
        ss2 << "camera calibration image (50% resized)";

        cv::resize(imVis, imVis, cv::Size(imVis.cols * 0.5, imVis.rows * 0.5));

        cv::imshow(ss2.str(), imVis);
        cv::waitKey(100);
    }
}

void CalibPattern::findCirclesImPts(cv::Mat& im, cv::Size circlesPatternSize,
    std::vector<std::vector<cv::Point2f>>& outCirclesImPts, cv::Size& outImSize, cv::Mat& outImVis,
    bool applyIntrinsics, cv::Mat& K, cv::Mat& distCoeffs)
{
    cv::Mat imUndistort, imGrayUndistort, imGrayUndistortInvert;

    if (applyIntrinsics)
        cv::undistort(im, imUndistort, K, distCoeffs);
    else
        im.copyTo(imUndistort);

    if (outImVis.size != imUndistort.size)
        imUndistort.copyTo(outImVis);

    cv::cvtColor(imUndistort, imGrayUndistort, cv::COLOR_BGR2GRAY);

    cv::bitwise_not(imGrayUndistort, imGrayUndistortInvert);

    outImSize = cv::Size(im.cols, im.rows);

    cv::Mat imGrayUndistortInvertVis;
    cv::resize(imGrayUndistortInvert, imGrayUndistortInvertVis, cv::Size(imGrayUndistortInvert.cols * 0.5, imGrayUndistortInvert.rows * 0.5));
    cv::imshow("imGrayUndistortInvert", imGrayUndistortInvertVis);
    cv::waitKey(10);

    std::vector<cv::Point2f> candidateCirclesImPts;
    if (findCirclesGrid(imGrayUndistortInvert, circlesPatternSize, candidateCirclesImPts, cv::CALIB_CB_ASYMMETRIC_GRID))
    {
        cv::cornerSubPix(imGrayUndistort, candidateCirclesImPts, cv::Size(5, 5), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001));

        drawChessboardCorners(outImVis, circlesPatternSize, candidateCirclesImPts, true);
        outCirclesImPts.push_back(candidateCirclesImPts);
    }
}

void CalibPattern::findChessboardAndCirclesImPts(std::string inDir, int numIms, cv::Size chessboardPatternSize, cv::Size circlesPatternSize,
    std::vector<std::vector<cv::Point2f>>& outChessboardImPts, std::vector<std::vector<cv::Point2f>>& outCirclesImPts, cv::Size& outImSize,
    bool applyIntrinsics, cv::Mat& K, cv::Mat& distCoeffs, cv::Mat& imVis, int visImIdx)
{
    for (int numIm = 0; numIm < numIms; numIm++)
    {
        std::stringstream ss1;
        ss1 << inDir << "\\" << numIm << ".png";
        std::cout << ss1.str() << std::endl;

        cv::Mat imVis_;
        findChessboardImPts(cv::imread(ss1.str()), chessboardPatternSize,
            outChessboardImPts, outImSize, imVis_,
            applyIntrinsics, K, distCoeffs);

        std::cout << "findChessboardImPts done" << std::endl;

        findCirclesImPts(cv::imread(ss1.str()), circlesPatternSize,
            outCirclesImPts, outImSize, imVis_,
            applyIntrinsics, K, distCoeffs);

        std::cout << "findCirclesImPts done" << std::endl;

        if (numIm == visImIdx)
            imVis_.copyTo(imVis);

        std::stringstream ss2;
        ss2 << "projector calibration image (50% resized)";

        cv::resize(imVis_, imVis_, cv::Size(imVis_.cols * 0.5, imVis_.rows * 0.5));

        cv::imshow(ss2.str(), imVis_);
        cv::waitKey(100);
    }
}

