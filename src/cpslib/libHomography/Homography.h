#ifndef HOMOGRAPHY
#define HOMOGRAPHY

#include "Plane.h"
#include "Camera.h"

class Homography
{
public:
	static void computeBirdsEyeViewHomography(Plane& plane, Camera& proj, cv::Mat& outH, cv::Mat& outR, cv::Mat& outT);
	static void computeBirdsEyeViewVirtualCam(Plane& plane, Camera& cam, float verticalOffset, cv::Mat& outVirtualCamR, cv::Mat& outVirtualCamT);
	static void computeBirdsEyeViewVirtualProjAlignedWithVirtualCam(Plane& plane, Camera& cam, Camera& proj, float verticalOffset, cv::Mat& outCamR, cv::Mat& outCamT, cv::Mat& outProjR, cv::Mat& outProjT);
	static void computePlaneInducedHomography(Plane& plane, Camera& cam0, Camera& cam1, cv::Mat& outH);

private:


};

#endif
