#ifndef CAMERA
#define CAMERA
#include "headers.h"

class Camera
{
public:
	Camera(); // default constructor
	Camera(const Camera& cam); // copy constructor
	Camera(cv::Mat& K43d, cv::Mat& Rt34d, int width, int height, float CCDWidth = -1);
	Camera(cv::Mat& K33d, cv::Mat& R33d, cv::Mat& T31d, int width, int height, float CCDWidth = -1);

	double getf() { return f_; }

	cv::Vec2d& getPrincipalPt() { return principalPt_; }
	cv::Vec3d& getLookDir() { return lookDir_; };

	cv::Vec3d& getC() { return C_; }; // camera center
	cv::Mat& getP() { return P34d_; }; // complete camera projection matrix
	cv::Mat& getM() { return M33d_; }; // M = KR
	cv::Mat& getMInv() { return MInv33d_; }; // M inverse
	cv::Mat& getK() { return K33d_; }; // camera calibration matrix
	cv::Mat& getKInv() { return KInv33d_; };
	cv::Mat& getR() { return R33d_; }; // camera pose rotation matrix
	cv::Vec3d& gett() { return t_; }; // camera pose rigid body translation vector (applied after rotation)
	cv::Mat& getRt34() { return Rt34d_; };
	cv::Mat& getRt44() { return Rt44d_; };
	cv::Mat& getRt44Inv() { return RtInv44d_; };

	int getWidth() { return width_; };
	int getHeight() { return height_; };

	void scale(float imScalingFactor); // scale camera's pixel resolution
	void rotate(cv::Mat rotToApply33d); // rotate camera about the camera center

	cv::Vec2f projectLocal(cv::Vec3f ptInLocalCoordFrame);
	cv::Vec2f projectWorld(cv::Vec3f ptInWorldCoordFrame);

	cv::Vec3f backprojectLocal(cv::Vec2f pixel); // KInv * (pixel, 1)^T

	void displayLocal();
	void displayLocal(float r, float g, float b);

	void displayWorld();
	void displayWorld(float r, float g, float b);

	~Camera();

private:
	void initVariables();
	void initDisplayList();

	bool hasDisplayList_;
	GLuint displayList_;

	double f_;
	cv::Vec2d principalPt_;
	cv::Vec3d lookDir_;

	cv::Vec3d C_;
	cv::Mat P34d_;
	cv::Mat M33d_;
	cv::Mat MInv33d_;
	cv::Mat K33d_;
	cv::Mat KInv33d_;
	cv::Mat R33d_;
	cv::Vec3d t_;
	cv::Mat Rt34d_;
	cv::Mat Rt44d_;
	cv::Mat RtInv44d_;

	int width_, height_;
	float f_mm_, CCDWidth_, CCDHeight_; // optional; used only for visualization
};

#endif
