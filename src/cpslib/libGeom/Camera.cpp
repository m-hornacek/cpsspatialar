#include "Camera.h"

Camera::Camera()
{
	f_mm_ = CCDWidth_ = CCDHeight_ = -1;

	P34d_ = cv::Mat(3, 4, CV_64F);

	principalPt_ = cv::Vec2d();
	K33d_ = cv::Mat(3, 3, CV_64F);
	KInv33d_ = cv::Mat(3, 3, CV_64F);
	R33d_ = cv::Mat(3, 3, CV_64F);
	M33d_ = cv::Mat(3, 3, CV_64F);
	MInv33d_ = cv::Mat(3, 3, CV_64F);
	C_ = cv::Vec3d();
	lookDir_ = cv::Vec3d();
	t_ = cv::Vec3d();
	Rt34d_ = cv::Mat(3, 4, CV_64F);
	Rt44d_ = cv::Mat(4, 4, CV_64F);
	RtInv44d_ = cv::Mat(4, 4, CV_64F);

	hasDisplayList_ = false;
}

Camera::Camera(const Camera& cam)
{
	width_ = cam.width_;
	height_ = cam.height_;
	CCDWidth_ = cam.CCDWidth_;
	CCDHeight_ = cam.CCDHeight_;

	f_ = cam.f_;
	f_mm_ = cam.f_mm_;

	P34d_ = cv::Mat(3, 4, CV_64F);
	principalPt_ = cv::Vec2d();
	K33d_ = cv::Mat(3, 3, CV_64F);
	KInv33d_ = cv::Mat(3, 3, CV_64F);
	R33d_ = cv::Mat(3, 3, CV_64F);
	M33d_ = cv::Mat(3, 3, CV_64F);
	MInv33d_ = cv::Mat(3, 3, CV_64F);
	C_ = cv::Vec3d();
	lookDir_ = cv::Vec3d();
	t_ = cv::Vec3d();
	Rt34d_ = cv::Mat(3, 4, CV_64F);
	Rt44d_ = cv::Mat(4, 4, CV_64F);
	RtInv44d_ = cv::Mat(4, 4, CV_64F);

	cam.P34d_.copyTo(P34d_);
	cam.K33d_.copyTo(K33d_);
	cam.KInv33d_.copyTo(KInv33d_);
	cam.R33d_.copyTo(R33d_);
	cam.M33d_.copyTo(M33d_);
	cam.MInv33d_.copyTo(MInv33d_);
	cam.Rt34d_.copyTo(Rt34d_);
	cam.Rt44d_.copyTo(Rt44d_);
	cam.RtInv44d_.copyTo(RtInv44d_);

	principalPt_ = cv::Vec2d((cam.principalPt_)[0], (cam.principalPt_)[1]);
	lookDir_ = cv::Vec3d((cam.lookDir_)[0], (cam.lookDir_)[1], (cam.lookDir_)[2]);
	C_ = cv::Vec3d((cam.C_)[0], (cam.C_)[1], (cam.C_)[2]);
	t_ = cv::Vec3d((cam.t_)[0], (cam.t_)[1], (cam.t_)[2]);

	hasDisplayList_ = false;
	//initVariables();
}

//Camera::Camera(const Camera& cam)
//{
//	f_ = cam.f_;
//	f_mm_ = cam.f_mm_;
//
//	width_ = cam.width_;
//	height_ = cam.height_;
//	CCDWidth_ = cam.CCDWidth_;
//	CCDHeight_ = cam.CCDHeight_;
//
//	P34d_ = cv::Mat(3, 4, CV_64F);
//	K33d_ = cv::Mat(3, 3, CV_64F);
//	KInv33d_ = cv::Mat(3, 3, CV_64F);
//	R33d_ = cv::Mat(3, 3, CV_64F);
//	M33d_ = cv::Mat(3, 3, CV_64F);
//	MInv33d_ = cv::Mat(3, 3, CV_64F);
//	Rt34d_ = cv::Mat(3, 4, CV_64F);
//	Rt44d_ = cv::Mat(4, 4, CV_64F);
//	RtInv44d_ = cv::Mat(4, 4, CV_64F);
//
//	cam.K33d_.copyTo(K33d_);
//	cam.Rt34d_.copyTo(Rt34d_);
//	cam.R33d_.copyTo(R33d_);
//
//	principalPt_ = cv::Vec2d();
//	KInv33d_ = cv::Mat(3, 3, CV_64F);
//	M33d_ = cv::Mat(3, 3, CV_64F);
//	MInv33d_ = cv::Mat(3, 3, CV_64F);
//	C_ = cv::Vec3d();
//	lookDir_ = cv::Vec3d();
//	t_ = cv::Vec3d();
//	Rt34d_ = cv::Mat(3, 4, CV_64F);
//	Rt44d_ = cv::Mat(4, 4, CV_64F);
//	RtInv44d_ = cv::Mat(4, 4, CV_64F);
//
//	hasDisplayList_ = false;
//	initVariables();
//	initDisplayList();
//}


Camera::Camera(cv::Mat& K33d, cv::Mat& Rt34d, int width, int height, float CCDWidth)
{
	f_mm_ = CCDHeight_ = -1;

	width_ = width;
	height_ = height;
	CCDWidth_ = CCDWidth;

	std::cout << Rt34d << std::endl;

	P34d_ = cv::Mat(3, 4, CV_64F);
	cv::gemm(K33d, Rt34d, 1.0, cv::Mat(), 0.0, P34d_);

	K33d.copyTo(K33d_);

	cv::Mat R33d(3, 3, CV_64F);
	R33d.at<double>(0, 0) = Rt34d.at<double>(0, 0);
	R33d.at<double>(1, 0) = Rt34d.at<double>(1, 0);
	R33d.at<double>(2, 0) = Rt34d.at<double>(2, 0);
	R33d.at<double>(0, 1) = Rt34d.at<double>(0, 1);
	R33d.at<double>(1, 1) = Rt34d.at<double>(1, 1);
	R33d.at<double>(2, 1) = Rt34d.at<double>(2, 1);
	R33d.at<double>(0, 2) = Rt34d.at<double>(0, 2);
	R33d.at<double>(1, 2) = Rt34d.at<double>(1, 2);
	R33d.at<double>(2, 2) = Rt34d.at<double>(2, 2);

	R33d.copyTo(R33d_);

	principalPt_ = cv::Vec2d();
	KInv33d_ = cv::Mat(3, 3, CV_64F);
	M33d_ = cv::Mat(3, 3, CV_64F);
	MInv33d_ = cv::Mat(3, 3, CV_64F);
	C_ = cv::Vec3d();
	lookDir_ = cv::Vec3d();
	t_ = cv::Vec3d();
	Rt34d_ = cv::Mat(3, 4, CV_64F);
	Rt44d_ = cv::Mat(4, 4, CV_64F);
	RtInv44d_ = cv::Mat(4, 4, CV_64F);

	hasDisplayList_ = false;
	initVariables();
	initDisplayList();
}

Camera::Camera(cv::Mat& K33d, cv::Mat& R33d, cv::Mat& T13d, int width, int height, float CCDWidth)
{
	f_mm_ = CCDHeight_ = -1;

	width_ = width;
	height_ = height;
	CCDWidth_ = CCDWidth;

	cv::Mat Rt34d(3, 4, CV_64F);
	Rt34d.at<double>(0, 0) = R33d.at<double>(0, 0);
	Rt34d.at<double>(1, 0) = R33d.at<double>(1, 0);
	Rt34d.at<double>(2, 0) = R33d.at<double>(2, 0);
	Rt34d.at<double>(0, 1) = R33d.at<double>(0, 1);
	Rt34d.at<double>(1, 1) = R33d.at<double>(1, 1);
	Rt34d.at<double>(2, 1) = R33d.at<double>(2, 1);
	Rt34d.at<double>(0, 2) = R33d.at<double>(0, 2);
	Rt34d.at<double>(1, 2) = R33d.at<double>(1, 2);
	Rt34d.at<double>(2, 2) = R33d.at<double>(2, 2);
	Rt34d.at<double>(0, 3) = T13d.at<double>(0, 0);
	Rt34d.at<double>(1, 3) = T13d.at<double>(0, 1);
	Rt34d.at<double>(2, 3) = T13d.at<double>(0, 2);

	std::cout << Rt34d << std::endl;

	P34d_ = cv::Mat(3, 4, CV_64F);
	cv::gemm(K33d, Rt34d, 1.0, cv::Mat(), 0.0, P34d_);

	K33d.copyTo(K33d_);
	R33d.copyTo(R33d_);

	principalPt_ = cv::Vec2d();
	KInv33d_ = cv::Mat(3, 3, CV_64F);
	M33d_ = cv::Mat(3, 3, CV_64F);
	MInv33d_ = cv::Mat(3, 3, CV_64F);
	C_ = cv::Vec3d();
	lookDir_ = cv::Vec3d();
	t_ = cv::Vec3d();
	Rt34d_ = cv::Mat(3, 4, CV_64F);
	Rt44d_ = cv::Mat(4, 4, CV_64F);
	RtInv44d_ = cv::Mat(4, 4, CV_64F);

	hasDisplayList_ = false;
	initVariables();
	initDisplayList();
}


void Camera::initVariables()
{
	f_ = K33d_.at<double>(1, 1);

	principalPt_[0] = K33d_.at<double>(0, 2);
	principalPt_[1] = K33d_.at<double>(1, 2);

	// compute M = KR
	cv::gemm(K33d_, R33d_, 1.0, cv::Mat(), 0.0, M33d_);

	// compute M inverse
	cv::invert(M33d_, MInv33d_, cv::DECOMP_SVD);

	// compute K inverse
	cv::invert(K33d_, KInv33d_, cv::DECOMP_SVD);

	// compute camera center
	C_[0] = -(MInv33d_.at<double>(0, 0) * P34d_.at<double>(0, 3) + MInv33d_.at<double>(0, 1) * P34d_.at<double>(1, 3) + MInv33d_.at<double>(0, 2) * P34d_.at<double>(2, 3));
	C_[1] = -(MInv33d_.at<double>(1, 0) * P34d_.at<double>(0, 3) + MInv33d_.at<double>(1, 1) * P34d_.at<double>(1, 3) + MInv33d_.at<double>(1, 2) * P34d_.at<double>(2, 3));
	C_[2] = -(MInv33d_.at<double>(2, 0) * P34d_.at<double>(0, 3) + MInv33d_.at<double>(2, 1) * P34d_.at<double>(1, 3) + MInv33d_.at<double>(2, 2) * P34d_.at<double>(2, 3));

	// compute look direction
	lookDir_[0] = -R33d_.at<double>(2, 0);
	lookDir_[1] = -R33d_.at<double>(2, 1);
	lookDir_[2] = -R33d_.at<double>(2, 2);

	// compute translation vector (rigid body)
	t_[0] = -(R33d_.at<double>(0, 0) * C_[0] + R33d_.at<double>(0, 1) * C_[1] + R33d_.at<double>(0, 2) * C_[2]);
	t_[1] = -(R33d_.at<double>(1, 0) * C_[0] + R33d_.at<double>(1, 1) * C_[1] + R33d_.at<double>(1, 2) * C_[2]);
	t_[2] = -(R33d_.at<double>(2, 0) * C_[0] + R33d_.at<double>(2, 1) * C_[1] + R33d_.at<double>(2, 2) * C_[2]);

	Rt44d_.at<double>(0, 0) = R33d_.at<double>(0, 0);
	Rt44d_.at<double>(1, 0) = R33d_.at<double>(1, 0);
	Rt44d_.at<double>(2, 0) = R33d_.at<double>(2, 0);
	Rt44d_.at<double>(3, 0) = 0;
	Rt44d_.at<double>(0, 1) = R33d_.at<double>(0, 1);
	Rt44d_.at<double>(1, 1) = R33d_.at<double>(1, 1);
	Rt44d_.at<double>(2, 1) = R33d_.at<double>(2, 1);
	Rt44d_.at<double>(3, 1) = 0;
	Rt44d_.at<double>(0, 2) = R33d_.at<double>(0, 2);
	Rt44d_.at<double>(1, 2) = R33d_.at<double>(1, 2);
	Rt44d_.at<double>(2, 2) = R33d_.at<double>(2, 2);
	Rt44d_.at<double>(3, 2) = 0;
	Rt44d_.at<double>(0, 3) = t_[0];
	Rt44d_.at<double>(1, 3) = t_[1];
	Rt44d_.at<double>(2, 3) = t_[2];
	Rt44d_.at<double>(3, 3) = 1;

	Rt34d_.at<double>(0, 0) = Rt44d_.at<double>(0, 0);
	Rt34d_.at<double>(1, 0) = Rt44d_.at<double>(1, 0);
	Rt34d_.at<double>(2, 0) = Rt44d_.at<double>(2, 0);
	Rt34d_.at<double>(0, 1) = Rt44d_.at<double>(0, 1);
	Rt34d_.at<double>(1, 1) = Rt44d_.at<double>(1, 1);
	Rt34d_.at<double>(2, 1) = Rt44d_.at<double>(2, 1);
	Rt34d_.at<double>(0, 2) = Rt44d_.at<double>(0, 2);
	Rt34d_.at<double>(1, 2) = Rt44d_.at<double>(1, 2);
	Rt34d_.at<double>(2, 2) = Rt44d_.at<double>(2, 2);
	Rt34d_.at<double>(0, 3) = Rt44d_.at<double>(0, 3);
	Rt34d_.at<double>(1, 3) = Rt44d_.at<double>(1, 3);
	Rt34d_.at<double>(2, 3) = Rt44d_.at<double>(2, 3);

	// rigid body motion giving the camera's pose in the world coordinate frame
	cv::invert(Rt44d_, RtInv44d_, cv::DECOMP_SVD);

	if (width_ != -1 && height_ != -1 && CCDWidth_ != -1)
	{
		f_mm_ = f_ * CCDWidth_ / width_;
		CCDHeight_ = height_ * CCDWidth_ / width_;
	}
}

void Camera::initDisplayList()
{
	if (hasDisplayList_)
		return;

	float s = 20;
	float CCDWidth_half_mm = s * CCDWidth_ * 0.5;
	float CCDHeight_half_mm = s * CCDHeight_ * 0.5;

	float f_mm = CCDWidth_half_mm; // s* f_mm_;

	displayList_ = glGenLists(1);
	glNewList(displayList_, GL_COMPILE);

		// camera front
		glBegin(GL_LINES);
		glVertex3f(CCDWidth_half_mm, CCDHeight_half_mm, f_mm);
		glVertex3f(CCDWidth_half_mm, -CCDHeight_half_mm, f_mm);
		glEnd();
		glBegin(GL_LINES);
		glVertex3f(CCDWidth_half_mm, CCDHeight_half_mm, f_mm);
		glVertex3f(-CCDWidth_half_mm, CCDHeight_half_mm, f_mm);
		glEnd();
		glBegin(GL_LINES);
		glVertex3f(-CCDWidth_half_mm, -CCDHeight_half_mm, f_mm);
		glVertex3f(CCDWidth_half_mm, -CCDHeight_half_mm, f_mm);
		glEnd();
		glBegin(GL_LINES);
		glVertex3f(-CCDWidth_half_mm, -CCDHeight_half_mm, f_mm);
		glVertex3f(-CCDWidth_half_mm, CCDHeight_half_mm, f_mm);
		glEnd();

		// camera body
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(CCDWidth_half_mm, CCDHeight_half_mm, f_mm);
		glEnd();
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(CCDWidth_half_mm, -CCDHeight_half_mm, f_mm);
		glEnd();
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(-CCDWidth_half_mm, CCDHeight_half_mm, f_mm);
		glEnd();
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(-CCDWidth_half_mm, -CCDHeight_half_mm, f_mm);
		glEnd();

		//glColor3f(1, 0, 0);

		//// x-direction
		//glBegin(GL_LINES);
		//glVertex3f(0, 0, 0);
		//glVertex3f(f_mm * 0.5, 0, 0);
		//glEnd();

		glColor3f(1, 0, 0);

		// y-direction
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(0, -f_mm * 0.5, 0);
		glEnd();

		//glColor3f(0, 0, 1);

		//// z-direction
		//glBegin(GL_LINES);
		//glVertex3f(0, 0, 0);
		//glVertex3f(0, 0, f_mm * 0.5);
		//glEnd();

	glEndList();

	hasDisplayList_ = true;
}

void Camera::scale(float imScalingFactor)
{
	if (width_ != -1 && height_ != -1)
	{
		width_ *= imScalingFactor;
		height_ *= imScalingFactor;
	}

	K33d_.at<double>(0, 0) *= imScalingFactor;
	K33d_.at<double>(1, 1) *= imScalingFactor;
	K33d_.at<double>(0, 2) *= imScalingFactor;
	K33d_.at<double>(1, 2) *= imScalingFactor;

	f_ = K33d_.at<double>(1, 1);

	principalPt_[0] = K33d_.at<double>(0, 2);
	principalPt_[1] = K33d_.at<double>(1, 2);

	// compute M = KR
	cv::gemm(K33d_, R33d_, 1.0, cv::Mat(), 0.0, M33d_);

	// compute M inverse
	cv::invert(M33d_, MInv33d_, cv::DECOMP_SVD);

	// compute K inverse
	cv::invert(K33d_, KInv33d_, cv::DECOMP_SVD);

	P34d_ = cv::Mat(3, 4, CV_64F);
	cv::gemm(K33d_, Rt34d_, 1.0, cv::Mat(), 0.0, P34d_);

	glDeleteLists(displayList_, 1);
	initDisplayList();
}

void Camera::rotate(cv::Mat rotToApply33d)
{
	cv::Mat oldR33d;
	R33d_.copyTo(oldR33d);

	cv::Mat R33d(3, 3, CV_64F);
	cv::gemm(rotToApply33d, oldR33d, 1.0, cv::Mat(), 0.0, R33d);

	cv::Vec3d t(-(R33d.at<double>(0, 0) * C_[0] + R33d.at<double>(0, 1) * C_[1] + R33d.at<double>(0, 2) * C_[2]),
		-(R33d.at<double>(1, 0) * C_[0] + R33d.at<double>(1, 1) * C_[1] + R33d.at<double>(1, 2) * C_[2]),
		-(R33d.at<double>(2, 0) * C_[0] + R33d.at<double>(2, 1) * C_[1] + R33d.at<double>(2, 2) * C_[2]));

	cv::Mat Rt34d(3, 4, CV_64F);
	Rt34d.at<double>(0, 0) = R33d.at<double>(0, 0);
	Rt34d.at<double>(1, 0) = R33d.at<double>(1, 0);
	Rt34d.at<double>(2, 0) = R33d.at<double>(2, 0);
	Rt34d.at<double>(0, 1) = R33d.at<double>(0, 1);
	Rt34d.at<double>(1, 1) = R33d.at<double>(1, 1);
	Rt34d.at<double>(2, 1) = R33d.at<double>(2, 1);
	Rt34d.at<double>(0, 2) = R33d.at<double>(0, 2);
	Rt34d.at<double>(1, 2) = R33d.at<double>(1, 2);
	Rt34d.at<double>(2, 2) = R33d.at<double>(2, 2);
	Rt34d.at<double>(0, 3) = t[0];
	Rt34d.at<double>(1, 3) = t[1];
	Rt34d.at<double>(2, 3) = t[2];

	P34d_ = cv::Mat(3, 4, CV_64F);
	cv::gemm(K33d_, Rt34d, 1.0, cv::Mat(), 0.0, P34d_);

	initVariables();
	glDeleteLists(displayList_, 1);
	initDisplayList();
}

void Camera::displayLocal()
{
	displayLocal(0, 0, 0);
}

void Camera::displayLocal(float r, float g, float b)
{
	if (!hasDisplayList_)
		initDisplayList();

	glColor3f(r, g, b);
	glCallList(displayList_);
	glFlush();
}

void Camera::displayWorld()
{
	displayWorld(0, 0, 0);
}

void Camera::displayWorld(float r, float g, float b)
{
	if (!hasDisplayList_)
		initDisplayList();

	glColor4f(r, g, b, 1.0);
	glPushMatrix();
	glMultMatrixd(Ancillary::flattenMat44d(RtInv44d_));
	glCallList(displayList_);
	glPopMatrix();
	glFlush();
}

cv::Vec2f Camera::projectLocal(cv::Vec3f ptInLocalCoordFrame)
{
	cv::Vec2f ret;

	ret[0] = K33d_.at<double>(0, 0) * ptInLocalCoordFrame[0] + K33d_.at<double>(0, 1) * ptInLocalCoordFrame[1] + K33d_.at<double>(0, 2) * ptInLocalCoordFrame[2];
	ret[1] = K33d_.at<double>(1, 0) * ptInLocalCoordFrame[0] + K33d_.at<double>(1, 1) * ptInLocalCoordFrame[1] + K33d_.at<double>(1, 2) * ptInLocalCoordFrame[2];
	double s = K33d_.at<double>(2, 0) * ptInLocalCoordFrame[0] + K33d_.at<double>(2, 1) * ptInLocalCoordFrame[1] + K33d_.at<double>(2, 2) * ptInLocalCoordFrame[2];

	ret[0] /= s;
	ret[1] /= s;

	return ret;
}

cv::Vec2f Camera::projectWorld(cv::Vec3f ptInWorldCoordFrame)
{
	cv::Vec2f ret;

	ret[0] = P34d_.at<double>(0, 0) * ptInWorldCoordFrame[0] + P34d_.at<double>(0, 1) * ptInWorldCoordFrame[1] + P34d_.at<double>(0, 2) * ptInWorldCoordFrame[2] + P34d_.at<double>(0, 3);
	ret[1] = P34d_.at<double>(1, 0) * ptInWorldCoordFrame[0] + P34d_.at<double>(1, 1) * ptInWorldCoordFrame[1] + P34d_.at<double>(1, 2) * ptInWorldCoordFrame[2] + P34d_.at<double>(1, 3);
	double s = P34d_.at<double>(2, 0) * ptInWorldCoordFrame[0] + P34d_.at<double>(2, 1) * ptInWorldCoordFrame[1] + P34d_.at<double>(2, 2) * ptInWorldCoordFrame[2] + P34d_.at<double>(2, 3);

	ret[0] /= s;
	ret[1] /= s;

	return ret;
}

cv::Vec3f Camera::backprojectLocal(cv::Vec2f pixel)
{
	return Ancillary::Mat33dTimesVec3d(KInv33d_, cv::Vec3d(pixel[0], pixel[1], 1));
}

Camera::~Camera()
{
	glDeleteLists(displayList_, 1);
}
