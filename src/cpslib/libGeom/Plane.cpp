#include "Plane.h"

Plane::Plane(const Plane & plane)
{
	normal_ = cv::Vec3d(plane.normal_);
	distance_ = plane.distance_;

	plane.rigid_.copyTo(rigid_);
}

Plane::Plane(cv::Mat & rigid)
{
	normal_ = cv::Vec3d(0, 0, -1);
	distance_ = 0;

	rigid_ = cv::Mat::eye(cv::Size(4, 4), CV_64F);
	cv::Vec3d pt = -normal_ * distance_;

	pt = Ancillary::Mat44dTimesVec3dHomog(rigid, pt);

	normal_ = Ancillary::Mat33dTimesVec3d(rigid, normal_);
	distance_ = -normal_.dot(pt);

	rigid.copyTo(rigid_);
}

Plane::Plane(cv::Vec3d normal, float distance)
{
	normal_ = cv::Vec3d(normal);
	distance_ = distance;

	cv::Mat rot = Ancillary::getMinArclengthRotationMat33d(cv::Vec3d(0, 0, -1), normal_);

	rigid_ = cv::Mat::eye(cv::Size(4, 4), CV_64F);
	for (int y = 0; y < 3; y++)
		for (int x = 0; x < 3; x++)
			rigid_.at<double>(y, x) = rot.at<double>(y, x);

	// todo: got R, but what about t?
}

void Plane::rigidTransform(cv::Mat & rigid44d)
{
	cv::Vec3d pt = -normal_ * distance_;
	pt = Ancillary::Mat44dTimesVec3dHomog(rigid44d, pt);

	normal_ = Ancillary::Mat33dTimesVec3d(rigid44d, normal_);
	distance_ = -normal_.dot(pt);

	cv::gemm(rigid44d, rigid_, 1.0, cv::Mat(), 0.0, rigid_);
}

cv::Vec3d Plane::intersect(cv::Vec3d ray)
{
	cv::Vec3d pt = -normal_ * distance_; // a point in the plane
	double d = (normal_.dot(pt)) / (ray.dot(normal_)); // distance along ray where ray intersects plane

	return d * ray;
}

double Plane::projDistanceToPlane(cv::Vec3d pt)
{
	return std::abs(normal_.dot(pt) + distance_); // assumes normal has length 1
}

double Plane::projSquaredDistanceToPlane(cv::Vec3d pt)
{
	double dist = projDistanceToPlane(pt);
	return dist * dist;
}

void Plane::display(double radius, float pointSize)
{
	glPointSize(pointSize);
	glColor3f(0.75, 0.75, 0.75);

	// 3D coordinates of plane corner points

	cv::Vec3d ulCanonical = cv::Vec3d(-radius, -radius, 0);
	cv::Vec3d ul3D = Ancillary::Mat44dTimesVec3dHomog(rigid_, ulCanonical);

	cv::Vec3d urCanonical = cv::Vec3d(radius, -radius, 0);
	cv::Vec3d ur3D = Ancillary::Mat44dTimesVec3dHomog(rigid_, urCanonical);

	cv::Vec3d llCanonical = cv::Vec3d(-radius, radius, 0);
	cv::Vec3d ll3D = Ancillary::Mat44dTimesVec3dHomog(rigid_, llCanonical);

	cv::Vec3d lrCanonical = cv::Vec3d(radius, radius, 0);
	cv::Vec3d lr3D = Ancillary::Mat44dTimesVec3dHomog(rigid_, lrCanonical);

	// normal

	//glBegin(GL_LINES);
	//	glVertex3f(centerPoint[0], centerPoint[1], centerPoint[2]);
	//	glVertex3f(radius*normal_[0] + centerPoint[0], radius*normal_[1] + centerPoint[1], radius*normal_[2] + centerPoint[2]);
	//glEnd();


	// plane

	glBegin(GL_LINES);
		glVertex3f(ul3D[0], ul3D[1], ul3D[2]);
		glVertex3f(ur3D[0], ur3D[1], ur3D[2]);
	glEnd();

	glBegin(GL_LINES);
		glVertex3f(ur3D[0], ur3D[1], ur3D[2]);
		glVertex3f(lr3D[0], lr3D[1], lr3D[2]);
	glEnd();

	glBegin(GL_LINES);
		glVertex3f(lr3D[0], lr3D[1], lr3D[2]);
		glVertex3f(ll3D[0], ll3D[1], ll3D[2]);
	glEnd();

	glBegin(GL_LINES);
		glVertex3f(ll3D[0], ll3D[1], ll3D[2]);
		glVertex3f(ul3D[0], ul3D[1], ul3D[2]);
	glEnd();

	glFlush();
}
