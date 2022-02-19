#ifndef PLANE
#define PLANE
#include "headers.h"

class Plane
{
public:
	Plane();
	Plane(const Plane & a);

	Plane(cv::Mat & rigid);
	Plane(cv::Vec3d normal, float distance);
	Plane(cv::Vec3d normal, cv::Vec3d pt);
	Plane(std::vector<cv::Point3f> points, bool ransac = false);

	void rigidTransform(cv::Mat & rigid44d);

	cv::Vec3d intersect(cv::Vec3d ray); // e.g., cam->backprojectLocal(cv::Vec2f(x, y));
	double projDistanceToPlane(cv::Vec3d pt);
	double projSquaredDistanceToPlane(cv::Vec3d pt);

	cv::Vec3d getNormal() { return normal_; }; // first three Hessian normal form plane parameters (normal vector)
	float getDistance() { return distance_; }; // fourth Hessian normal form plane parameter (distance along normal direction from origin to plane)
	void getRigid(cv::Mat& outRigid) { rigid_.copyTo(outRigid); };

	void display(double radius, float pointSize = 1.0);

	~Plane(void) { };

protected:
	cv::Vec3d normal_;
	double distance_;

	cv::Mat rigid_;
};

#endif
