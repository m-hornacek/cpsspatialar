#ifndef POINTCLOUD
#define POINTCLOUD
#include "headers.h"

class PointCloud
{
public:
	PointCloud(); // default constructor
	PointCloud(const PointCloud& points);
	PointCloud(std::vector<cv::Point3f>& points);
	PointCloud(std::vector<cv::Point3f>& points, std::vector<cv::Point3f>& colors);
	
	std::vector<cv::Point3f> getPoints() { return points_; };
	std::vector<cv::Point3f> getColors() { return colors_; };

	void display(float pointSize);
	void display(float pointSize, float r, float g, float b, float a = 1.0);

	~PointCloud(void) { };
private:
	void initPointsDisplayList();

	std::vector<cv::Point3f> points_;

	bool hasColors_;
	std::vector<cv::Point3f> colors_;

	bool hasDisplayList_;
	GLuint displayList_;
};

#endif
 