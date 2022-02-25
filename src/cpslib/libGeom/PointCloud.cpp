 #include "PointCloud.h"

PointCloud::PointCloud()
{
	hasDisplayList_ = false;
}

PointCloud::PointCloud(const PointCloud& points)
{
	points_ = std::vector<cv::Point3f>(points.points_);

	hasColors_ = points.hasColors_;
	colors_ = std::vector<cv::Point3f>(points.colors_);

	hasDisplayList_ = false;
}

PointCloud::PointCloud(std::vector<cv::Point3f>& points)
{
	points_ = std::vector<cv::Point3f>(points);

	hasColors_ = false;
	colors_ = std::vector<cv::Point3f>();

	hasDisplayList_ = false;
}

PointCloud::PointCloud(std::vector<cv::Point3f>& points, std::vector<cv::Point3f>& colors)
{
	points_ = std::vector<cv::Point3f>(points);

	hasColors_ = true;
	colors_ = std::vector<cv::Point3f>(colors);

	hasDisplayList_ = false;
}

void PointCloud::initPointsDisplayList()
{
	if (hasDisplayList_)
		return;
	
	displayList_ = glGenLists(1);
	glNewList(displayList_, GL_COMPILE);

	for (int x = 0; x < points_.size(); x++)
	{
		cv::Vec3f pt = points_.at(x);

		if (hasColors_)
		{
			cv::Vec3f color = colors_.at(x);
			glColor3f(color[0], color[1], color[2]);
		}

		glBegin(GL_POINTS);
			glVertex3f(pt[0], pt[1], pt[2]);
		glEnd();
	}

	glEndList();

	hasDisplayList_ = true;
}

void PointCloud::display(float pointSize)
{
	if (!hasDisplayList_)
		initPointsDisplayList();

	if (!hasColors_)
		glColor3f(0, 0, 0);

	glPointSize(pointSize);
	glCallList(displayList_);

	glFlush();
}

void PointCloud::display(float pointSize, float r, float g, float b, float a)
{
	if (!hasDisplayList_)
		initPointsDisplayList();

	glColor4f(r, g, b, a);
	glPointSize(pointSize);
	glCallList(displayList_);

	glFlush();
}
