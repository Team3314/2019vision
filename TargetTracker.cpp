#include <string>
#include "TargetTracker.hpp"
#include "CameraInfo.hpp"

TargetTracker::TargetTracker(CameraInfo *camInfo, bool Verbose)
	: input(camInfo->GetPipeline(), cv::CAP_GSTREAMER)
{
	this->camInfo = camInfo;
	verbose = Verbose;
}

void TargetTracker::capture()
{
	double ta, tb, tc;

	frame++;

	if (verbose) 
	{
		std::cout << "\nCapturing..." << std::endl;
		ta = (double)cv::getTickCount();
	}

	input.read(source);
	
	if (verbose) 
	{
		tb = (double)cv::getTickCount();
		std::cout << "\n**** CAPTURE time ****:" << (tb - ta) / cv::getTickFrequency() << std::endl;
		tb = (double)cv::getTickCount();
	}
	
	if (camInfo->Transpose)
	{
		cv::transpose(source, source);
	}
	if (camInfo->FlipMode != 0)
	{
		cv::flip(source, source, camInfo->FlipMode);
	}

	if (verbose)
	{
		tc = (double)cv::getTickCount();
		std::cout << "\n**** transpose/flip time ****:" << (tc - tb) / cv::getTickFrequency() << std::endl;
	}
}

void TargetTracker::analyze(){};

double TargetTracker::angleFromPixels(double ctx)
{
	// Compute focal length in pixels from FOV
	double f = (0.5 * camInfo->ImageWidth) / tan(0.5 * camInfo->HorizViewAngle);

	// Vectors subtending image center and pixel from optical center
	// in camera coordinates.
	// center(0, 0, f) is not needed 
	// it can be reduced to f in every use
	//cv::Point3f center(0, 0, f), pixel(ctx, 0, f);
	cv::Point3f  pixel(ctx, 0, f);

	// angle between vector (0, 0, f) and pixel
	//double dot = center.x*pixel.x + center.y*pixel.y + center.z*pixel.z;
	// Optimize dot out
	//double dot = f * f;

	// cv::norm(center) will always be equal to f 
	//double alpha = (acos(dot / (cv::norm(center) * cv::norm(pixel)))) * (180 / CV_PI);
	double alpha = (acos((f*f) / (f * cv::norm(pixel)))) * (180 / CV_PI);

	// The dot product will always return a cos>0
	// when the vectors are pointing in the same general
	// direction.
	// This means that no ctx will produce a negative value.
	// We need to force the value negative to indicate "to the left".
	if (ctx < 0)
		alpha = -alpha;
	return alpha;
}

double TargetTracker::angleFromRawPixels(double tx)
{
	return angleFromPixels(tx - (camInfo->ImageWidth / 2));
}

cv::Point2f TargetTracker::findLongEdge(cv::Point2f pts[])
{
	cv::Point2f edge1 = cv::Vec2f(pts[1].x, pts[1].y) - cv::Vec2f(pts[0].x, pts[0].y);
	cv::Point2f edge2 = cv::Vec2f(pts[2].x, pts[2].y) - cv::Vec2f(pts[1].x, pts[1].y);

	cv::Point2f longEdge = edge1;
	if (cv::norm(edge2) > cv::norm(edge1))
		longEdge = edge2;

	return longEdge;
}

cv::Point2f TargetTracker::findShortEdge(cv::Point2f pts[])
{
	cv::Point2f edge1 = cv::Vec2f(pts[1].x, pts[1].y) - cv::Vec2f(pts[0].x, pts[0].y);
	cv::Point2f edge2 = cv::Vec2f(pts[2].x, pts[2].y) - cv::Vec2f(pts[1].x, pts[1].y);

	cv::Point2f shortEdge = edge1;
	if (cv::norm(edge2) < cv::norm(edge1))
		shortEdge = edge2;

	return shortEdge;
}

float TargetTracker::rrectAngleEdge(cv::Point2f usedEdge)
{
	cv::Point2f ref = cv::Vec2f(1, 0); /*Horizontal edge*/

	float angle = 180.0f / CV_PI * acos((ref.x * usedEdge.x + ref.y * usedEdge.y) / (cv::norm(ref) * cv::norm(usedEdge)));
	return angle;
}

float TargetTracker::rrectAngleOffset(float angle)
{
	return fabs(90.0 - angle);
}

bool TargetTracker::isBetween(double min, double max, double min2, double max2)
{
	if (min >= min2 && min <= max2)
		return true;
	if (max >= min2 && max <= max2)
		return true;
	if (min2 >= min && min2 <= max)
		return true;
	if (max2 >= min && max2 <= max)
		return true;
	return false;
}
