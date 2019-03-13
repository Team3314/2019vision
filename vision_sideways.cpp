#include "vision.hpp"

//TODO
//PRIORITY: Primary/secondary targets (fix issue of left and right picking up different pairs)
//Time frame grabs []
//Finish camera switching []
//Confidence levels []

//Displaying and streaming cameras
std::string create_write_pipeline(int width, int height, int framerate,
								  int bitrate, std::string ip, int port)
{

	char buff[500];
	sprintf(buff,
			"appsrc ! "
			"video/x-raw, format=(string)BGR, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
			"videoconvert ! omxh264enc bitrate=%d ! video/x-h264, stream-format=(string)byte-stream ! h264parse ! rtph264pay ! "
			"udpsink host=%s port=%d",
			width, height, framerate, bitrate, ip.c_str(), port);

			/*"appsrc ! "
			"video/x-raw, format=(string)BGR, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
			"videoconvert ! x264enc speed-preset=1 tune=zerolatency bitrate=%d ! rtph264pay ! "
			"udpsink host=%s port=%d",
			width, height, framerate, bitrate, ip.c_str(), port);*/

			/*"appsrc ! "
			"video/x-raw, format=(string)BGR ! videoscale ! videoconvert ! queue ! omxh264enc target-bitrate=%d control-rate=variable ! "
			"video/x-h264, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1, profile=high ! rtph264pay ! gdppay ! "
			"udpsink host=%s port=%d",
			bitrate, width, height, framerate, ip.c_str(), port);*/

	std::string pipstring = buff;

	printf("write string: %s\n", pipstring.c_str());
	return pipstring;
}

void flash_good_settings(int device)
{
	char setting_script[100];
	sprintf(setting_script, "/3314/src/good_settings.sh %d", device);
	system(setting_script);
}

void flash_bad_settings(int device)
{
	char setting_script[100];
	sprintf(setting_script, "/3314/src/bad_settings.sh %d", device);
	system(setting_script);
}

void setVideoCaps(cv::VideoCapture &input)
{
	input.set(CV_CAP_PROP_FRAME_WIDTH, OPENCV_WIDTH);
	input.set(CV_CAP_PROP_FRAME_HEIGHT, OPENCV_HEIGHT);
}

//Calculate angles to target
double point3fLength(cv::Point3f point)
{
	return sqrt((point.x) * (point.x) + (point.y) * (point.y) + (point.z) * (point.z));
}

double angleFromPixels(double ctx)
{
	// Compute focal length in pixels from FOV
	double f = (0.5 * OPENCV_WIDTH) / tan(0.5 * FOV_RADIANS);

	// Vectors subtending image center and pixel from optical center
	// in camera coordinates.
	cv::Point3f center(0, 0, f), pixel(ctx, 0, f);

	// angle between vector (0, 0, f) and pixel
	//double dot = dot_product(center, pixel);
	//double dot = center.x*pixel.x + center.y*pixel.y + center.z*pixel.z;
	double dot = f * f;

	// TODO: Possibly replace dot with f*f
	// TODO: Possibly replace point3fLength() with cv::norm()
	double alpha = (acos(dot / (cv::norm(center) * cv::norm(pixel)))) * (180 / CV_PI);

	// The dot product will always return a cos>0
	// when the vectors are pointing in the same general
	// direction.
	// This means that no ctx will produce a negative value.
	// We need to force the value negative to indicate "to the left".
	if (ctx < 0)
		alpha = -alpha;
	return alpha;
}

double angleFromRawPixels(double tx)
{
	return angleFromPixels(tx - (OPENCV_WIDTH / 2));
}

//Detect camera ID on Ubuntu
int findFirstCamera()
{
	cv::Mat src;
	for (int i = 0; i < 10; i++)
	{
		cv::VideoCapture camera(i);
		try
		{
			camera.read(src);
			cv::cvtColor(src, src, cv::COLOR_BGR2HSV);
			camera.release();
			return i;
		}
		catch (...)
		{
			camera.release();
		}
	}
	return -1;
}

cv::Point2f findLongEdge(cv::Point2f pts[])
{
	cv::Point2f edge1 = cv::Vec2f(pts[1].x, pts[1].y) - cv::Vec2f(pts[0].x, pts[0].y);
	cv::Point2f edge2 = cv::Vec2f(pts[2].x, pts[2].y) - cv::Vec2f(pts[1].x, pts[1].y);

	cv::Point2f longEdge = edge1;
	if (cv::norm(edge2) > cv::norm(edge1))
		longEdge = edge2;

	return longEdge;
}

cv::Point2f findShortEdge(cv::Point2f pts[])
{
	cv::Point2f edge1 = cv::Vec2f(pts[1].x, pts[1].y) - cv::Vec2f(pts[0].x, pts[0].y);
	cv::Point2f edge2 = cv::Vec2f(pts[2].x, pts[2].y) - cv::Vec2f(pts[1].x, pts[1].y);

	cv::Point2f shortEdge = edge1;
	if (cv::norm(edge2) < cv::norm(edge1))
		shortEdge = edge2;

	return shortEdge;
}

float rrectAnglePts(cv::Point2f pts[])
{
	/*Choose the longer edge of the rotated rect to compute the angle*/
	cv::Point2f edge1 = cv::Vec2f(pts[1].x, pts[1].y) - cv::Vec2f(pts[0].x, pts[0].y);
	cv::Point2f edge2 = cv::Vec2f(pts[2].x, pts[2].y) - cv::Vec2f(pts[1].x, pts[1].y);

	cv::Point2f usedEdge = edge1;
	if (cv::norm(edge2) > cv::norm(edge1))
		usedEdge = edge2;

	cv::Point2f ref = cv::Vec2f(1, 0); /*Horizontal edge*/

	float angle = 180.0f / CV_PI * acos((ref.x * usedEdge.x + ref.y * usedEdge.y) / (cv::norm(ref) * cv::norm(usedEdge)));
	return angle;
}

float rrectAngleEdge(cv::Point2f usedEdge)
{
	cv::Point2f ref = cv::Vec2f(1, 0); /*Horizontal edge*/

	float angle = 180.0f / CV_PI * acos((ref.x * usedEdge.x + ref.y * usedEdge.y) / (cv::norm(ref) * cv::norm(usedEdge)));
	return angle;
}

float rrectAngleOffset(float angle)
{
	return fabs(90.0 - angle);
}

bool isBetween(double min, double max, double min2, double max2)
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

/*void calculate(Goal goal)
{
	cv::Point leftCenter, rightCenter;
	if (goal.partner != 0) {
		if (goal.isLeft) {
			leftCenter = goal.center;
		}
	}
	
	if (leftCenter.x != 0 && rightCenter.x != 0)
	{
		targetX = (rightCenter.x + leftCenter.x) / 2;
		targetY = (rightCenter.y + leftCenter.y) / 2;
		distance = (rightCenter.x - leftCenter.x) / 2;
		//leftTargetAngle = (leftCenter.x-(OPENCV_WIDTH/2)) * HORZ_DEGREES_PER_PIXEL * multiplier + baseOffset;
		//rightTargetAngle = (rightCenter.x-(OPENCV_WIDTH/2)) * HORZ_DEGREES_PER_PIXEL * multiplier + baseOffset;
		leftTargetAngle = angleFromRawPixels(leftCenter.x) + baseOffset;
		rightTargetAngle = angleFromRawPixels(rightCenter.x) + baseOffset;
		targetsFound = 2;
		hasLeft = true, hasRight = true;
	}
	else
	{
		if (rightCenter.x != 0)
		{
			targetX = rightCenter.x - distance;
			rightTargetAngle = angleFromRawPixels(rightCenter.x) + baseOffset;
			targetsFound = 1;
			hasRight = true;
		}
		if (leftCenter.x != 0)
		{
			targetX = leftCenter.x + distance;
			leftTargetAngle = angleFromRawPixels(leftCenter.x) + baseOffset;
			targetsFound = 1;
			hasLeft = true;
		}
	}
}*/

class Goal
{
  public:
	std::vector<cv::Point> contour;
	cv::Rect rect;
	cv::RotatedRect rRect;
	cv::Point2f pts[4];
	cv::Point2f center;

	float angle;
	float offset;

	double rHeight;
	double rWidth;
	double rArea;
	double rRatio;

	double cArea;
	double cRatio;
	double areaRatio;

	bool isLeft;
	bool isRight;
	//std::vector<cv::Point> partner;
	int partner;

	Goal(std::vector<cv::Point> Contour)
	{
		contour = Contour;
		rect = cv::boundingRect(contour);
		rRect = cv::minAreaRect(contour);
		rRect.points(pts);
		center = rRect.center;

		angle = rrectAngleEdge(findLongEdge(pts));
		offset = rrectAngleOffset(angle);

		rHeight = cv::norm(findLongEdge(pts));
		rWidth = cv::norm(findShortEdge(pts));

		/*angle = rrectAnglePts(pts);
		offset = rrectAngleOffset(angle);

		rHeight = cv::norm(pts[1] - pts[0]);
		rWidth = cv::norm(pts[2] - pts[1]);*/

		rArea = rWidth * rHeight;
		rRatio = rWidth / rHeight;
		if (rRatio > 1)
		{
			rRatio = 1 / rRatio;
		}

		cArea = cv::contourArea(contour); //(double)(rect.width*rect.height);
		cRatio = (double)(rect.width / rect.height);
		areaRatio = rArea / cArea;

		if (angle > 90)
		{
			isLeft = false;
			isRight = true;
		}
		else
		{
			isLeft = true;
			isRight = false;
		}

		partner = -1;
	}
};

class TargetTracker
{
	cv::VideoCapture input;

  public:
	long frame;
	int device;
	bool verbose;
	cv::Mat source;
	cv::Mat hsv;
	cv::Mat output;

	double baseOffset = 0;
	double multiplier = 0;
	cv::Scalar minHSV{55, 80, 35};
	cv::Scalar maxHSV{120, 255, 255};

	double targetX = 0;
	double targetY = 0;
	double centeredTargetX = 0;
	double centeredTargetY = 0;

	double targetAngle = 0;
	double leftTargetAngle = 0;
	double rightTargetAngle = 0;

	int targetsFound = 0;
	bool hasLeft = false;
	bool hasRight = false;
	bool lowestAreaFilter = false;

	TargetTracker(int Device, double BaseOffset, double Multiplier, bool Verbose, cv::Scalar MinHSV, cv::Scalar MaxHSV)
		: input(Device)
	{
		device = Device;
		baseOffset = BaseOffset;
		multiplier = Multiplier;
		verbose = Verbose;
		minHSV = MinHSV;
		maxHSV = MaxHSV;
		setVideoCaps(input);
	}

	void capture()
	{
		frame++;
		if (frame == 50)
		{
			flash_bad_settings(device);
		}
		else if (frame == 0 || frame == 100)
		{
			flash_good_settings(device);
		}

		input.read(source);
		//cv::rotate(source, source, cv::ROTATE_90_CLOCKWISE);
		cv::transpose(source, source);
		cv::flip(source, source, 1);
	}

	void analyze()
	{
		targetsFound = 0, centeredTargetX = 0, centeredTargetY = 0, targetAngle = 0, hasLeft = false, hasRight = false;

		double t = (double)cv::getTickCount();
		double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;
		double distance;

		//cv::imshow("Source", source);
		cv::normalize(source, source, 0, 255, cv::NORM_MINMAX);

		// TODO: don't need 3 images
		output = source.clone();

		//HSV threshold
		cv::cvtColor(source, hsv, cv::COLOR_BGR2HSV);
		cv::inRange(hsv, minHSV, maxHSV, hsv);

		t1 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

		//Finding all contours
		std::vector<std::vector<cv::Point>> contours;
		std::vector<Goal> possible, best;
		std::vector<cv::Vec4i> hierarchy;

		// TODO: don't need hierarchy
		cv::findContours(hsv, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
		if (verbose)
		{
			std::cout << "\nTotal contours: " << contours.size() << std::endl;
		}

		//take min and max x-pos of two contours in question
		//if any x-pos in between, passes
		//if passes, convex hull around 2 and erase old two contours
		for (size_t i = 0; i < contours.size(); i++)
		{
			cv::RotatedRect rRect1 = cv::minAreaRect(contours[i]);
			cv::Point2f pts1[4];
			rRect1.points(pts1);
			if (pts1[0].y < (OPENCV_HEIGHT / 5))
			{
				contours.erase(contours.begin() + i);
				i--;
			}
		}

		int size = contours.size();
		std::vector<int> rejects;
		for (size_t i = 0; i < size && size < 20; i++)
		{
			cv::RotatedRect rRect1 = cv::minAreaRect(contours[i]);
			cv::Point2f pts1[4];
			rRect1.points(pts1);
			double min = pts1[0].x;
			double max = pts1[2].x;
			for (size_t j = i + 1; j < size; j++)
			{
				cv::RotatedRect rRect2 = cv::minAreaRect(contours[j]);
				cv::Point2f pts2[4];
				rRect2.points(pts2);
				double min2 = pts2[0].x;
				double max2 = pts2[2].x;
				if (!isBetween(min, max, min2, max2))
					continue;

				//convex hull around 2 contours
				//push back convex hull
				//erase position of two separate contours
				if (verbose) {
				std::cout << "in between horiz: " << isBetween(min, max, min2, max2) << std::endl;
				}
				std::vector<cv::Point> points, hull;

				points.insert(points.end(), contours[i].begin(), contours[i].end());
				points.insert(points.end(), contours[j].begin(), contours[j].end());
				cv::convexHull(cv::Mat(points), hull);

				contours.push_back(hull);

				if (std::find(rejects.begin(), rejects.end(), i) == rejects.end())
					rejects.push_back(i);
				if (std::find(rejects.begin(), rejects.end(), j) == rejects.end())
					rejects.push_back(j);
			}
		}

		// must sort in order to ensure that we remove from the back.
		std::sort(rejects.begin(), rejects.end());
		for (size_t i = rejects.size(); i > 0; i--)
		{
			contours.erase(contours.begin() + rejects[i - 1]);
		}

		size = contours.size();
		rejects.clear();
		for (size_t i = 0; i < size && size < 20; i++)
		{
			cv::RotatedRect rRect1 = cv::minAreaRect(contours[i]);
			cv::Point2f pts1[4];
			rRect1.points(pts1);
			double min = pts1[0].x;
			double max = pts1[2].x;
			for (size_t j = i + 1; j < size; j++)
			{
				cv::RotatedRect rRect2 = cv::minAreaRect(contours[j]);
				cv::Point2f pts2[4];
				rRect2.points(pts2);
				double min2 = pts2[0].x;
				double max2 = pts2[2].x;
				if (!isBetween(min, max, min2, max2))
					continue;

				//convex hull around 2 contours
				//push back convex hull
				//erase position of two separate contours
				if (verbose) {
				std::cout << "in between horiz: " << isBetween(min, max, min2, max2) << std::endl;
				}
				std::vector<cv::Point> points, hull;

				points.insert(points.end(), contours[i].begin(), contours[i].end());
				points.insert(points.end(), contours[j].begin(), contours[j].end());
				cv::convexHull(cv::Mat(points), hull);

				contours.push_back(hull);

				if (std::find(rejects.begin(), rejects.end(), i) == rejects.end())
					rejects.push_back(i);
				if (std::find(rejects.begin(), rejects.end(), j) == rejects.end())
					rejects.push_back(j);
			}
		}
		for (int i = 0; i < contours.size(); i++)
			cv::drawContours(output, contours, i, WHITE, 2);

		// must sort in order to ensure that we remove from the back.
		std::sort(rejects.begin(), rejects.end());
		for (size_t i = rejects.size(); i > 0; i--)
		{
			contours.erase(contours.begin() + rejects[i - 1]);
		}

		//Filters out bad contours from possible goals
		for (size_t i = 0; i < contours.size(); i++)
		{
			//std::vector<cv::Point> current = contours[i];
			Goal goal(contours[i]);
			//if (goal.rRatio < MIN_ASPECT_RATIO || goal.rRatio > MAX_ASPECT_RATIO)
			//	continue;
			//std::cout << "Ratio: " << rRatio << std::endl;

			//Checks if area is too small/aspect ratio is not close to 2/5.5
			if (goal.areaRatio < MIN_AREA_RATIO)
			{
				if (verbose)
				{
					std::cout << "Failed min area ratio" << std::endl;
				}
				continue;
			}
			//std::cout << "Area ratio: " << areaRatio << std::endl;
			if (goal.cArea < MIN_AREA)
			{
				if (verbose)
				{
					std::cout << "Failed min area" << std::endl;
				}
				continue;
			}
			//std::cout << "Area: " << cArea << std::endl;
			if (goal.rect.width > goal.rect.height)
			{
				if (verbose)
				{
					std::cout << "Failed vertical check" << std::endl;
				}
				continue;
			}
			//std::cout << "Width: " << rect.width << " Height: " << rect.height << std::endl;
			if (goal.offset < MIN_OFFSET || goal.offset > MAX_OFFSET)
			{
				if (verbose) {
					std::cout << "Failed correct angle" << std::endl;
				}
				continue;
			}
			//std::cout << "Offset: " << offset << std::endl;

			possible.push_back(goal);
		}

		if (verbose)
		{
			std::cout << "Total possibles: " << possible.size() << std::endl;
		}
		t2 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

		double bestDistBetween = 10000;
		if (possible.size() > 0)
		{
			//match left and right pairs
			size = possible.size();
			for (size_t i = 0; i < size; i++)
			{
				double min = possible[i].pts[2].y;
				double max = possible[i].pts[0].y;
				if (possible[i].partner == -1)
				{
					possible[i].partner = i;
				}
				for (size_t j = i + 1; j < size; j++)
				{
					double min2 = possible[j].pts[2].y;
					double max2 = possible[j].pts[0].y;
					if (possible[j].partner == -1)
					{
						possible[j].partner = j;
					}
					if (possible[i].isLeft == possible[j].isLeft || possible[i].isRight == possible[j].isRight)
						continue;
					if (possible[i].isLeft && possible[j].isRight && possible[i].center.x > possible[j].center.x)
						continue;
					if (possible[i].isRight && possible[j].isLeft && possible[i].center.x < possible[j].center.x)
						continue;
					if (!isBetween(min, max, min2, max2))
						continue;
					double currentDistBetween = fabs(possible[i].center.x - possible[j].center.x);
					if (currentDistBetween > bestDistBetween)
						continue;
					if (currentDistBetween > (OPENCV_WIDTH/1.5))
						continue;
					//if (isBetween(min, max, min2, max2) && possible[i].isLeft != possible[j].isLeft)
					//{
					//convex hull around 2 contours
					//push back convex hull
					//erase position of two separate contours
					if (verbose) {
					std::cout << "in between vert: " << isBetween(min, max, min2, max2) << std::endl;
					}
					//std::vector<cv::Point> points, hull;
					//Goal mergedGoal;

					//points.insert(points.end(), possible[i].contour.begin(), possible[i].contour.end());
					//points.insert(points.end(), possible[j].contour.begin(), possible[j].contour.end());
					//cv::convexHull(cv::Mat(points), hull);

					//possible.push_back(mergedGoal(hull));
					bestDistBetween = currentDistBetween;
					possible[i].partner = j;
					possible[j].partner = i;
					cv::line(output, possible[i].center, possible[j].center, GREEN, 2);
					//}
				}
			}
		}

		if (possible.size() > 0)
		{
			double mostArea = 0, closest = 10000;
			double secondMostArea = 0, secondClosest = 10000;

			int firstBest = 0;
			int secondBest = 0;

			//Goal* bestGoal;
			//Goal* secondBestGoal;

			//Draws mininum area rects. and centers for possible goals
			/*for (size_t i = 0; i < possible.size(); i++)
			{
				//std::cout << "Lowest area filter on: " << lowestAreaFilter << std::endl;
				if (lowestAreaFilter) //just an example case here: if hinting value is put on nettables by robot, pick lowest areas from possibles
				{
					if (possible[i].cArea < secondLeastArea)
					{
						if (possible[i].cArea <= leastArea)
						{
							secondLeastArea = leastArea;
							secondBest = firstBest;
							leastArea = possible[i].cArea;
							firstBest = i;
						}
						else if (possible[i].cArea > leastArea)
						{
							secondLeastArea = possible[i].cArea;
							secondBest = i;
						}
					}
				}
				else if (!lowestAreaFilter) //default to largest areas otherwise
				{
					if (possible[i].cArea > secondMostArea)
					{
						if (possible[i].cArea >= mostArea)
						{
							secondMostArea = mostArea;
							secondBest = firstBest;
							mostArea = possible[i].cArea;
							firstBest = i;
						}
						else if (possible[i].cArea < mostArea)
						{
							secondMostArea = possible[i].cArea;
							secondBest = i;
						}
					}
				}
			}*/

			//wip code for largest target + its partner
			for (size_t i = 0; i < possible.size(); i++)
			{
				if (possible[i].cArea >= mostArea)
				{
					firstBest = i;
					//bestGoal = &possible[firstBest];
					mostArea = possible[i].cArea;
					secondBest = possible[i].partner;
					//secondBestGoal = &possible[secondBest];
				}
			}

			/*for (size_t i = 0; i < possible.size(); i++)
			{
				if (fabs(possible[i].center.x - (OPENCV_WIDTH/2)) < closest)
				{
					firstBest = i;
					//bestGoal = &possible[firstBest];
					closest = fabs(possible[i].center.x - (OPENCV_WIDTH/2));
					secondBest = possible[i].partner;
					//secondBestGoal = &possible[secondBest];
				}
			}*/

			t3 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

			best.push_back(possible[firstBest]);
			if (firstBest != secondBest)
			{
				best.push_back(possible[secondBest]);
			}
			/*best.push_back(*bestGoal);
			if (firstBest != secondBest)
			{
				best.push_back(*secondBestGoal);
				best[0].partner = 1;
				best[1].partner = 0;
			} else {
				best[0].partner = 0;
			}*/

			//t4 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

			cv::RotatedRect rrect[2];
			bool isLeft[2];
			double area[2];
			double height[2];

			cv::Point2f rightCenter, leftCenter;
			if (verbose)
			{
				std::cout << "Total bests: " << best.size() << std::endl;
			}
			for (size_t i = 0; i < best.size(); i++)
			{
				rrect[i] = best[i].rRect;
				area[i] = best[i].rArea;
				if (best[i].rHeight >= best[i].rWidth)
				{
					height[i] = best[i].rHeight;
				}
				else
				{
					height[i] = best[i].rWidth;
				}

				cv::circle(output, best[i].center, 2, RED);
				//Point 0 is the lowest point, height is distance between 0 and 1, width is distance between 1 and 2
				for (unsigned int j = 0; j < 4; ++j)
				{
					//if (pts[0].x < pts[2].x)
					if (!best[i].isLeft)
					{
						//cv::drawContours(output, best, i, cv::Scalar(255,0,255), 2);
						cv::line(output, best[i].pts[j], best[i].pts[(j + 1) % 4], PINK, 5);
						rightCenter = best[i].center;
						isLeft[i] = false;
					}
					else
					{
						//cv::drawContours(output, best, i, cv::Scalar(255,0,0), 2);
						cv::line(output, best[i].pts[j], best[i].pts[(j + 1) % 4], YELLOW, 5);
						leftCenter = best[i].center;
						isLeft[i] = true;
					}
				}
				//std::cout << "Position: " << center.x << std::endl;
			}

			//check if within 80% of firstbest's area
			//if secondbest center is +/- best center/2
			//need area/boundingbox of both
			//is left target is left of right target
			if (best.size() == 2)
			{
				/*if (area[1] < 0.8 * area[0])
				{
					best.erase(best.begin() + 1);
				}*/
				if (rrect[1].center.y > (rrect[0].center.y) + (height[0] / 2) || rrect[1].center.y < (rrect[0].center.y) - (height[0] / 2))
				{
					best.erase(best.begin() + 1);
				}
				else if (isLeft[1] == isLeft[0])
				{
					best.erase(best.begin() + 1);
				}
				else if ((rrect[1].center.x > rrect[0].center.x && isLeft[1]) || (rrect[1].center.x < rrect[0].center.x && !isLeft[1]))
				{
					best.erase(best.begin() + 1);
				}
				if (best.size() == 1)
				{
					if (isLeft[0])
					{
						rightCenter.x = 0;
					}
					else
					{
						leftCenter.x = 0;
					}
				}
			}

			hasLeft = false, hasRight = false;
			if (leftCenter.x != 0 && rightCenter.x != 0)
			{
				targetX = (rightCenter.x + leftCenter.x) / 2;
				targetY = (rightCenter.y + leftCenter.y) / 2;
				distance = (rightCenter.x - leftCenter.x) / 2;
				//leftTargetAngle = (leftCenter.x-(OPENCV_WIDTH/2)) * HORZ_DEGREES_PER_PIXEL * multiplier + baseOffset;
				//rightTargetAngle = (rightCenter.x-(OPENCV_WIDTH/2)) * HORZ_DEGREES_PER_PIXEL * multiplier + baseOffset;
				leftTargetAngle = angleFromRawPixels(leftCenter.x) + baseOffset;
				rightTargetAngle = angleFromRawPixels(rightCenter.x) + baseOffset;
				targetsFound = 2;
				hasLeft = true, hasRight = true;
			}
			else
			{
				if (rightCenter.x != 0)
				{
					targetX = rightCenter.x - distance;
					rightTargetAngle = angleFromRawPixels(rightCenter.x) + baseOffset;
					targetsFound = 1;
					hasRight = true;
				}
				if (leftCenter.x != 0)
				{
					targetX = leftCenter.x + distance;
					leftTargetAngle = angleFromRawPixels(leftCenter.x) + baseOffset;
					targetsFound = 1;
					hasLeft = true;
				}
			}
		}

		cv::line(output, cv::Point(targetX, 0), cv::Point(targetX, OPENCV_HEIGHT), cv::Scalar(0, 0, 255), 2);

		centeredTargetX = targetX - (OPENCV_WIDTH / 2);
		//std::cout << "centeredTargetX: " << centeredTargetX << std::endl;
		centeredTargetY = -targetY + (OPENCV_HEIGHT / 2);
		//targetAngle = centeredTargetX * HORZ_DEGREES_PER_PIXEL * multiplier + baseOffset; // Could be a more complex calc, we'll see if we need it
		targetAngle = angleFromPixels(centeredTargetX) + baseOffset;
		//std::cout << "base offset: " << baseOffset << std::endl;

		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		//std::cout << t * 1000 << "ms" << std::endl;
		//std::cout << t1 * 1000 << "ms " << t2 * 1000 << "ms " << t3 * 1000 << "ms " << t4 * 1000 << "ms " /*<< t5 * 1000 << "ms"*/ << std::endl;
	}
};

int main(int argc, char *argv[])
{
	// default to robot mode
	bool robot = true;
	bool debug = false;
	bool verbose = false;
	bool showOutputWindow = false;
	std::string ntIP = "10.33.14.2";
	std::string streamIP = "10.33.14.5";
	int leftCameraID = 1;
	int rightCameraID = 2;
	int frontCameraID = 0;
	int backCameraID = 3;
	double leftCameraAngle = 9.5; //deg
	double rightCameraAngle = -9.5; //deg
	double cameraSeparation = 22;  //inches
	double lastGoodDistance = -1;
	//cv::Scalar minHueSatVal(55, 80, 35);
	//cv::Scalar maxHueSatVal(120, 255, 245);
	cv::Scalar minHueSatVal(65, 0, 0);
	cv::Scalar maxHueSatVal(90, 254, 254);

	std::vector<std::string> args(argv, argv + argc);
	for (size_t i = 1; i < args.size(); ++i)
	{
		if (args[i] == "-h")
		{
			std::cout << "Options:" << std::endl;
			std::cout << "dev - dev mode defaults" << std::endl;
			std::cout << "robot - robot mode defaults" << std::endl;
			std::cout << "ntip <ip> - network table ip address" << std::endl;
			std::cout << "streamip <ip> - stream output ip address" << std::endl;
			std::cout << "showoutput - show output window locally" << std::endl;
			std::cout << "leftcameraid <id> - left camera id override" << std::endl;
			std::cout << "rightcameraid <id> - right camera id override" << std::endl;
			std::cout << "leftcameraangle <double> - left camera angle override" << std::endl;
			std::cout << "rightcameraangle <double> - right camera angle override" << std::endl;
			std::cout << "cameraseparation <double> - camera separation override" << std::endl;
			std::cout << "" << std::endl;
			return 0;
		}
		if (args[i] == "dev")
		{
			robot = false;
			debug = true;
			verbose = true;
			showOutputWindow = true;
			ntIP = "192.168.1.198";
			streamIP = "192.168.1.198";
			leftCameraID = 1;
			rightCameraID = 2;
			frontCameraID = 0;
			backCameraID = 3;
			leftCameraAngle = 9.5;   //deg
			rightCameraAngle = -9.5; //deg
			cameraSeparation = 22;   //inches
		}
		if (args[i] == "debug")
		{
			robot = true;
			debug = true;
			verbose = true;
			showOutputWindow = true;
			ntIP = "10.33.14.2";
			streamIP = "10.33.14.15";
			leftCameraID = 1;
			rightCameraID = 2;
			frontCameraID = 0;
			backCameraID = 3;
			leftCameraAngle = 9.5;
			rightCameraAngle = -9.5;
			cameraSeparation = 22;
		}
		if (args[i] == "robot")
		{
			robot = true;
			debug = false;
			verbose = false;
			showOutputWindow = false;
			ntIP = "10.33.14.2";
			streamIP = "10.33.14.5";
			leftCameraID = 1;
			rightCameraID = 2;
			frontCameraID = 0;
			backCameraID = 3;
			leftCameraAngle = 9.5;   //deg
			rightCameraAngle = -9.5; //deg
			cameraSeparation = 22;   //inches
		}
		if (args[i] == "ntip")
		{
			ntIP = args[i + 1];
		}
		if (args[i] == "streamip")
		{
			streamIP = args[i + 1];
		}
		if (args[i] == "showoutput")
		{
			showOutputWindow = true;
		}
		if (args[i] == "verbose")
		{
			verbose = true;
		}
		if (args[i] == "leftcameraid")
		{
			leftCameraID = atoi(args[i + 1].c_str());
		}
		if (args[i] == "rightcameraid")
		{
			rightCameraID = atoi(args[i + 1].c_str());
		}
		if (args[i] == "frontcameraid")
		{
			frontCameraID = atoi(args[i + 1].c_str());
		}
		if (args[i] == "backcameraid")
		{
			backCameraID = atoi(args[i + 1].c_str());
		}
		if (args[i] == "leftcameraangle")
		{
			leftCameraAngle = stod(args[i + 1]);
		}
		if (args[i] == "rightcameraangle")
		{
			rightCameraAngle = stod(args[i + 1]);
		}
		if (args[i] == "cameraseparation")
		{
			cameraSeparation = stod(args[i + 1]);
		}
		if (args[i] == "minhsv")
		{
			minHueSatVal = cv::Scalar(stoi(args[i + 1]), stoi(args[i + 2]), stoi(args[i + 3]));
		}
		if (args[i] == "maxhsv")
		{
			maxHueSatVal = cv::Scalar(stoi(args[i + 1]), stoi(args[i + 2]), stoi(args[i + 3]));
		}
	}

	int firstCamera = findFirstCamera();
	if (firstCamera == -1)
	{
		std::cout << "Cameras not found" << std::endl;
	}
	else
	{
		frontCameraID += firstCamera;
		leftCameraID += firstCamera;
		rightCameraID += firstCamera;
		backCameraID += firstCamera;
	}

	// output this always...
	//if (!robot)
	//{
	std::cout << "LeftCameraID: " << leftCameraID << "  RightCameraID: " << rightCameraID << "  FrontCameraID: " << frontCameraID << "  BackCameraID: " << backCameraID << std::endl;
	std::cout << "ntIP: " << ntIP << "  streamIP: " << streamIP << std::endl;
	std::cout << "LeftCameraAngle: " << leftCameraAngle << " RightCameraAngle: " << rightCameraAngle << " CameraSeparation: " << cameraSeparation << std::endl;
	std::cout << "MinHSV: " << minHueSatVal << " MaxHSV: " << maxHueSatVal << std::endl;
	//}

	CvVideoWriter_GStreamer mywriter;
	std::string write_pipeline = create_write_pipeline(STREAM_WIDTH, STREAM_HEIGHT, FRAMERATE,
													   BITRATE, streamIP, PORT);
	 if (verbose)
	 {
	 	printf("GStreamer write pipeline: %s\n", write_pipeline.c_str());
	 }
	mywriter.open(write_pipeline.c_str(),
				  0, FRAMERATE, cv::Size(STREAM_WIDTH, STREAM_HEIGHT), true);

	long long increment = 0;

	TargetTracker leftTracker(leftCameraID, leftCameraAngle, LEFT_MULTIPLIER, verbose, minHueSatVal, maxHueSatVal);
	TargetTracker rightTracker(rightCameraID, rightCameraAngle, RIGHT_MULTIPLIER, verbose, minHueSatVal, maxHueSatVal);
	cv::VideoCapture frontCamera(frontCameraID);
	//cv::VideoCapture backCamera(backCameraID);
	cv::Mat frontImg, backImg;
	setVideoCaps(frontCamera);
	//setVideoCaps(backCamera);

	std::shared_ptr<NetworkTable> myNetTable;
	NetworkTable::SetClientMode();
	NetworkTable::SetDSClientEnabled(false);
	NetworkTable::SetIPAddress(llvm::StringRef(ntIP));
	NetworkTable::Initialize();
	myNetTable = NetworkTable::GetTable("SmartDashboard/jetson");

	bool hintingExample;

	for (;;)
	{
		double distance = -1;
		double botDistance = -1;
		double offset = -1;
		double angleToTarget = -1;
		hintingExample = myNetTable->GetBoolean("Lowest area test", false);
		leftTracker.lowestAreaFilter = hintingExample;
		rightTracker.lowestAreaFilter = hintingExample;

		leftTracker.capture();
		rightTracker.capture();
		frontCamera.read(frontImg);
		//backCamera.read(backImg);
		if (leftTracker.frame > 100 && rightTracker.frame > 100)
		{
			leftTracker.analyze();
			rightTracker.analyze();

			if (verbose)
			{
				std::cout << "\nIncrement: " << increment << std::endl;
				std::cout << "LEFT" << std::endl;
				std::cout << "centeredTargetX: " << leftTracker.centeredTargetX << std::endl;
				std::cout << "Target angle: " << leftTracker.targetAngle << std::endl;
				std::cout << "RIGHT" << std::endl;
				std::cout << "centeredTargetX: " << rightTracker.centeredTargetX << std::endl;
				std::cout << "Target angle: " << rightTracker.targetAngle << std::endl;
			}

			if (leftTracker.targetsFound == 2 && rightTracker.targetsFound == 2)
			{
				double tanLeft = tan((CV_PI / 180) * leftTracker.targetAngle);
				double tanRight = tan((CV_PI / 180) * -rightTracker.targetAngle);
				//std::cout << "Tans - left" << tanLeft << "   " << tanRight << "   " << tanLeft + tanRight << std::endl;

				// TODO: distance can't be modifed here,
				// because it is used in the calculations below.
				// This needs to remain the camera distance for now.
				// Maybe split it out... camDistance and botDistance?
				distance = (cameraSeparation / (tanLeft + tanRight)); //subtract dist from frame (5in)
				lastGoodDistance = distance;
				botDistance = distance - 5;
				//offset = tanLeft * distance - cameraSeparation/2;
				/*angleToTarget = ((180 / CV_PI) * atan((tanLeft * distance - cameraSeparation / 2) / distance) +
							 (180 / CV_PI) * atan((tanRight * distance + cameraSeparation / 2) / distance)) /
							2;*/
				angleToTarget = leftTracker.targetAngle + rightTracker.targetAngle;
			}
			else if (leftTracker.targetsFound >= 1 && rightTracker.targetsFound >= 1 && leftTracker.hasLeft && rightTracker.hasRight)
			{
				double tanLeft = tan((CV_PI / 180) * leftTracker.leftTargetAngle);
				double tanRight = tan((CV_PI / 180) * -rightTracker.rightTargetAngle);
				angleToTarget = leftTracker.leftTargetAngle + rightTracker.rightTargetAngle;
				distance = ((cameraSeparation - 11) / (tanLeft + tanRight));
				botDistance = distance - 5;
			}
			/*else if (leftTracker.targetsFound == 2)
		{
			if (lastGoodDistance > 36)
			{
				angleToTarget = 10;
			}
		}
		else if (rightTracker.targetsFound == 2)
		{
			if (lastGoodDistance > 36)
			{
				angleToTarget = -10;
			}
		}*/
			/*
		else if (leftTracker.targetsFound >= 1 && rightTracker.targetsFound >= 1)
		{
			angleToTarget = leftTracker.targetAngle + rightTracker.targetAngle;
		}*/
			/*else if (rightTracker.targetsFound == 2)
		{
			angleToTarget = rightTracker.targetAngle;
		}*/

			//t5 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
			myNetTable->PutNumber("Left targetX", leftTracker.centeredTargetX);
			myNetTable->PutNumber("Left targetY", leftTracker.centeredTargetY);
			myNetTable->PutNumber("Left targetsFound", leftTracker.targetsFound);
			myNetTable->PutBoolean("Left hasLeft", leftTracker.hasLeft);
			myNetTable->PutBoolean("Left hasRight", leftTracker.hasRight);
			myNetTable->PutNumber("Left Angle to Target", leftTracker.targetAngle);

			myNetTable->PutNumber("Right targetX", rightTracker.centeredTargetX);
			myNetTable->PutNumber("Right targetY", rightTracker.centeredTargetY);
			myNetTable->PutNumber("Right targetsFound", rightTracker.targetsFound);
			myNetTable->PutBoolean("Right hasLeft", rightTracker.hasLeft);
			myNetTable->PutBoolean("Right hasRight", rightTracker.hasRight);
			myNetTable->PutNumber("Right Angle to Target", rightTracker.targetAngle);

			myNetTable->PutNumber("Distance", distance);
			myNetTable->PutNumber("Offset", offset);
			myNetTable->PutNumber("Angle To Target", angleToTarget);

			if (verbose)
			{
				std::cout << "COMBINED" << std::endl;
				std::cout << "Combined distance: " << distance << std::endl;
				std::cout << "Combined offset: " << offset << std::endl;
				std::cout << "Combined angle: " << angleToTarget << std::endl;
			}

			myNetTable->PutNumber("increment", increment);
			myNetTable->Flush();

			cv::Mat combine(240, 480, CV_8UC3);
			int primary = myNetTable->GetNumber("Primary img", -1);

			cv::Rect ROI1(0, 0, 320, 240);
			cv::Rect ROI2(320, 0, 160, 120);
			cv::Rect ROI3(320, 120, 160, 120);
			//cv::Rect ROI4;
			cv::Mat temp;
			switch (primary)
			{
			case 0:
				/*cv::resize(frontImg, temp, cv::Size(ROI1.width, ROI1.height));
			temp.copyTo(combine(ROI1));
			cv::resize(leftTracker.output, temp, cv::Size(ROI2.width, ROI2.height));
			temp.copyTo(combine(ROI2));
			cv::resize(rightTracker.output, temp, cv::Size(ROI4.width, ROI4.height));
			temp.copyTo(combine(ROI4));
			cv::resize(backImg, temp, cv::Size(ROI3.width, ROI3.height));
			temp.copyTo(combine(ROI3));*/
				break;
			case 1:
				/*cv::resize(frontImg, temp, cv::Size(ROI1.width, ROI1.height));
			temp.copyTo(combine(ROI3));
			cv::resize(leftTracker.output, temp, cv::Size(ROI2.width, ROI2.height));
			temp.copyTo(combine(ROI2));
			cv::resize(rightTracker.output, temp, cv::Size(ROI3.width, ROI3.height));
			temp.copyTo(combine(ROI4));
			//cv::resize(backImg, temp, cv::Size(ROI1.width, ROI1.height));
			//temp.copyTo(combine(ROI1));*/
				break;
			case 2:
				/*cv::resize(frontImg, temp, cv::Size(ROI1.width, ROI1.height));
			temp.copyTo(combine(ROI2));
			cv::resize(leftTracker.output, temp, cv::Size(ROI2.width, ROI2.height));
			temp.copyTo(combine(ROI1));
			cv::resize(rightTracker.output, temp, cv::Size(ROI3.width, ROI3.height));
			temp.copyTo(combine(ROI4));
			//cv::resize(backImg, temp, cv::Size(ROI4.width, ROI4.height));
			//temp.copyTo(combine(ROI3));*/
				break;
			case 3:
				/*cv::resize(frontImg, temp, cv::Size(ROI1.width, ROI1.height));
			temp.copyTo(combine(ROI3));
			cv::resize(leftTracker.output, temp, cv::Size(ROI2.width, ROI2.height));
			temp.copyTo(combine(ROI2));
			cv::resize(rightTracker.output, temp, cv::Size(ROI3.width, ROI3.height));
			temp.copyTo(combine(ROI1));
			//cv::resize(backImg, temp, cv::Size(ROI4.width, ROI4.height));
			//temp.copyTo(combine(ROI4));*/
				break;
			default:
				cv::resize(frontImg, temp, cv::Size(ROI1.width, ROI1.height));
				temp.copyTo(combine(ROI1));
				cv::resize(leftTracker.output, temp, cv::Size(ROI2.width, ROI2.height));
				temp.copyTo(combine(ROI2));
				cv::resize(rightTracker.output, temp, cv::Size(ROI3.width, ROI3.height));
				temp.copyTo(combine(ROI3));
				//cv::resize(backImg, temp, cv::Size(ROI4.width, ROI4.height));
				//temp.copyTo(combine(ROI4));
			}
			//cv::hconcat(leftTracker.output, rightTracker.output, combine);
			//cv::imshow("Left Output", leftTracker.output);
			//cv::imshow("Right Output", rightTracker.output);
			if (showOutputWindow)
			{
				if (debug)
				{
					cv::imshow("Left Output", leftTracker.output);
					cv::imshow("Right Output", rightTracker.output);
				}
				else
				{
					cv::imshow("Output", combine);
					//cv::imshow("front", frontImg);
					//cv::imshow("back", backImg);
				}
			}
			if (increment % 3)
			{
				IplImage outImage = (IplImage)combine;
				mywriter.writeFrame(&outImage); //write output image over network
			}

			increment++;
			cv::waitKey(1);
		}
	}
	cv::waitKey();
}
