#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <cstdlib>

#include <networktables/NetworkTable.h>
#include <stdlib.h>
#include <string.h>

#include "cap_gstreamer.hpp"

// Camera Parameters
const double HORZ_DEGREES_PER_PIXEL = 1;                        //left, right (70.42 / 640)
//const double LEFT_MULTIPLIER = 0.091, RIGHT_MULTIPLIER = 0.076; //.764, RIGHT_MULTIPLIER = .875; .0835
const double LEFT_MULTIPLIER = 0.0835, RIGHT_MULTIPLIER = 0.0835;

// Camera Configuration Parameters
const double CALIBRATION_DISTANCE = 27.64; //string len. in inches div. 2 (hypotenuse)
//const double LEFT_BASE_ANGLE = 0, RIGHT_BASE_ANGLE = 0; // degrees

// Camera Calibration Parameters
// using T bracket
const double LEFT_SEPARATION = 10.5;  //inches
const double RIGHT_SEPARATION = 11.5; //inches
const double TARGET_DISTANCE = 29;    //inches perpendicular
const bool USE_T_CALIBRATION = true;

//OpenCV camera calc parameters
const int OPENCV_WIDTH = 640, OPENCV_HEIGHT = 480;

const double FOV_RADIANS = (LEFT_MULTIPLIER + RIGHT_MULTIPLIER)/2 * OPENCV_WIDTH * (CV_PI/180);

// Output stream parameters
const int STREAM_WIDTH = 640 * 2, STREAM_HEIGHT = 480, FRAMERATE = 15, BITRATE = 600000, PORT = 5001;

const cv::Scalar MIN_HSV(80, 60, 140);
const cv::Scalar MAX_HSV(120, 220, 255);

const double MIN_ASPECT_RATIO = 0.4, MAX_ASPECT_RATIO = 0.6, MIN_AREA_RATIO = 0.75, MIN_AREA = 100, MIN_OFFSET = 3, MAX_OFFSET = 20;

const cv::Scalar BLACK(0, 0, 0);
const cv::Scalar RED(0, 0, 255);
const cv::Scalar PINK(255, 0, 255);
const cv::Scalar BLUE(255, 0, 0);


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

	/*sprintf(buff,
	"appsrc ! video/x-raw, format=(string)I420, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! omxh264enc bitrate=%d ! video/x-h264, stream-format=(string)byte-stream ! h264parse ! rtph264pay ! udpsink host=%s port=%d", width, height, framerate, bitrate, ip.c_str(), port);*/

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

double point3fLength(cv::Point3f point) {
	return sqrt((point.x)*(point.x) + (point.y)*(point.y) + (point.z)*(point.z));
}

double angleFromPixels(double ctx) {
	// Compute focal length in pixels from FOV
	double f = (0.5 * OPENCV_WIDTH) / tan(0.5 * 0.942478);

	// Vectors subtending image center and pixel from optical center
	// in camera coordinates.
	cv::Point3f center(0,0,f), pixel(ctx, 0, f);

	// angle between vector (0, 0, f) and pixel
	//double dot = dot_product(center, pixel);
	double dot = center.x*pixel.x + center.y*pixel.y + center.z*pixel.z;
	double alpha = (acos(dot / (point3fLength(center) * point3fLength(pixel))))*(180/CV_PI);
	if (ctx<0) alpha = -alpha;
	return alpha;
}

double angleFromRawPixels(double tx) {
	return angleFromPixels(tx - (OPENCV_WIDTH/2));
}

class TargetTracker
{
	cv::VideoCapture input;

  public:
	long frame;
	int device;
	cv::Mat source;
	cv::Mat output;

	double baseOffset = 0;
	double multiplier = 0;
	
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

	TargetTracker(int Device, double BaseOffset, double Multiplier)
		: input(Device)
	{
		device = Device;
		baseOffset = BaseOffset;
		multiplier = Multiplier;
		setVideoCaps(input);
	}

	void capture()
	{
		frame++;
		/*
		if (frame == 50)
		{
			flash_bad_settings(device);
		}
		else if (frame == 0 || frame == 100)
		{
			flash_good_settings(device);
		}
		*/
		input.read(source);
	}

	void analyze()
	{
		double t = (double)cv::getTickCount();
		double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;
		double distance;

		//cv::imshow("Source", source);
		cv::normalize(source, source, 0, 255, cv::NORM_MINMAX);

		//cv::Mat output = source.clone(), poss = source.clone();
		cv::Mat poss = source.clone();
		cv::Mat hsv;
		//cv::Mat noise;

		output = source.clone();

		//HSV threshold
		cv::cvtColor(source, hsv, cv::COLOR_BGR2HSV);
		cv::inRange(hsv, MIN_HSV, MAX_HSV, hsv);

		t1 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

		//Finding all contours
		std::vector<std::vector<cv::Point>> contours, possible, best;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(hsv, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

		//Filters out bad contours from possible goals
		for (size_t i = 0; i < contours.size(); i++)
		{
			std::vector<cv::Point> current = contours[i];
			cv::Rect rect = cv::boundingRect(current);
			cv::RotatedRect rRect = cv::minAreaRect(current);
			cv::Point2f pts[4];
			rRect.points(pts);

			/*Choose the longer edge of the rotated rect to compute the angle*/
			cv::Point2f edge1 = cv::Vec2f(pts[1].x, pts[1].y) - cv::Vec2f(pts[0].x, pts[0].y);
			cv::Point2f edge2 = cv::Vec2f(pts[2].x, pts[2].y) - cv::Vec2f(pts[1].x, pts[1].y);

			cv::Point2f usedEdge = edge1;
			if (cv::norm(edge2) > cv::norm(edge1))
				usedEdge = edge2;

			cv::Point2f ref = cv::Vec2f(1, 0); /*Horizontal edge*/

			float angle = 180.0f / CV_PI * acos((ref.x * usedEdge.x + ref.y * usedEdge.y) / (cv::norm(ref) * cv::norm(usedEdge)));
			float offset = fabs(90.0 - angle);

			double rHeight = cv::norm(pts[1] - pts[0]);
			double rWidth = cv::norm(pts[2] - pts[1]);
			double rRatio = rWidth / rHeight;
			double rArea = rWidth * rHeight;
			if (rRatio > 1)
			{
				rRatio = 1 / rRatio;
			}
			//if (rRatio < MIN_ASPECT_RATIO || rRatio > MAX_ASPECT_RATIO)
			//	continue;
			//std::cout << "Ratio: " << rRatio << std::endl;

			double cArea = cv::contourArea(current); //(double)(rect.width*rect.height);
			double cRatio = (double)(rect.width / rect.height);
			double areaRatio = rArea / cArea;

			//Checks if area is too small/aspect ratio is not close to 2/5
			if (areaRatio < MIN_AREA_RATIO)
				continue;
			//std::cout << "Area ratio: " << areaRatio << std::endl;
			if (cArea < MIN_AREA)
				continue;
			//std::cout << "Area: " << cArea << std::endl;
			if (rect.width > rect.height)
				continue;
			//std::cout << "Width: " << rect.width << " Height: " << rect.height << std::endl;
			//if (offset < MIN_OFFSET || offset > MAX_OFFSET)
			//	continue;
			//std::cout << "Offset: " << offset << std::endl;
			//if (cRatio < 0.2 || cRatio > 0.6) continue;

			possible.push_back(current);
		}

		t2 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

		if (possible.size() > 0)
		{
			double mostArea = 0, leastArea = 10000;
			double secondMostArea = 0, secondLeastArea = 10000;
			std::vector<cv::Point> firstBest = possible[0];
			std::vector<cv::Point> secondBest = possible[0];

			//Draws mininum area rects. and centers for possible goals
			for (size_t i = 0; i < possible.size(); i++)
			{
				cv::RotatedRect rrect = cv::minAreaRect(possible[i]);
				cv::Point2f pts[4];
				rrect.points(pts);
				cv::Point2f center = rrect.center;

				for (unsigned int j = 0; j < 4; ++j)
				{
					cv::line(poss, pts[j], pts[(j + 1) % 4], BLACK);
					//double ratio = angle(pts[(j+1)%4], pts[(j+2)%4], pts[j]);
					//std::cout << "Ratio: " << ratio << "; ";
				}

				//Commented out but this is distinguish L/R code
				//Centers are red
				cv::circle(poss, center, 2, RED);

				for (unsigned int j = 0; j < 4; ++j)
				{
					//Rights are magenta
					if (pts[0].x < pts[2].x)
					{
						cv::line(poss, pts[j], pts[(j + 1) % 4], PINK, 2);
					}
					//Lefts are black
					else
					{
						cv::line(poss, pts[j], pts[(j + 1) % 4], BLUE, 2);
					}
				}
				//cv::imshow("Possible", poss);

				double area = cv::contourArea(possible[i]);
				//std::cout << "Lowest area filter on: " << lowestAreaFilter << std::endl;
				if (lowestAreaFilter) //just an example case here: if hinting value is put on nettables by robot, pick lowest areas from possibles
				{
					if (area < secondLeastArea)
					{
						if (area <= leastArea)
						{
							secondLeastArea = leastArea;
							secondBest = firstBest;
							leastArea = area;
							firstBest = possible[i];
						}
						else if (area > leastArea)
						{
							secondLeastArea = area;
							secondBest = possible[i];
						}
					}
				}
				else if (!lowestAreaFilter) //default to largest areas otherwise
				{
					if (area > secondMostArea)
					{
						if (area >= mostArea)
						{
							secondMostArea = mostArea;
							secondBest = firstBest;
							mostArea = area;
							firstBest = possible[i];
						}
						else if (area < mostArea)
						{
							secondMostArea = area;
							secondBest = possible[i];
						}
					}
				}
			}

			t3 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

			best.push_back(firstBest);
			if (firstBest != secondBest)
			{
				best.push_back(secondBest);
			}
			/*for (size_t j = 0; j < possible.size(); j++)
			{
				if (possible[j] == firstBest || possible[j] == secondBest)
				{
					possible.erase(possible.begin() + j);
				}
			}*/

			for (size_t i = 0; i < possible.size(); i++)
			{
				if (possible[i] != firstBest && possible[i] != secondBest)
				{
					cv::RotatedRect rrect = cv::minAreaRect(possible[i]);
					cv::Point2f pts[4];
					rrect.points(pts);
					for (unsigned int j = 0; j < 4; ++j)
					{
						cv::line(output, pts[j], pts[(j + 1) % 4], BLACK, 2);
					}
				}
			}

			t4 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

			cv::Point2f rightCenter, leftCenter;
			for (size_t i = 0; i < best.size(); i++)
			{
				cv::RotatedRect rrect = cv::minAreaRect(best[i]);
				cv::Point2f pts[4];
				rrect.points(pts);
				cv::Point2f center = rrect.center;
				cv::circle(output, center, 2, RED);
				//Point 0 is the lowest point, height is distance between 0 and 1, width is distance between 1 and 2
				for (unsigned int j = 0; j < 4; ++j)
				{
					if (pts[0].x < pts[2].x)
					{
						//cv::drawContours(output, best, i, cv::Scalar(255,0,255), 2);
						cv::line(output, pts[j], pts[(j + 1) % 4], PINK, 2);
						rightCenter = center;
					}
					else
					{
						//cv::drawContours(output, best, i, cv::Scalar(255,0,0), 2);
						cv::line(output, pts[j], pts[(j + 1) % 4], BLUE, 2);
						leftCenter = center;
					}
				}
				//std::cout << "Position: " << center.x << std::endl;
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
		cv::line(output, cv::Point((OPENCV_WIDTH / 2), 0), cv::Point((OPENCV_WIDTH / 2), OPENCV_HEIGHT), cv::Scalar(255, 0, 0), 1);

		cv::line(output, cv::Point(targetX, 0), cv::Point(targetX, OPENCV_HEIGHT), cv::Scalar(0, 0, 255), 1);

		centeredTargetX = targetX - (OPENCV_WIDTH / 2);
		//std::cout << "centeredTargetX: " << centeredTargetX << std::endl;
		centeredTargetY = -targetY + (OPENCV_HEIGHT / 2);
		//targetAngle = centeredTargetX * HORZ_DEGREES_PER_PIXEL * multiplier + baseOffset; // Could be a more complex calc, we'll see if we need it
		targetAngle = angleFromPixels(centeredTargetX) + baseOffset;
		//std::cout << "Target angle: " << targetAngle << std::endl;

		//if (best.size() == 0) {
		//	targetsFound = 0, centeredTargetX = 0, centeredTargetY = 0, targetAngle = 0, hasLeft = false, hasRight = false;
		//}

		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		//std::cout << t * 1000 << "ms" << std::endl;
		//std::cout << t1 * 1000 << "ms " << t2 * 1000 << "ms " << t3 * 1000 << "ms " << t4 * 1000 << "ms " /*<< t5 * 1000 << "ms"*/ << std::endl;
	}
};

int main(int argc, char *argv[])
{
	// default to robot mode
	bool robot = true;
	bool verbose = false;
	bool showOutputWindow = false;
	std::string ntIP = "10.33.14.2";
	std::string streamIP = "10.33.14.5";
	int leftCameraID = 0;
	int rightCameraID = 1;
	double leftCameraAngle = 0;   //deg
	double rightCameraAngle = 0;  //deg
	double cameraSeparation = 22; //inches
	double lastGoodDistance = -1;

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
			verbose = true;
			showOutputWindow = true;
			ntIP = "192.168.1.198";
			streamIP = "192.168.1.198";
			leftCameraID = 1;
			rightCameraID = 2;
			leftCameraAngle = 0;   //deg
			rightCameraAngle = 0;  //deg
			cameraSeparation = 22; //inches
		}
		if (args[i] == "robot")
		{
			robot = true;
			verbose = false;
			showOutputWindow = false;
			ntIP = "10.33.14.2";
			streamIP = "10.33.14.5";
			leftCameraID = 0;
			rightCameraID = 1;
			leftCameraAngle = 0;   //deg
			rightCameraAngle = 0;  //deg
			cameraSeparation = 22; //inches
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
	}

	// output this always...
	//if (!robot)
	//{
	std::cout << "LeftCameraID: " << leftCameraID << "  RightCameraID: " << rightCameraID << std::endl;
	std::cout << "ntIP: " << ntIP << "  streamIP: " << streamIP << std::endl;
	std::cout << "LeftCameraAngle: " << leftCameraAngle << " RightCameraAngle: " << rightCameraAngle << " CameraSeparation: " << cameraSeparation << std::endl;
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

	long increment = 0;

	TargetTracker leftTracker(leftCameraID, leftCameraAngle, LEFT_MULTIPLIER);
	TargetTracker rightTracker(rightCameraID, rightCameraAngle, RIGHT_MULTIPLIER);

	std::shared_ptr<NetworkTable> myNetTable;
	NetworkTable::SetClientMode();
	NetworkTable::SetDSClientEnabled(false);
	NetworkTable::SetIPAddress(llvm::StringRef(ntIP));
	NetworkTable::Initialize();
	myNetTable = NetworkTable::GetTable("SmartDashboard/jetson");

	bool hintingExample;

	for (;;)
	{
		std::cout << "\033[2J\033[1;1H";
		double distance = -1;
		double offset = -1;
		double angleToTarget = -1;
		hintingExample = myNetTable->GetBoolean("Lowest area test", false);
		leftTracker.lowestAreaFilter = hintingExample;
		rightTracker.lowestAreaFilter = hintingExample;

		leftTracker.capture();
		rightTracker.capture();
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

		if (leftTracker.targetsFound == 1 && rightTracker.targetsFound == 1)
		{
			double tanLeft = tan((CV_PI / 180) * leftTracker.targetAngle);
			double tanRight = tan((CV_PI / 180) * rightTracker.targetAngle);
			std::cout << "Tans - left" << tanLeft << "   " << tanRight << "   " << tanLeft + tanRight << std::endl;
			distance = (cameraSeparation / (tanLeft - tanRight)); //subtract dist from frame
			lastGoodDistance = distance;
			offset = tanLeft * distance - cameraSeparation/2;
			angleToTarget = ((180 / CV_PI) * atan((tanLeft*distance-cameraSeparation/2) / distance) +
								(180 / CV_PI) * atan((tanRight*distance+cameraSeparation/2) / distance))/2;
		}
		/*
		else if (leftTracker.targetsFound >= 1 && rightTracker.targetsFound >= 1 && leftTracker.hasLeft && rightTracker.hasRight)
		{
			angleToTarget = leftTracker.leftTargetAngle + rightTracker.rightTargetAngle;
		}
		else if (leftTracker.targetsFound == 2)
		{
			if (lastGoodDistance>36) {
				angleToTarget = 10;
			}
		}
		else if (rightTracker.targetsFound == 2)
		{
			if (lastGoodDistance>36) {
				angleToTarget = -10;
			}
		}
		*/
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

		myNetTable->PutNumber("Right targetX", rightTracker.centeredTargetX);
		myNetTable->PutNumber("Right targetY", rightTracker.centeredTargetY);
		myNetTable->PutNumber("Right targetsFound", rightTracker.targetsFound);
		myNetTable->PutBoolean("Right hasLeft", rightTracker.hasLeft);
		myNetTable->PutBoolean("Right hasRight", rightTracker.hasRight);

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
		if (increment % 3)
		{
			IplImage outImage = (IplImage)leftTracker.output;
			mywriter.writeFrame(&outImage); //write output image over network
		}
		cv::Mat combine;
		cv::hconcat(leftTracker.output, rightTracker.output, combine);
		//cv::imshow("Left Output", leftTracker.output);
		//cv::imshow("Right Output", rightTracker.output);
		if (showOutputWindow)
		{
			cv::imshow("Output", combine);
		}

		increment++;
		cv::waitKey(1);
	}
	cv::waitKey();
}