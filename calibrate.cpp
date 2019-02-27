#include "vision.hpp"

//TODO
//Hinting at other targets by pulling from net tables []
//video conference style streaming/multiple outputs on one streamed mat []
//GUI to fix thresholding []
//auto run vision script on startup []
//Confidence levels []
// [.] In order to optimize each independently, split camera and driver station stream params (i.e. width)
// [] command line flags to switch between robot mode/dev board mode
// calibration system with known distances []

//split hfov/width*multiplier into 2 vars, one for each cam? could give more accurate camera angles which leads to better distances/target angles
//e.g. while .875 may be a better approx for the right camera, maybe the left's consistently lower .764 would be better for that side

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
	bool twoTargetsFound = false;
	int leftCameraID, rightCameraID;
	double cameraSeparation;

	TargetTracker(int Device, double BaseOffset, double CameraSeparation, double Multiplier, int LeftCameraID, int RightCameraID)
		: input(Device)
	{
		device = Device;
		baseOffset = BaseOffset;
		cameraSeparation = CameraSeparation;
		multiplier = Multiplier;
		leftCameraID = LeftCameraID;
		rightCameraID = RightCameraID;
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
			if (rRatio < MIN_ASPECT_RATIO || rRatio > MAX_ASPECT_RATIO)
				continue;
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
			//	if (offset < MIN_OFFSET || offset > MAX_OFFSET)
			//		continue;
			//std::cout << "Offset: " << offset << std::endl;
			//if (cRatio < 0.2 || cRatio > 0.6) continue;

			possible.push_back(current);
		}

		t2 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

		if (possible.size() > 0)
		{
			double mostArea = 0;
			double secondMostArea = 0;
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
					cv::line(poss, pts[j], pts[(j + 1) % 4], WHITE);
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
						cv::line(poss, pts[j], pts[(j + 1) % 4], YELLOW, 2);
					}
				}
				//cv::imshow("Possible", poss);

				double area = cv::contourArea(possible[i]);
				//std::cout << "Area: " << area << std::endl;
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

			t3 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

			best.push_back(firstBest);
			if (firstBest != secondBest)
			{
				//best.push_back(secondBest);
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
						cv::line(output, pts[j], pts[(j + 1) % 4], WHITE, 2);
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
						cv::line(output, pts[j], pts[(j + 1) % 4], YELLOW, 2);
						leftCenter = center;
					}
				}
				//std::cout << "Position: " << center.x << std::endl;
			}
			if (leftCenter.x != 0 && rightCenter.x != 0)
			{
				targetX = (rightCenter.x + leftCenter.x) / 2;
				targetY = (rightCenter.y + leftCenter.y) / 2;
				distance = (rightCenter.x - leftCenter.x) / 2;
				twoTargetsFound = true;
			}
			else
			{
				twoTargetsFound = false;
				if (rightCenter.x != 0)
				{
					targetX = rightCenter.x;
				}
				if (leftCenter.x != 0)
				{
					targetX = leftCenter.x;
				}
			}
		}

		cv::line(output, cv::Point(targetX, 0), cv::Point(targetX, OPENCV_HEIGHT), cv::Scalar(0, 0, 255), 2);

		centeredTargetX = targetX - (OPENCV_WIDTH / 2);
		std::cout << "centeredTargetX: " << centeredTargetX << std::endl;
		if (USE_T_CALIBRATION)
		{
			double cameraAngle;
			double inCameraAngle;
			double angleToTarget;
			if (device == leftCameraID)
			{
				angleToTarget = atan(TARGET_DISTANCE / LEFT_SEPARATION) * (180.0 / CV_PI);
				inCameraAngle = (centeredTargetX)*HORZ_DEGREES_PER_PIXEL * multiplier;
				cameraAngle = angleToTarget + inCameraAngle;
			}
			if (device == rightCameraID)
			{
				angleToTarget = atan(TARGET_DISTANCE / RIGHT_SEPARATION) * (180.0 / CV_PI);
				inCameraAngle = (centeredTargetX)*HORZ_DEGREES_PER_PIXEL * multiplier;
				cameraAngle = angleToTarget - inCameraAngle;
			}
			std::cout << "angle to target: " << angleToTarget << std::endl;
			std::cout << "in Camera Angle: " << inCameraAngle << std::endl;
			std::cout << "Camera Angle: " << cameraAngle << "   " << 90 - cameraAngle << std::endl;
			std::cout << "deg/px " << 1 / (fabs(centeredTargetX) / (90 - angleToTarget)) << std::endl;
		}
		else
		{
			//perpendicular distance
			//separation of cameras / 2
			//atan perp/sep = angle to target
			//(aot - ca)*(width/hfov)=pixel offset from 0

			double cameraAngle;
			double perpDist = sqrt((CALIBRATION_DISTANCE) * (CALIBRATION_DISTANCE) - (cameraSeparation / 2) * (cameraSeparation / 2));
			std::cout << "Perp dist: " << perpDist << std::endl;
			//separation/2 defined in header
			double angleToTarg = atan(perpDist / (cameraSeparation / 2)) * (180 / CV_PI);
			std::cout << "Calc'd angle to target: " << angleToTarg << std::endl;
			if (device == rightCameraID)
			{
				cameraAngle = -angleToTarg + ((centeredTargetX)*HORZ_DEGREES_PER_PIXEL * multiplier);
			}
			else
			{
				cameraAngle = angleToTarg + ((centeredTargetX)*HORZ_DEGREES_PER_PIXEL * multiplier);
			}
			std::cout << "Calculated camera angle: " << cameraAngle << std::endl;
		}
		centeredTargetY = -targetY + (OPENCV_HEIGHT / 2);
		targetAngle = centeredTargetX * HORZ_DEGREES_PER_PIXEL * multiplier + baseOffset; // Could be a more complex calc, we'll see if we need it
																						  //std::cout << "Target angle: " << targetAngle << std::endl;

		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		std::cout << t * 1000 << "ms" << std::endl;
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

	TargetTracker leftTracker(leftCameraID, leftCameraAngle, cameraSeparation, LEFT_MULTIPLIER, leftCameraID, rightCameraID);
	TargetTracker rightTracker(rightCameraID, rightCameraAngle, cameraSeparation, RIGHT_MULTIPLIER, leftCameraID, rightCameraID);

	std::shared_ptr<NetworkTable> myNetTable;
	NetworkTable::SetClientMode();
	NetworkTable::SetDSClientEnabled(false);
	NetworkTable::SetIPAddress(llvm::StringRef(streamIP));
	NetworkTable::Initialize();
	myNetTable = NetworkTable::GetTable("SmartDashboard");

	for (;;)
	{
		double distance = 0;
		double offset = 0;
		double angleToTarget = 0;

		std::cout << "\nIncrement: " << increment << std::endl;

		leftTracker.capture();
		rightTracker.capture();

		std::cout << "LEFT" << std::endl;
		leftTracker.analyze();
		std::cout << "RIGHT" << std::endl;
		rightTracker.analyze();

		distance = -1;
		offset = 0;
		if (leftTracker.twoTargetsFound && rightTracker.twoTargetsFound)
		{
			double tanLeft = tan((CV_PI / 180) * leftTracker.targetAngle);
			double tanRight = tan((CV_PI / 180) * -rightTracker.targetAngle);
			distance = cameraSeparation / (tanLeft + tanRight);
			offset = tanLeft * distance - cameraSeparation / 2.0;
			angleToTarget = (180 / CV_PI) * atan(offset / distance);
		}
		else if (leftTracker.twoTargetsFound)
		{
			angleToTarget = leftTracker.targetAngle;
		}
		else if (rightTracker.twoTargetsFound)
		{
			angleToTarget = rightTracker.targetAngle;
		}

		//t5 = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		myNetTable->PutNumber("Left targetX", leftTracker.centeredTargetX);
		myNetTable->PutNumber("Left targetY", leftTracker.centeredTargetY);
		myNetTable->PutBoolean("Left twoTargetsFound", leftTracker.twoTargetsFound);
		myNetTable->PutNumber("Right targetX", rightTracker.centeredTargetX);
		myNetTable->PutNumber("Right targetY", rightTracker.centeredTargetY);
		myNetTable->PutBoolean("Right twoTargetsFound", rightTracker.twoTargetsFound);
		std::cout << "COMBINED" << std::endl;
		myNetTable->PutNumber("Distance", distance);
		std::cout << "Combined distance: " << distance << std::endl;
		myNetTable->PutNumber("Offset", offset);
		std::cout << "Combined offset: " << offset << std::endl;
		myNetTable->PutNumber("Angle To Target", angleToTarget);
		std::cout << "Combined angle: " << angleToTarget << std::endl;
		myNetTable->PutNumber("increment", increment);
		myNetTable->Flush();
		if (increment % 3)
		{
			IplImage outImage = (IplImage)leftTracker.output;
			mywriter.writeFrame(&outImage); //write output image over network
		}
		cv::imshow("Left Output", leftTracker.output);
		cv::imshow("Right Output", rightTracker.output);

		increment++;
		cv::waitKey(1);
	}
	cv::waitKey();
}
