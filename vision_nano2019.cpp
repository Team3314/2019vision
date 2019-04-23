#include "vision.hpp"
#include "RPV2CameraInfo.hpp"
#include "TargetTracker2019.hpp"
#include "Goal2019.hpp"

bool ProcessCmdLineArgs(int argc, char *argv[]);

bool robot = true;
bool debug = false;
bool verbose = false;
bool showOutputWindow = false;
std::string ntIP = "10.33.14.2";
std::string streamIP = "10.33.14.5";
double lastGoodDistance = -1;
cv::Scalar minHueSatVal(65, 0, 100);
cv::Scalar maxHueSatVal(100, 245, 234);
RPV2CameraInfo camInfo(1280, 720, 30, 1280, 720, 62.2, 48.8, 0, 0, 1, 1, 1, 1, 36000000, 36000000);

int main(int argc, char *argv[])
{
	//TODO: Overhaul script parameters to account for new camera setup and differentiating production/practice
	//Default to production mode

	ProcessCmdLineArgs(argc, argv);

	//Always console output
	std::cout << "ntIP: " << ntIP << "  streamIP: " << streamIP << std::endl;
	std::cout << "CameraAngle: " << camInfo.HorizMountAngle << std::endl;
	std::cout << "MinHSV: " << minHueSatVal << " MaxHSV: " << maxHueSatVal << std::endl;

	long long increment = 0;

	TargetTracker2019 targetTracker(&camInfo, verbose, minHueSatVal, maxHueSatVal);

	std::shared_ptr<NetworkTable> myNetTable;
	if (!debug)
	{
		NetworkTable::SetClientMode();
		NetworkTable::SetDSClientEnabled(false);
		NetworkTable::SetIPAddress(llvm::StringRef(ntIP));
		NetworkTable::Initialize();
		myNetTable = NetworkTable::GetTable("SmartDashboard/jetson");
	}

	for (;;)
	{
		double distance = -1;
		double distanceHigh = -1;
		double botDistance = -1;
		double offset = -1;
		double angleToTarget = -1;

		targetTracker.capture();

		if (targetTracker.frame >= camInfo.WarmupDelay)
		{
			targetTracker.analyze();

			if (verbose)
			{
				std::cout << "\nIncrement: " << increment << std::endl;
				std::cout << "centeredTargetX: " << targetTracker.centeredTargetX << std::endl;
				std::cout << "fixed target y: " << targetTracker.targetY << std::endl;
				std::cout << "bottom target angle: " << targetTracker.bottomTargetAngle << std::endl;
				std::cout << "Target angle: " << targetTracker.targetAngle << std::endl;
			}

			// TODO: This should move to TargetTracker2019::analyze() as it is very season specific.
			if (targetTracker.targetsFound == 2)
			{
				//TODO: Adapt these into existing or new functions

				bool production = false;
				double camAngle;	  // = 18*(CV_PI/180.0); // radians
				double camView;		  // = 67*(CV_PI/180.0); // radians
				double camView_x;	 //radians
				double camHeight;	 // = 13.0; // inches
				double camOffset;	 // = 1.25; // inches
				double camHorizAngle; // = 5;

				if (production)
				{
					camAngle = 27.25 * (CV_PI / 180.0); // radians
					camView = 67 * (CV_PI / 180.0);		// radians
					camView_x = 43 * (CV_PI / 180.0);   //radians
					camHeight = 12;						// inches
					camOffset = 0;						// inches - 0 for production, for now leave in practice
					camHorizAngle = 0;					// degrees - now in main code
				}
				else
				{
					camAngle = 23.5 * (CV_PI / 180.0); // radians
					camView = 67 * (CV_PI / 180.0);	// radians
					camView_x = 43 * (CV_PI / 180.0);  //radians
					camHeight = 12;					   // inches
					camOffset = 0;					   // inches - 0 for production, for now leave in practice
					camHorizAngle = 0;				   // degrees - now in main code
				}

				//Y-pos
				double pixel = targetTracker.targetY - (camInfo.ImageHeight / 2); // pixels
				double fpix = (camInfo.ImageHeight / 2) / tan(camView / 2);		  // pixels
				double fdot = fpix * fpix;										  // pixels^2
				double normpix = sqrt(fpix * fpix + pixel * pixel);				  // pixels
				double anglepix = acos(fdot / (fpix * normpix));				  // radians without sign
				if (pixel < 0)
					anglepix *= -1; // radians with sign
				double netAngle = anglepix + camAngle;
				if (netAngle <= 0)
				{
					distance = 1337.254; // inches
					distanceHigh = 1337.254;
				}
				else
				{
					distance = (31.40 - camHeight) / tan(netAngle);  // inches
					distanceHigh = (40 - camHeight) / tan(netAngle); // inches
				}

				//X-pos
				double pixel_x = targetTracker.targetX - (camInfo.ImageWidth / 2);
				double fpix_x = (camInfo.ImageWidth / 2) / tan(camView_x / 2);
				double fdot_x = fpix_x * fpix_x;
				double normpix_x = sqrt(fpix_x * fpix_x + pixel_x * pixel_x);
				double anglepix_x = acos(fdot_x / (fpix_x * normpix_x));
				if (pixel_x < 0)
					anglepix_x *= -1;
				//double correction = atan(camOffset/distance);
				angleToTarget = (anglepix_x) * (180.0 / CV_PI) - camInfo.HorizMountAngle;

				lastGoodDistance = distance;
			}

			if (!debug)
			{
				myNetTable->PutNumber("Left targetX", targetTracker.centeredTargetX);
				myNetTable->PutNumber("Left targetY", targetTracker.centeredTargetY);
				myNetTable->PutNumber("Left targetsFound", targetTracker.targetsFound);
				myNetTable->PutBoolean("Left hasLeft", targetTracker.hasLeft);
				myNetTable->PutBoolean("Left hasRight", targetTracker.hasRight);
				myNetTable->PutNumber("Left Angle to Target", targetTracker.targetAngle);

				myNetTable->PutNumber("Distance", distance);
				myNetTable->PutNumber("DistanceHigh", distanceHigh);
				myNetTable->PutNumber("Angle To Target", angleToTarget);
				myNetTable->PutNumber("increment", increment);
				//myNetTable->Flush();
			}
			if (verbose)
			{
				std::cout << "distance: " << distance << std::endl;
				std::cout << "distance high: " << distanceHigh << std::endl;
				std::cout << "angle to target: " << angleToTarget << std::endl;
			}

			if (showOutputWindow)
			{
				//cv::imshow("Output", combine);
				cv::imshow("Output", targetTracker.output);
				cv::imshow("Output2", targetTracker.output);
			}

			if (!debug)
			{
				bool enabled = myNetTable->GetBoolean("Enabled", false);

				int matchNumber = myNetTable->GetNumber("Match Number", 0);
				int timeRemaining = myNetTable->GetNumber("Time Remaining", 0);

				if (((increment % 5) == 0) && enabled && matchNumber != 0)
				{
					std::vector<int> compression_params;
					compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
					compression_params.push_back(95);
					char file_name[100];
					char src_file_name[100];

					sprintf(file_name, "/3314/images/match%d_%d_%lld.jpg", matchNumber, timeRemaining, increment);
					cv::imwrite(file_name, targetTracker.output, compression_params);
					sprintf(src_file_name, "/3314/images/SRC_match%d_%d_%lld.jpg", matchNumber, timeRemaining, increment);
					cv::imwrite(src_file_name, targetTracker.source, compression_params);
				}
			}
		}
		increment++;
		cv::waitKey(1);
	}
}

bool ProcessCmdLineArgs(int argc, char *argv[])
{
	bool res = false;

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
			std::cout << "leftcameraangle <double> - left camera angle override" << std::endl;
			std::cout << "minhsv <int> <int> <int> - min hue sat val override" << std::endl;
			std::cout << "maxhsv <int> <int> <int> - max hue sat val override\n"
					  << std::endl;
			res = true;
		}
		if (args[i] == "dev")
		{
			robot = false;
			debug = true;
			verbose = true;
			showOutputWindow = true;
			ntIP = "192.168.1.198";
			streamIP = "192.168.1.198";
			camInfo.VertMountAngle = 17.25;
			minHueSatVal = cv::Scalar(65, 0, 100);
			maxHueSatVal = cv::Scalar(100, 245, 234);
		}
		if (args[i] == "debug")
		{
			robot = true;
			debug = true;
			verbose = true;
			showOutputWindow = true;
			ntIP = "10.33.14.2";
			streamIP = "10.33.14.15";
			camInfo.VertMountAngle = 17.25;
			minHueSatVal = cv::Scalar(85, 40, 100);
			maxHueSatVal = cv::Scalar(140, 245, 234);
		}
		if (args[i] == "robot")
		{
			robot = true;
			debug = false;
			verbose = false;
			showOutputWindow = false;
			ntIP = "10.33.14.2";
			streamIP = "10.33.14.5";
			camInfo.VertMountAngle = 17.25;
			minHueSatVal = cv::Scalar(65, 0, 100);
			maxHueSatVal = cv::Scalar(100, 245, 234);
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
		if (args[i] == "leftcameraangle")
		{
			camInfo.VertMountAngle = stod(args[i + 1]);
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
	return res;
}

//Utilities...

//Detect first USB Camera ID
int findFirstUSBCamera()
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
