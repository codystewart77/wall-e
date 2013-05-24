/* File to take a calibrated stereo pair and map the disparity
** By default will load disparity_settings_default.xml
** and stereo_calib_default.xml
** Author: Cody Stewart
** May 23, 2013
*/


#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

using namespace cv;
using namespace std;

class Settings
{
	public:

		Settings() : goodInput(false) {};
		enum _alg { UNKNOWN, STEREO_BM, STEREO_SGBM, STEREO_HH, STEREO_VAR};
		void read(const FileNode& node)
		{
			node["Algorithm"] >> algorithm;
			//node["Max_Disparity"] >> maxDisparity;
			node["Blocksize"] >> blocksize;
			node["Scale"] >> scale;
			node["ShowDisplay"] >> showDisplay;
			node["Pointcloud_Filename"] >> pointcloudFilename;
			interprate();
		}
		
		void interprate()
		{
			goodInput = true;
			alg = 	algorithm.compare( "bm") == 0 ? STEREO_BM :
				algorithm.compare( "sgbm") == 0 ? STEREO_SGBM :
				algorithm.compare( "hh") == 0 ? STEREO_HH :
				algorithm.compare( "var") == 0 ? STEREO_VAR : UNKNOWN;
			if( alg == UNKNOWN )
			{
				cout << "Unknown stereo algorithm" << endl;
				goodInput = false;
				return;
			}
			
			if(blocksize % 2 != 1)
			{
				cout << "Blocksize must be an odd number" << endl;
				goodInput = false;
				return;
			}



		}

	public:
		string algorithm;
		_alg alg;
		bool goodInput;
		string pointcloudFilename;
		bool showDisplay;
		float scale;
		int blocksize;
};

class CalibData
{
	public:
		CalibData() : goodInput(false) {};
		//enum Pattern { NOT_EXISTING, CHESSBOARD };

		void read(const FileNode& node) //read the class from the .xml file
		{
			node["Rotation_Matrix"] >> R;
			node["Translation_Matrix"] >> T;
			node["Essential_Matrix"] >> E;
			node["Fundamental_Matrix"] >> F;
			node["RMS_Error"] >> rms;
			node["Disparity_to_Depth_Map"] >> d2dMap;
			interprate();
		}

		void interprate()
		{
			goodInput = true;
			
			if( R.rows != 3 || T.rows != 3 || E.rows != 3 || F.rows != 3 || d2dMap.rows != 4)
			{
				cout << "Invalid Calibration Data: Stereo Matrix Size" << endl;
				goodInput = false;
				return;
			}
			
		}

	public:
		Mat R;
		Mat T;
		Mat E;
		Mat F;
		Mat d2dMap;
		double rms;
		bool goodInput;
};

class Camera
{	
	public:
		Camera(): goodInput(false) {};
		enum CamType {NONE, USB, IP};

		void read(const FileNode& node)
		{
			node["Camera_Matrix"] >> cameraMat;
			node["Distortion_Coefficients"] >> distCoeffs;
			node["Input"] >> input;
			node["Image_Height"] >> imageSize.height;
			node["Image_Width"] >> imageSize.width;
			node["Rotation_Matrix"] >> R;
			node["Projection_Matrix"] >> P;
			node["Map_1"] >> map1;
			node["Map_2"] >> map2;
			node["ROI_x"] >> roi.x;
			node["ROI_y"] >> roi.y;
			node["ROI_height"] >> roi.height;
			node["ROI_width"] >> roi.width;
			interprate();
		}

		void interprate()
		{
			goodInput = true;
			cap.open(input);
			if(!cap.isOpened())
			{
				cout << "Failed to open camera" << endl;
				goodInput = false;
				return;
			}
			
			if(cameraMat.cols != 3 ||  R.cols != 3 || P.cols != 4)
			{
				cout << "Invalid Calibration Data: Camera" << endl;
				goodInput = false;
				return;
			}
			if(map1.cols != imageSize.width || map2.cols != imageSize.width)
			{
				cout << "Invalid Camera Map Size" << endl;
				goodInput = false;
				return;
			}
		}

		void grab()
		{
			cap.grab();
		}

		Mat retrieve()
		{
			Mat frame;
			cap.retrieve(frame,0);
			return frame;
		}

	public:
		VideoCapture cap;
		Mat cameraMat;
		Mat distCoeffs;
		Mat map1;
		Mat map2;
		Mat R;
		Mat P;
		Rect roi;
		Size imageSize;
		string input;
		bool goodInput;
};

static void read(const FileNode& node, Camera& cam, const Camera& default_value = Camera())
{
	if(node.empty())
		cam = default_value;
	else
		cam.read(node);
}

static void read(const FileNode& node, CalibData& data, const CalibData& default_value = CalibData())
{
	if(node.empty())
		data = default_value;
	else
		data.read(node);
}

static void read(const FileNode& node, Settings& s, const Settings& default_value = Settings())
{
	if(node.empty())
		s = default_value;
	else
		s.read(node);
}

static void help()
{
	cout 	<< "This is the stereovision disparity mapping module. \n"
		<< "'s' to save point cloud and corresponding left and right images, distorted. \n"
		<< "'c' to switch between colour and grayscale disparity map. \n"
		<< "'esc' to quit the program" 
			<< endl;
}







int main(int argc, char* argv[])
{
	help();
	const char ESC_KEY = 27;
	CalibData data;
	Settings s;
	Camera LeftCam, RightCam;
	const string calibFile = argc > 2 ? argv[1] : "~/wall-e/savefiles/stereo_calib.xml";
	const string settingsFile = argc > 2 ? argv[2] : "~/wall-e/src/disparity_settings_default.xml"; 
	if(argc <= 2)
		cout << "It is not recommended to use the default settings" << endl;

	FileStorage fs;
	fs.open(calibFile , FileStorage::READ);
	if(!fs.isOpened())
	{
		cout << "Could not open the calibration file: \"" << calibFile << "\"" << endl;
		return -1;
	}
	
	fs["Stereovision_Parameters"] >> data;
	if(!data.goodInput)
	{
		cout << "Invalid Stereo Input" << endl;
		return -1;
	}

	fs["Left_Camera_Parameters"] >> LeftCam;
	if(!LeftCam.goodInput)
	{
		cout << "Invalid Left Camera Data" << endl;
		return -1;
	}
	
	fs["Right_Camera_Parameters"] >> RightCam;
	if(!RightCam.goodInput)
	{
		cout << "Invalid Right Camera Data" << endl;
		return -1;
	}
	fs.release();
	
	fs.open(settingsFile, FileStorage::READ);
	if(!fs.isOpened())
	{
		cout << "Could not open the settings file" << endl;
		return -1;
	}
	fs["Settings"] >> s;
	if(!s.goodInput)
	{
		cout << "Invalid settings input" << endl;
		return -1;
	}

	
	cout << "Calibration and Settings data accepted. \nBoth cameras opened" << endl;
	
	Mat leftView, rightView;
	Mat leftView8, rightView8; //rectified images
	Mat disp, disp8;
	int numDisps = (LeftCam.imageSize.width/8) + 15 & -16;
	namedWindow("Left",CV_WINDOW_AUTOSIZE);
	namedWindow("Right",CV_WINDOW_AUTOSIZE);
	namedWindow("Disparity",CV_WINDOW_AUTOSIZE);
	int minDisp = 0;
	int numberOfDisparities =  256;
	StereoBM bm;
	bm.state->roi1 = LeftCam.roi;
	bm.state->roi2 = RightCam.roi;
	bm.state->preFilterCap = 31;
	bm.state->SADWindowSize = 9;
	bm.state->minDisparity = minDisp;
	bm.state->numberOfDisparities = numberOfDisparities;
	bm.state->textureThreshold = 10;
	bm.state->uniquenessRatio = 15;
	bm.state->speckleWindowSize = 100;
	bm.state->speckleRange = 32;
	bm.state->disp12MaxDiff = 1;


	
	while(true)
	{
		LeftCam.grab();
		RightCam.grab();
		remap(LeftCam.retrieve(), leftView, LeftCam.map1, LeftCam.map2, INTER_LINEAR);
		remap(RightCam.retrieve(), rightView, RightCam.map1, RightCam.map2, INTER_LINEAR);
		cvtColor(leftView, leftView8, CV_RGB2GRAY);
		cvtColor(rightView,rightView8, CV_RGB2GRAY);
		//cout << "GOT HERE" << endl;
		//if(s.alg == STEREO_BM)
		//imshow("Left", leftView8);
		//imshow("Right", leftView);
		//waitKey(10000);
		bm(leftView8, rightView8, disp);
		//cout << "BUT NOT HERE" << endl;
		/*else if(s.alg == STEREO_VAR )
			var(leftView, rightView, disp);
		else
			sgbm(leftView, rightView, disp);
		*/
		//if( alg != STEREO_VAR)
		disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
		//else
		//	disp.convertTo(disp8, CV_8U);

		imshow("Left", leftView);
		imshow("Right", rightView);
		imshow("Disparity", disp8);

		char k = (char)waitKey(10);
		if(k == ESC_KEY)
			break;

	
	}
	return 0;
}

