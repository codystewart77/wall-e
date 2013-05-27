/* File to take a calibrated stereo pair and map the disparity
** By default will load disparity_settings_default.xml
** and stereo_calib_default.xml
** Author: Cody Stewart
** May 23, 2013
*/
#define STEREOBM

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

int cn = 1;
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

void cvtDepth2Cloud(const Mat& depth, Mat& cloud, const Mat& cameraMatrix)
{
	const float inv_fx = 1.f/cameraMatrix.at<float>(0,0);
	const float inv_fy = 1.f/cameraMatrix.at<float>(1,1);
	const float ox = cameraMatrix.at<float>(0,2);
	const float oy = cameraMatrix.at<float>(1,2);
	cloud.create( depth.size(), CV_32FC3);
	for(int y = 0; y< cloud.rows; y++)
	{
		Point3f* cloud_ptr = (Point3f*)cloud.ptr(y);
		const float* depth_ptr = (const float*) depth.ptr(y);
		for(int x = 0; x < cloud.cols; x++)
		{
			float z = depth_ptr[x];
			cloud_ptr[x].x = (x - ox) * z * inv_fx;
			cloud_ptr[x].y = (y - oy) * z * inv_fy;
			cloud_ptr[x].z = z;
		}
	}
}

static void saveXYZ( const char* filename, const Mat& mat)
{
	const double max_z = 1.0e5;
	FILE* fp = fopen(filename, "wt");
	for(int y = 0; y < mat.rows; y++)
	{
		for(int x = 0; x < mat.cols; x++)
		{
			Vec3f point = mat.at<Vec3f>(y,x);
			if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
			fprintf(fp, "%f %f %f\n",point[0], point[1], point[2]);
		}
	}
	fclose(fp);
}


//Trackbar Settings functions

#ifdef STEREOBM
//StereoBM Correspondances

void setMinDisp(int val, void* ptr)
{	
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	if(bm->state->minDisparity >= 0)
		bm->state->minDisparity = val;
	else
		bm->state->minDisparity = -val;
}

void flipMinDisp(int val, void* ptr)
{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	bm->state->minDisparity = -bm->state->minDisparity;
}

void setNumDisp( int val, void* ptr )
{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	bm->state->numberOfDisparities = (val + 1) * 16  ;
}

void setPreFilterCap( int val , void* ptr )
{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	bm->state->preFilterCap = val + 1;
}

void setSAD( int val, void* ptr )
{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	if(val < 5)
		val = 5;
	if(val > 255)
		val = 255;

	if(val % 2 != 1)
		bm->state->SADWindowSize = val + 1;
	else
		bm->state->SADWindowSize = val;
	
}
void setTextureThresh( int val, void* ptr )
{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	bm->state->textureThreshold = val;
}
void setUniqueness( int val, void* ptr )

{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	bm->state->uniquenessRatio = val;
}

void setSpeckleWindowSize(int val, void* ptr)
{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	bm->state->speckleWindowSize = val;
}

void setSpeckleRange(int val, void* ptr)
{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	bm->state->speckleRange = val;
}

void setMaxDiff(int val, void* ptr)
{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	bm->state->disp12MaxDiff = static_cast<float>(val/100.);
}
#endif

#ifdef STEREOVAR
//StereoVar trackbar functions
void setMinDisp(int val ,void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	bm->minDisp = - val*16;
}

void setLevels(int val ,void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	bm->levels = val;
}

void setPyrScale(int val ,void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	bm->pyrScale = val/10.0f;
}


void setNIts(int val ,void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	bm->nIt = val;
}

void setPoly_n(int val ,void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	if(val <= 3)
		bm->poly_n = 3;
	else if (val > 3 && val <= 5)
		bm->poly_n = 5;
	else
		bm->poly_n = 7;
}

void setPoly_sigma(int val ,void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	bm->minDisp = val/100.0f;
}

void setFi(int val ,void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	bm->fi = val/10.0f;
}

void setLambda(int val ,void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	bm->lambda = val/100.0f;;
}
#endif






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
	//int numDisps = (LeftCam.imageSize.width/8) + 15 & -16;
	namedWindow("Left",CV_WINDOW_AUTOSIZE);
//	namedWindow("Right",CV_WINDOW_AUTOSIZE);
	namedWindow("Disparity",CV_WINDOW_AUTOSIZE);
	int trackMinDisp = 0;
	int trackFlipMinDisp = 0;
	int trackNumberOfDisparities = 10;
	int trackPreFilterCap = 6;
	int trackSADWindowSize = 9;
	int trackTextureThreshold = 10;
	int trackUniquenessRatio = 15;
	int trackSpeckleWindowSize = 100;
	int trackSpeckleRange = 31;
	int trackDisp12MaxDiff = 100;
	//Mat quickView;
	//LeftCam.grab();
	//quickView = LeftCam.retrieve();
	//int cn = quickView.channels();

#ifdef STEREOBM
	//Initialization if alg = StereoBM
	
	StereoBM bm;
	bm.state->roi1 = LeftCam.roi;
	bm.state->roi2 = RightCam.roi;
	bm.state->preFilterCap = trackPreFilterCap;
	bm.state->SADWindowSize = trackSADWindowSize;
	bm.state->minDisparity = trackMinDisp;
	bm.state->numberOfDisparities = trackNumberOfDisparities*16;
	bm.state->textureThreshold = trackTextureThreshold;
	bm.state->uniquenessRatio = trackUniquenessRatio;
	bm.state->speckleWindowSize = trackSpeckleWindowSize;
	bm.state->speckleRange = trackSpeckleRange;
	bm.state->disp12MaxDiff = trackDisp12MaxDiff/100.;
#endif

#ifdef STEREOVAR
	int levels = 3;
	int pyrScale = 50;
	int nIt = 25;
	int poly_n = 3;
	int poly_sigma = 0;
	int fi = 150;
	int lambda = 30;
	//Initialization if alg = StereoVar
	StereoVar bm;
	bm.levels = 3;
	bm.pyrScale = 0.5;
	bm.nIt = 25;
	bm.minDisp = -trackNumberOfDisparities*16;
	bm.maxDisp = 0;
	bm.poly_n = 3;
	bm.poly_sigma = 0.0;
	bm.fi = 15.0f;
	bm.lambda = 0.03f;
	bm.penalization = bm.PENALIZATION_TICHONOV;
	bm.cycle = bm.CYCLE_V;
	bm.flags = bm.USE_SMART_ID | bm.USE_AUTO_PARAMS | bm.USE_INITIAL_DISPARITY | bm.USE_MEDIAN_FILTERING;
#endif
	


	namedWindow("trackbars", CV_WINDOW_AUTOSIZE);
	
	//StereoBM trackbars
#ifdef STEREOBM	
	createTrackbar("minDisp", "trackbars", &trackMinDisp, 16, setMinDisp, &bm);
	createTrackbar("flipminDisp","trackbars", &trackFlipMinDisp, 1, flipMinDisp, &bm);
	createTrackbar("numDisp","trackbars", &trackNumberOfDisparities , 100 , setNumDisp, &bm);
	createTrackbar("preFilterCap","trackbars", &trackPreFilterCap , 62, setPreFilterCap, &bm);
	createTrackbar("SADWindowSize","trackbars", &trackSADWindowSize , 255, setSAD, &bm);
	createTrackbar("TextureThresh","trackbars", &trackTextureThreshold , 10, setTextureThresh, &bm);
	createTrackbar("UniquenessRatio","trackbars", &trackUniquenessRatio , 100, setUniqueness, &bm);
	createTrackbar("SpeckleWindowSize","trackbars", &trackSpeckleWindowSize , 200, setSpeckleWindowSize, &bm);
	createTrackbar("SpeckleRange","trackbars", &trackSpeckleRange , 100, setSpeckleRange, &bm);
	createTrackbar("MaxDiff","trackbars", &trackDisp12MaxDiff , 2000, setMaxDiff, &bm);
#endif

	//StereoVar trackbars
#ifdef STEREOVAR
	createTrackbar("minDisp", "trackbars", &trackNumberOfDisparities, 16, setMinDisp, &bm);
	createTrackbar("levels", "trackbars", &levels, 5, setLevels, &bm );	
	createTrackbar("pyrScale", "trackbars", &pyrScale, 100, setPyrScale, &bm );	
	createTrackbar("nIts", "trackbars", &nIt, 50, setNIts, &bm );	
	createTrackbar("poly_n", "trackbars", &poly_n, 7, setPoly_n, &bm );	
	createTrackbar("poly_sigma", "trackbars", &poly_sigma, 2200, setPoly_sigma, &bm );	
	createTrackbar("fi", "trackbars", &fi, 200, setFi, &bm );	
	createTrackbar("lambda", "trackbars", &lambda, 100, setLambda, &bm );	
#endif
	Mat depthImg;
	
	while(true)
	{
		LeftCam.grab();
		RightCam.grab();
		remap(LeftCam.retrieve(), leftView, LeftCam.map1, LeftCam.map2, INTER_LINEAR);
		remap(RightCam.retrieve(), rightView, RightCam.map1, RightCam.map2, INTER_LINEAR);
		//waitKey(10000);
#ifdef STEREOBM
		cvtColor(leftView, leftView8, CV_RGB2GRAY);
		cvtColor(rightView,rightView8, CV_RGB2GRAY);
		bm(leftView8, rightView8, disp);
#endif
#ifdef STEREOVAR
		bm(leftView, rightView, disp);
#endif
		
		//normalize(disp,disp8,0,255, CV_8U);
		//disp.convertTo(disp8, CV_8U, 255/(trackNumberOfDisparities*16.));
		//else
		disp.convertTo(disp8, CV_8U);

		imshow("Left", leftView);
		//imshow("Right", rightView);
		imshow("Disparity", disp8);
		//imshow("Depth",depthImg);
		char k = (char)waitKey(10);
		if(k == ESC_KEY)
			break;

	
	}
/*	
	disp.convertTo(disp8, CV_8U, 255/(trackNumberOfDisparities*16*16.));
	reprojectImageTo3D(disp, depthImg, data.d2dMap);
	saveXYZ("../savefiles/disp1.pcl",depthImg);

	
	reprojectImageTo3D(disp8, depthImg, data.d2dMap);
	saveXYZ("../savefiles/disp8.pcl",depthImg);
*/	
	return 0;
}

