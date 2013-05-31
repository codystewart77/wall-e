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
#include <fstream>

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

static void saveXYZ( const char* filename, const Mat& mat)
{
	const double max_z = 1.0e4;
	ofstream fp(filename, std::ofstream::out);
	
	if(!fp.is_open())
	{
		cout << "Failed to open file" << endl;
		return;
	}
	fp 	<< "VERSION  " << 1.0 << "\n"
		<< "FIELDS x y z \n"
		<< "SIZE 4 4 4 \n" 
		<< "TYPE F F F \n" 
		<< "COUNT 1 1 1 \n" 
		<< "WIDTH " << mat.cols << "\n"
		<< "HEIGHT " << mat.rows << "\n"
		<< "VIEWPOINT 0 0 0 1 0 0 0 \n" 
		<< "POINTS " << ( mat.cols * mat.rows) << "\n"
		<< "DATA ascii \n";
	for(int y = 0; y < mat.rows; y++)
	{
		for(int x = 0; x < mat.cols; x++)
		{
			Vec3f point = mat.at<Vec3f>(y,x);
			if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
			fp << point[0] << " " << point[1] << " " << point[2] << "\n";;
		}
	}
	fp.close();
}


//Trackbar Settings functions

//#ifdef STEREOBM
//StereoBM Correspondances

void BMSetMinDisp(int val, void* ptr)
{	
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	if(bm->state->minDisparity >= 0)
		bm->state->minDisparity = val;
	else
		bm->state->minDisparity = -val;
}

void BMflipMinDisp(int val, void* ptr)
{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	bm->state->minDisparity = -bm->state->minDisparity;
}

void BMSetNumDisp( int val, void* ptr )
{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	bm->state->numberOfDisparities = (val + 1) * 16  ;
}

void BMSetPreFilterCap( int val , void* ptr )
{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	bm->state->preFilterCap = val + 1;
}

void BMSetSAD( int val, void* ptr )
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
void BMSetTextureThresh( int val, void* ptr )
{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	bm->state->textureThreshold = val;
}
void BMSetUniqueness( int val, void* ptr )

{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	bm->state->uniquenessRatio = val;
}

void BMSetSpeckleWindowSize(int val, void* ptr)
{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	bm->state->speckleWindowSize = val;
}

void BMSetSpeckleRange(int val, void* ptr)
{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	bm->state->speckleRange = val;
}

void BMSetMaxDiff(int val, void* ptr)
{
	StereoBM *bm= static_cast<StereoBM*>(ptr);
	bm->state->disp12MaxDiff = static_cast<float>(val/100.);
}
//#endif

//#ifdef STEREOVAR
//StereoVar trackbar functions
void VarSetMinDisp(int val ,void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	bm->minDisp = -val;
}

void VarSetMaxDisp(int val, void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	bm->maxDisp = -val;
}

void VarSetLevels(int val ,void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	bm->levels = val;
}

void VarSetPyrScale(int val ,void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	if(val != 0)
		bm->pyrScale = val/10.0f;
	
}


void VarSetNIts(int val ,void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	bm->nIt = val;
}

void VarSetPoly_n(int val ,void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	if(val <= 3)
		bm->poly_n = 3;
	else if (val > 3 && val <= 5)
		bm->poly_n = 5;
	else
		bm->poly_n = 7;
}

void VarSetPoly_sigma(int val ,void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	bm->minDisp = val/100.0f;
}

void VarSetFi(int val ,void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	bm->fi = val/10.0f;
}

void VarSetLambda(int val ,void* ptr)
{
	StereoVar *bm = static_cast<StereoVar*>(ptr);
	bm->lambda = val/100.0f;;
}
//#endif


void SGBMSetPreFilterCap(int val, void* ptr)
{
	StereoSGBM *bm = static_cast<StereoSGBM*>(ptr);
	bm->preFilterCap = val;
}

void SGBMSetSADWindowSize(int val, void* ptr)
{
	StereoSGBM *bm = static_cast<StereoSGBM*>(ptr);
	if(val % 2 == 1)
	{
		bm->SADWindowSize = val;
		bm->P1 = 8*3*val*val;
		bm->P2 = 32*3*val*val;
	}
	else
	{
		val = val+1;
		bm->SADWindowSize = val;
		bm->P1 = 8*3*val*val;
		bm->P2 = 32*3*val*val;
	}
}

void SGBMSetMinDisparity(int val, void* ptr)
{
	StereoSGBM *bm = static_cast<StereoSGBM*>(ptr);
	bm->minDisparity = -val;
}

void SGBMSetNumberOfDisparities(int val, void* ptr)
{
	StereoSGBM *bm = static_cast<StereoSGBM*>(ptr);
	bm->numberOfDisparities = val*16;
}

void SGBMSetUniquenessRatio(int val, void* ptr)
{
	StereoSGBM *bm = static_cast<StereoSGBM*>(ptr);
	bm->uniquenessRatio = val;
}

void SGBMSetSpeckleWindowSize(int val, void* ptr)
{
	StereoSGBM *bm = static_cast<StereoSGBM*>(ptr);
	bm->speckleWindowSize = val;
}

void SGBMSetSpeckleRange(int val, void* ptr)
{
	StereoSGBM *bm = static_cast<StereoSGBM*>(ptr);
	bm->speckleRange = val*16;
}
		
void SGBMSetMaxDiff(int val, void* ptr)
{
	StereoSGBM *bm = static_cast<StereoSGBM*>(ptr);
	bm->disp12MaxDiff = val;
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
	
	namedWindow("Left",CV_WINDOW_AUTOSIZE);
	namedWindow("Right", CV_WINDOW_AUTOSIZE);
	namedWindow("Disparity",CV_WINDOW_AUTOSIZE);
	namedWindow("trackbars", CV_WINDOW_AUTOSIZE);
	
	moveWindow("Left", 0, 0);
	moveWindow("Right", 480, 0);
	moveWindow("Disparity", 0, 360);
	moveWindow("trackbars", 480, 360);
	int BMTrackMinDisp = 1;
	int BMTrackFlipMinDisp = 0;
	int BMTrackNumberOfDisparities = 2;
	int BMTrackPreFilterCap = 20;
	int BMTrackSADWindowSize = 21;
	int BMTrackTextureThreshold = 1;
	int BMTrackUniquenessRatio = 16;
	int BMTrackSpeckleWindowSize = 175;
	int BMTrackSpeckleRange = 3;
	int BMTrackDisp12MaxDiff = 100;
	
	int SGBMPreFilterCap = 0;
	int SGBMSADWindowSize = 0;
	int SGBMP1 = 8*3*SGBMSADWindowSize*SGBMSADWindowSize;
	int SGBMP2 = 32*3*SGBMSADWindowSize*SGBMSADWindowSize;
	int SGBMMinDisparity = 0;
	int SGBMNumberOfDisparities = 5;
	int SGBMUniquenessRatio = 0;
	int SGBMSpeckleWindowSize = 150;
	int SGBMSpeckleRange = 2;
	int SGBMMaxDiff = 10;

	int VarLevels = 1;
	int VarPyrScale = 17;
	int VarnIt = 12;
	int VarPoly_n = 3;
	int VarPoly_sigma = 0;
	int VarFi = 80;
	int VarLambda = 20;
	int VarNumberOfDisparities = 0;	
	StereoBM bm;
	StereoVar var;
	StereoSGBM sgbm;
	
	
	if(s.alg == Settings::STEREO_BM)
	{
		bm.state->roi1 = LeftCam.roi;
		bm.state->roi2 = RightCam.roi;
		bm.state->preFilterCap = BMTrackPreFilterCap;
		bm.state->SADWindowSize = BMTrackSADWindowSize;
		bm.state->minDisparity = BMTrackMinDisp;
		bm.state->numberOfDisparities = BMTrackNumberOfDisparities*16;
		bm.state->textureThreshold = BMTrackTextureThreshold;
		bm.state->uniquenessRatio = BMTrackUniquenessRatio;
		bm.state->speckleWindowSize = BMTrackSpeckleWindowSize;
		bm.state->speckleRange = BMTrackSpeckleRange;
		bm.state->disp12MaxDiff = BMTrackDisp12MaxDiff/100.;
	
		createTrackbar("minDisp", "trackbars", &BMTrackMinDisp, 16, BMSetMinDisp, &bm);
		createTrackbar("flipminDisp","trackbars", &BMTrackFlipMinDisp, 1, BMflipMinDisp, &bm);
		createTrackbar("numDisp","trackbars", &BMTrackNumberOfDisparities , 100 , BMSetNumDisp, &bm);
		createTrackbar("preFilterCap","trackbars", &BMTrackPreFilterCap , 62, BMSetPreFilterCap, &bm);
		createTrackbar("SADWindowSize","trackbars", &BMTrackSADWindowSize , 255, BMSetSAD, &bm);
		createTrackbar("TextureThresh","trackbars", &BMTrackTextureThreshold , 10, BMSetTextureThresh, &bm);
		createTrackbar("UniquenessRatio","trackbars", &BMTrackUniquenessRatio , 100, BMSetUniqueness, &bm);
		createTrackbar("SpeckleWindowSize","trackbars", &BMTrackSpeckleWindowSize , 200, BMSetSpeckleWindowSize, &bm);
		createTrackbar("SpeckleRange","trackbars", &BMTrackSpeckleRange , 100, BMSetSpeckleRange, &bm);
		createTrackbar("MaxDiff","trackbars", &BMTrackDisp12MaxDiff , 2000, BMSetMaxDiff, &bm);
	}
	else if(s.alg == Settings::STEREO_SGBM)
	{
		sgbm.preFilterCap = SGBMPreFilterCap;
		sgbm.SADWindowSize = SGBMSADWindowSize;
		sgbm.P1 = SGBMP1;
		sgbm.P2 = SGBMP2;
		sgbm.minDisparity = SGBMMinDisparity;
		sgbm.numberOfDisparities = SGBMNumberOfDisparities*16;
		sgbm.uniquenessRatio = SGBMUniquenessRatio;
		sgbm.speckleWindowSize = SGBMSpeckleWindowSize;
		sgbm.speckleRange = SGBMSpeckleRange*16;
		sgbm.disp12MaxDiff = SGBMMaxDiff;
		sgbm.fullDP = false;
	
		createTrackbar("PreFilter","trackbars", &SGBMPreFilterCap , 100, SGBMSetPreFilterCap , &sgbm);
		createTrackbar("SADWindowSize","trackbars", &SGBMSADWindowSize , 30, SGBMSetSADWindowSize , &sgbm);
		createTrackbar("MinDisparity","trackbars", &SGBMMinDisparity , 16, SGBMSetMinDisparity , &sgbm);
		createTrackbar("NumberOfDisparities","trackbars", &SGBMNumberOfDisparities , 16, SGBMSetNumberOfDisparities , &sgbm);
		createTrackbar("UniquenessRatio","trackbars", &SGBMUniquenessRatio , 30, SGBMSetUniquenessRatio , &sgbm);
		createTrackbar("SpeckleWindowSize","trackbars", &SGBMSpeckleWindowSize , 300, SGBMSetSpeckleWindowSize , &sgbm);
		createTrackbar("SpeckleRange","trackbars", &SGBMSpeckleRange , 20, SGBMSetSpeckleRange , &sgbm);
		createTrackbar("MaxDiff","trackbars", &SGBMMaxDiff , 100, SGBMSetMaxDiff , &sgbm);

		
	}
	else if(s.alg == Settings::STEREO_VAR)
	{
		//Initialization if alg = StereoVar
		var.levels = VarLevels;
		var.pyrScale = VarPyrScale/10.0f;
		var.nIt = VarnIt;
		var.minDisp = -VarNumberOfDisparities;
		var.maxDisp = 0;
		var.poly_n = VarPoly_n;
		var.poly_sigma = VarPoly_sigma/100.;;
		var.fi = VarFi/10.;
		var.lambda = VarLambda/100.;
		var.penalization = var.PENALIZATION_TICHONOV;
		var.cycle = var.CYCLE_V;
	
		createTrackbar("minDisp", "trackbars", &VarNumberOfDisparities,32 , VarSetMinDisp, &var);
		createTrackbar("levels", "trackbars", &VarLevels, 4, VarSetLevels, &var );	
		createTrackbar("pyrScale", "trackbars", &VarPyrScale, 100, VarSetPyrScale, &var );	
		createTrackbar("nIts", "trackbars", &VarnIt, 50, VarSetNIts, &var );	
		createTrackbar("poly_n", "trackbars", &VarPoly_n, 7, VarSetPoly_n, &var );	
		createTrackbar("poly_sigma", "trackbars", &VarPoly_sigma, 2200, VarSetPoly_sigma, &var );	
		createTrackbar("fi", "trackbars", &VarFi, 200, VarSetFi, &var );	
		createTrackbar("lambda", "trackbars", &VarLambda, 100, VarSetLambda, &var );	
	}
	
	Mat depthImg;
	Mat cloud;
	while(true)
	{
		LeftCam.grab();
		RightCam.grab();
		remap(LeftCam.retrieve(), leftView, LeftCam.map1, LeftCam.map2, INTER_LINEAR);
		remap(RightCam.retrieve(), rightView, RightCam.map1, RightCam.map2, INTER_LINEAR);
		
		if(s.alg == Settings::STEREO_BM)
		{
			cvtColor(leftView, leftView8, CV_RGB2GRAY);
			cvtColor(rightView,rightView8, CV_RGB2GRAY);
			bm(leftView8, rightView8, disp8);
			disp8.convertTo(disp, CV_8U);
		}
		else if(s.alg == Settings::STEREO_SGBM)
		{
			sgbm(leftView, rightView, disp8);
			disp8.convertTo(disp, CV_8U);
		}
		else if(s.alg == Settings::STEREO_VAR)
			var(leftView, rightView, disp);

		
		imshow("Left", leftView);
		imshow("Disparity", disp);
		imshow("Right", rightView);
		char k = (char)waitKey(30);
		if(k == ESC_KEY)
			break;
		
		if(k == 's')
		{
			reprojectImageTo3D(disp8, cloud, data.d2dMap, true);
			saveXYZ("pointcloud2.pcd",cloud);
		}
	
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

