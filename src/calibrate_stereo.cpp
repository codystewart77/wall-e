/* 
** C++ program developped using OpenCV to calibrate a stereo camera pair.  It assumes 
** that both cameras have already been calibrated independantly, and that the outputs
** of the calibration are by default called left_calib.xml and right_calib.xml.
** Default calibration options are loaded from stereo_options.xml
** 
** It outputs are the fundamental, essential, rotation and translation matrices, as 
** well as the internal parameters of both cameras to stereo_calib.xml
**
** Author: Cody Stewart
** Date: May 21, 2013
*/

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class Settings
{
public:
	Settings() : goodInput(false) {}
	enum Pattern { NOT_EXISTING, CHESSBOARD };

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{" << "BoardSize_Width"  << boardSize.width
                  << "BoardSize_Height" << boardSize.height
                  << "Square_Size"         << squareSize
           << "}";
    }
	void read(const FileNode& node)
	{
		node["BoardSize_Height"] >> boardSize.width;
		node["BoardSize_Width"] >> boardSize.height;
		node["Square_Size"] >> squareSize;
		node["Iteration_Criteria"] >> termcrit_iter;
		node["EPS_Criteria"] >> termcrit_eps;
		node["OutputFileName"] >> outputFileName;
		node["Left_Parameters"] >> leftParams;
		node["Right_Parameters"] >> rightParams;
		node["Output_FileName"] >> outputFileName;
		interprate();
	}
	
	void interprate()
	{
		showRectified = false;
		showDisparity = false;
		goodInput = true;
		if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
	}
	
	static bool readStringList( const string& filename, vector<string>& l )
    {
        l.clear();
        FileStorage fs(filename, FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != FileNode::SEQ )
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
            l.push_back((string)*it);
        return true;
    }
    
public:
	Size boardSize;
	float squareSize;
	float termcrit_iter;
	float termcrit_eps;
	bool showRectified;
	bool showDisparity;
	bool swap;
	bool goodInput;
	string leftParams;
	string rightParams;
	string outputFileName;


};

class Camera
{
public:
	Camera() : goodInput(false) {}
	enum InputType { INVALID, IP, USB };
	
	void read(const FileStorage& fs)
	{
		fs["Camera_Type"] >> cameraType;
		fs["input"] >> input;
		fs["image_Width"] >> imageSize.width;
		fs["image_Height"] >> imageSize.height;
		fs["Camera_Matrix"] >> cameraMatrix;
		fs["Distortion_Coefficients"] >> distCoeffs;
		fs["Avg_Reprojection_Errors"] >> avgReprojErr; 
		interprate();
	}	
	
	void interprate()
	{
		goodInput = true;
        inputType = IP;
        if(!cameraType.compare("IP")) inputType = IP;
        if(!cameraType.compare("USB")) inputType = USB;
        if (inputType == INVALID)
        {
            cerr << " Inexistent input: " << input;
            goodInput = false;
        }
        
		if (input.empty())      // Check for valid input
        	goodInput = false;
        else
        {	
        	if(inputType == IP)
        	{
                inputCapture.open(input);
        	}
        	if(inputType == USB)
        	{
        		if (input[0] >= '0' && input[0] <= '9')
	            {
    	            stringstream ss(input);
    	            ss >> cameraID;
    	            inputCapture.open(cameraID);
    	        }
    	        else
    	        	inputType = INVALID;
        	}
        	
        }
		
		if(inputType == INVALID)
		{
			cerr << " Inexistant input: " << input;
			goodInput = false;
		}
		if(!inputCapture.isOpened())
		{
			cerr << " Unable to open camera ";
			goodInput = false;
		}
	}
	
    Mat nextImage()
    {
        Mat result;
        inputCapture.read(result);
        return result;
    }

    void grab()
    {
	    inputCapture.grab();	
    }
	
    Mat retrieve()
    {
	    Mat result;
	    inputCapture.retrieve(result);
	    return result;
    }
    
public:
	string cameraType;
	string input;
	Size imageSize;
	Mat cameraMatrix;
	Mat distCoeffs;
	double avgReprojErr;
	VideoCapture inputCapture;
	int cameraID;
	InputType inputType;
	bool goodInput;
};

static bool runCalibration(double& rms, Settings& s, Camera& LeftCam, Camera& RightCam, vector<vector<Point2f> > lefImagePoints,
		vector<vector<Point2f> > rightImagePoints, Mat& R, Mat& T, Mat& E, Mat& F);
static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners);
static void saveStereoParams(Settings& s , Camera& LeftCam, Camera& RightCam, Mat& R, Mat& T, Mat& E,
		Mat& F, double rms, Mat& leftR, Mat& rightR, Mat& leftP, Mat& rightP, Mat& Q, Mat* leftMap,
		 Mat* rightMap, Rect& leftValidRoi, Rect& rightValidRoi);
static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

static void help()
{
	cout << "This is the stereovision calibration executeable" << endl;
	cout << "'t' to switch the left and right images" << endl;
	cout << "'g' to capture image pair" << endl;
	cout << "'c' to calibrate with images obtained" << endl;
	cout << "'r' to reset the calibration process" << endl;
	cout << "'s' to save the calibration" << endl;
	cout << "'d' to show the disparity image" << endl;
	cout << "'f' to toggle the rectified images" << endl;
	cout << "'esc' to quit" << endl;
}

int main(int argc, char* argv[])
{
	help();
	//Mat frameL, frameR;
	Settings s;
	Camera LeftCam, RightCam;
	const string inputSettingsFile = argc > 1 ? argv[1] : "stereo_default.xml";
	
	FileStorage fs(inputSettingsFile, FileStorage::READ);
	if(!fs.isOpened())
	{
		cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
		return -1;
	}
	fs["Settings"] >> s;
	fs.release();
	
	if(!s.goodInput)
	{
		cout << " Invalid input detected " << endl;
		return -1;
	}
	else
		cout << "Settings accepted" << endl;
	
	fs.open(s.leftParams, FileStorage::READ);
	if(!fs.isOpened())
	{
		cout << "Could not open the left parameters file: \"" << s.leftParams << "\"" << endl;
		return -1;
	}
	LeftCam.read(fs);
	fs.release();
	
	if(!LeftCam.goodInput)
	{
		cout << " Left Camera parameters not accepted " << endl;
		return -1;
	}
	else
		cout << "Left Camera opened" << endl;
	waitKey(30);
	
	fs.open(s.rightParams, FileStorage::READ);
	if(!fs.isOpened())
	{
		cout << "Could not open the right parameters file: \"" << s.rightParams << "\"" << endl;
		return -1;
	}
	RightCam.read(fs);
	fs.release();
	
	if(!RightCam.goodInput)
	{
		cout << " Right Camera parameters not accepted " << endl;
		return -1;
	}	
	else
		cout << "Right Camera opened" << endl;

	/* 
	** Calibrated cameras have now been loaded 
	** Now need to start calibrating the stereo system
	*/
	Mat leftView, rightView;
	vector<vector<Point2f> > leftImagePoints, rightImagePoints;
	Mat R, T, E, F; //Stereovision matrices
	Mat leftR, rightR, leftP, rightP; //rectification and projection matrices
	Mat Q; //Disparity to depth mapping
	Rect leftValidRoi, rightValidRoi; //valid regions of interest
	Mat leftMap[2];
	Mat rightMap[2];
	double rms;		//error
	Mat canvas;
	double sf;
	int w, h;
	sf = 600./MAX(LeftCam.imageSize.width, LeftCam.imageSize.height);
	w = cvRound(LeftCam.imageSize.width*sf);
	h = cvRound(LeftCam.imageSize.height*sf);
	canvas.create(h,w*2, CV_8UC3);
	bool isCalibrated = false;
	Size imageSize;
	const char ESC_KEY = 27;

	//named windows: left, right and disparity
	namedWindow("Left",CV_WINDOW_AUTOSIZE);
	namedWindow("Right",CV_WINDOW_AUTOSIZE);
	namedWindow("Disparity",CV_WINDOW_AUTOSIZE);
	namedWindow("Rectified",CV_WINDOW_AUTOSIZE);
	
	while(true)
	{
		LeftCam.grab();
		RightCam.grab();
		leftView = LeftCam.retrieve();
		rightView = RightCam.retrieve();

		vector<Point2f> leftPointBuff, rightPointBuff;

		char key = (char)waitKey(20);
		
		if(key == 'g') //grab the frames
		{
			bool leftFound, rightFound;
			
			leftFound = findChessboardCorners(leftView, s.boardSize, leftPointBuff,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
			rightFound = findChessboardCorners(rightView, s.boardSize, rightPointBuff,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

			if(leftFound && rightFound)
			{	
				Mat leftViewGray, rightViewGray;
				cvtColor(leftView, leftViewGray, CV_BGR2GRAY);
				cvtColor(rightView, rightViewGray, CV_BGR2GRAY);

				cornerSubPix( leftViewGray, leftPointBuff, Size(11,11), Size(-1,-1), 
						TermCriteria( CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.1));
				cornerSubPix( rightViewGray, rightPointBuff, Size(11,11), Size(-1,-1),
						TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.1));

				leftImagePoints.push_back(leftPointBuff);
				rightImagePoints.push_back(rightPointBuff);
				
				cout << "Frames: " << leftImagePoints.size() << endl;
			}
		}
		
		if( key == ESC_KEY )
		{
			return 0;
		}
		
		if( key == 'c' && leftImagePoints.size() > 5)
		{
			//run calibration
			bool ok = runCalibration(rms ,s, LeftCam, RightCam, leftImagePoints, rightImagePoints, R, T, E, F);
			if(ok)
			{
				stereoRectify(LeftCam.cameraMatrix, LeftCam.distCoeffs, RightCam.cameraMatrix,  RightCam.distCoeffs,
					LeftCam.imageSize, R, T, leftR, rightR, leftP, rightP, Q, CALIB_ZERO_DISPARITY,
					1, LeftCam.imageSize, &leftValidRoi, &rightValidRoi);
				initUndistortRectifyMap(LeftCam.cameraMatrix, LeftCam.distCoeffs, leftR, leftP, LeftCam.imageSize,
						CV_16SC2, leftMap[0], leftMap[1]);
				initUndistortRectifyMap(RightCam.cameraMatrix, RightCam.distCoeffs, rightR, rightP, RightCam.imageSize,
						CV_16SC2, rightMap[0], rightMap[1]);						
				cout << "Calibration succeeded" << endl;
				isCalibrated = true;
				cout << "R:" << R << endl;
				cout << "T:" << T << endl;
				cout << "E:" << E << endl;
				cout << "F:" << F << endl;
				
			}
		}
		
		if(isCalibrated && key == 'f')
			s.showRectified = !s.showRectified;
			
		if(s.showRectified)
		{
			Mat LImg, RImg;
			remap(leftView, LImg, leftMap[0], leftMap[1], CV_INTER_LINEAR);
			remap(rightView, RImg, rightMap[0], rightMap[1], CV_INTER_LINEAR);
			Mat canvasPartL = canvas(Rect(0,0,w,h));
			Mat canvasPartR = canvas(Rect(w,0,w,h));
			resize(LImg, canvasPartL, canvasPartL.size(),0,0,CV_INTER_AREA);
			resize(RImg, canvasPartR, canvasPartR.size(),0,0,CV_INTER_AREA);
			imshow("Rectified", canvas);
		}
		else
		{
			imshow("Left", leftView);
			imshow("Right", rightView);
		}
		
		if(isCalibrated && s.showRectified && key =='d')
			s.showDisparity = !s.showDisparity;
			
		if(showDisparity)
		{
		
		
		}
		
		if(isCalibrated && key == 's')
		{
			saveStereoParams( s , LeftCam, RightCam, R, T, E, F, rms, leftR, rightR, leftP, rightP, Q,
					leftMap, rightMap, leftValidRoi, rightValidRoi);
		}
	}
}

static bool runCalibration(double& rms, Settings& s, Camera& LeftCam, Camera& RightCam, vector<vector<Point2f> > leftImagePoints,
			vector<vector<Point2f> > rightImagePoints, Mat& R, Mat& T, Mat& E, Mat& F)
{
	vector<vector<Point3f> > objPoints(1);
	calcBoardCornerPositions(s.boardSize, s.squareSize, objPoints[0]);
	
	objPoints.resize(leftImagePoints.size(), objPoints[0]);
	rms = stereoCalibrate( objPoints, leftImagePoints, rightImagePoints, LeftCam.cameraMatrix, LeftCam.distCoeffs,
			RightCam.cameraMatrix, RightCam.distCoeffs, LeftCam.imageSize, R, T, E, F, 
			TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30, 1e-6));

			
}

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners)
{
	corners.clear();
	for( int i =0; i < boardSize.height; ++i)
		for( int j = 0; j < boardSize.width; ++j)
			corners.push_back(Point3f(float(j*squareSize), float(i*squareSize),0));
}

static void saveStereoParams(Settings& s , Camera& LeftCam, Camera& RightCam, Mat& R, Mat& T, Mat& E,
		Mat& F, double rms, Mat& leftR, Mat& rightR, Mat& leftP, Mat& rightP, Mat& Q, Mat* leftMap,
		 Mat* rightMap, Rect& leftValidRoi, Rect& rightValidRoi)
{
	time_t tm;
	time( &tm );
	struct tm *t2 = localtime(&tm);
	char buf[1024];
	strftime( buf, sizeof(buf)-1, "%A_%B_%d_%Y_%X",t2);
	string filestring = s.outputFileName + ".xml";		
	FileStorage rs(filestring, FileStorage::WRITE );
	if(!rs.isOpened())
	{
		cout << "File Storage failed to open" << endl;
	}
	else
	{
		rs 
			<< "Time" << buf
			<< "Left_Camera_Parameters" 
			<< "{"
				<< "Camera_Type" << LeftCam.cameraType
				<< "Input" << LeftCam.input
				<< "Image_Height" << LeftCam.imageSize.height
				<< "Image_Width" << LeftCam.imageSize.width
				<< "Camera_Matrix" << LeftCam.cameraMatrix
				<< "Distortion_Coefficients" << LeftCam.distCoeffs
		 		<< "Avg_Reproj_Errs" << LeftCam.avgReprojErr
		 		<< "Rotation_Matrix" << leftR
		 		<< "Projection_Matrix" << leftP
		 		<< "Map_1" << leftMap[0]
		 		<< "Map_2" << leftMap[1]
		 		<< "Valid_ROI" << leftValidRoi
		 	<< "}"
		 	
			<< "Right_Camera_Parameters" 
			<< "{"
				<< "Camera_Type" << RightCam.cameraType
				<< "Input" << RightCam.input
				<< "Image_Height" << RightCam.imageSize.height
				<< "Image_Width" << RightCam.imageSize.width
				<< "Camera_Matrix" << RightCam.cameraMatrix
				<< "Distortion_Coefficients" << RightCam.distCoeffs
		 		<< "Avg_Reproj_Errs" << RightCam.avgReprojErr
		 		<< "Rotation_Matrix" << rightR
		 		<< "Projection_Matrix" << rightP
		 		<< "Map_1" << rightMap[0]
		 		<< "Map_2" << rightMap[1]
		 		<< "Valid_ROI" << rightValidRoi	 		
		 	<< "}"		 	
		 		
			<< "Stereovision_Parameters" 
			<< "{"
				<< "Rotation_Matrix" << R
				<< "Translation_Matrix" << T
				<< "Essential_Matrix" << E
				<< "Fundamental_Matrix" << F
				<< "RMS_Error" << rms
				<< "Disparity_to_Depth_Map" << Q
			<< "}";
			rs.release();
		cout << "File Saved" << endl;
	}


}
