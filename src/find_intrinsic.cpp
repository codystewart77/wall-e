/* 
**	Modification of the OpenCV sample "camera_calibration.cpp"
**	for focus on IP and USB cameras.
**
**  Cody Stewart
**	Latest edit: May 21, 2013
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

//help function, always called when executable runs
static void help()
{
	cout 	<< "This is the camera calibration program for use with a chessboard pattern." << endl
			<< "Usage:" << endl
			<< "'g' to capture a frame, will notify if not a valid frame" << endl
			<< "'c' to start calibration when desired, suggest at least 16 frames" << endl
			<< "'s' to save the calibration when complete" << endl
			<< "'u' to show the undistroted image after calibration" << endl
			<< "'r' to reset the calibration and clear the captured frames" << endl
			<< "'esc' to leave the program, will save if calibrated" << endl;
}

//the settings class, loads default2.xml if no other file supplied
class Settings
{
public:
    Settings() : goodInput(false) {}
    enum Pattern { NOT_EXISTING , CHESSBOARD };
    enum InputType { INVALID , IP , USB };

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{" << "BoardSize_Width"  << boardSize.width
                  << "BoardSize_Height" << boardSize.height
                  << "Square_Size"         << squareSize
                  << "Calibrate_FixAspectRatio" << aspectRatio
                  << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
                  << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint
                  << "Write_DetectedFeaturePoints" << bwritePoints
                  << "Write_extrinsicParameters"   << bwriteExtrinsics
                  << "Write_outputFileName"  << outputFileName
                  << "Show_UndistortedImage" << showUndistorted
                  << "Input_FlipAroundHorizontalAxis" << flipVertical
                  << "Camera_Type" << cameraType  
                  << "Input" << input
           << "}";
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["BoardSize_Width" ] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Square_Size"]  >> squareSize;
        node["Calibrate_FixAspectRatio"] >> aspectRatio;
        node["Write_DetectedFeaturePoints"] >> bwritePoints;
        node["Write_extrinsicParameters"] >> bwriteExtrinsics;
        node["Write_outputFileName"] >> outputFileName;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
        node["Show_UndistortedImage"] >> showUndistorted;
        node["Camera_Type"] >> cameraType;
        node["Input"] >> input;
        interprate();
    }
    void interprate()
    {
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
        
        inputType = INVALID;
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
                cameraString = input;
                inputCapture.open(cameraString);
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
		
        flag = 0;
        if(calibFixPrincipalPoint) flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
        if(calibZeroTangentDist)   flag |= CV_CALIB_ZERO_TANGENT_DIST;
        if(aspectRatio)            flag |= CV_CALIB_FIX_ASPECT_RATIO;
        calibrationPattern = CHESSBOARD;
    }
    
    Mat nextImage()
    {
        Mat result;
        inputCapture.read(result);
        return result;
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
    Size boardSize;            // The size of the board -> Number of items by width and height
    float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
    float aspectRatio;         // The aspect ratio
    bool bwritePoints;         //  Write detected feature points
    bool bwriteExtrinsics;     // Write extrinsic parameters
    bool calibZeroTangentDist; // Assume zero tangential distortion
    bool calibFixPrincipalPoint;// Fix the principal point at the center
    bool flipVertical;          // Flip the captured images around the horizontal axis
    string outputFileName;      // The name of the file where to write
    bool showUndistorted;       // Show undistorted images after calibration
    string cameraType;			// The type of camera to use
    string input;               // The input ->
	Pattern calibrationPattern;
	int cameraID;
	string cameraString;
    vector<string> imageList;
    int atImageList;
    VideoCapture inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;

private:
    string patternToUse;


};

static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum { NORMAL = 0, UNDISTORTED = 1};
bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints );



static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();
    for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
}

static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr)
{

    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = 1.0;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, s.flag|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                             rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}



// Print camera parameters to the output file
static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr )
{	
	time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%A_%B_%d_%Y_%X", t2 );
    string filestring = s.outputFileName + ".xml";
    FileStorage fs(filestring, FileStorage::WRITE );
	fs << "input" << s.input;
    fs << "calibration_Time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nrOfFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_Width" << imageSize.width;
    fs << "image_Height" << imageSize.height;
    fs << "board_Width" << s.boardSize.width;
    fs << "board_Height" << s.boardSize.height;
    fs << "square_Size" << s.squareSize;

    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "FixAspectRatio" << s.aspectRatio;

    if( s.flag )
    {
        sprintf( buf, "flags: %s%s%s%s",
            s.flag & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
            s.flag & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
            s.flag & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
            s.flag & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );

    }

    fs << "flagValue" << s.flag;

    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;

    fs << "Avg_Reprojection_Error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "Extrinsic_Parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "Image_points" << imagePtMat;
    }
	cout << "Camera Parameters saved :)" << endl;
}

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,vector<vector<Point2f> > imagePoints )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s,imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
                             reprojErrs, totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
        << ". avg re projection error = "  << totalAvgErr ;

    if( ok )
        saveCameraParams( s, imageSize, cameraMatrix, distCoeffs, rvecs ,tvecs, reprojErrs,
                            imagePoints, totalAvgErr);
    return ok;
}

int main(int argc, char* argv[])
{
	help();
	Mat frame;
	Settings s;
	const string inputSettingsFile = argc > 1 ? argv[1] : "default2.xml";
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
	
	//VideoCapture cap("http://192.168.0.152/axis-cgi/mjpg/video.cgi?.mjpg");
	//if(!cap.isOpened())
	//	return 0;
	
	vector<vector<Point2f> > imagePoints;
	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	double totalAvgErr = 0;
	bool isCalibrated = false;
	Mat cameraMatrix, distCoeffs;
	Size imageSize;
	const Scalar RED(0,0,255), GREEN(0,255,0);
	const char ESC_KEY = 27;
	frame = s.nextImage();
	imageSize = frame.size();
	namedWindow("Camera View",0);
	int view = NORMAL;
	
	while(true)
	{	
		frame = s.nextImage();
		vector<Point2f> pointBuff;
		
		char key = (char)waitKey(20);
		
		if(key == 'g') //capture the frame
		{
			//check for chessboard
			bool found;
			found = findChessboardCorners(frame, s.boardSize, pointBuff,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
			//if found, add frame to calibration
			if(found)
			{
				Mat viewGray;
				cvtColor(frame, viewGray, CV_BGR2GRAY);
                cornerSubPix( viewGray, pointBuff, Size(11,11),
                    Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));	
                imagePoints.push_back(pointBuff);
                cout << "Frames: " << imagePoints.size() << endl;
            }
			
			//else do nothing
			
		}
		

		if(key == ESC_KEY)	//leave the program
		{
			if(!isCalibrated)
			{
				if(runCalibrationAndSave(s,imageSize, cameraMatrix, distCoeffs, imagePoints))
					return 0;
				else
					return -1;
			}
			else
				return 0;
		}
		
		
		if(isCalibrated &&  key == 'u') //show undistorted image
			s.showUndistorted = !s.showUndistorted;
		if(s.showUndistorted)
		{
			Mat temp = frame.clone();
			undistort(frame, temp, cameraMatrix, distCoeffs);
			imshow("Camera View", temp);
		}
		else //show the usual view
		{	
			imshow("Camera View",frame);
		}
		
		if(key == 's' && isCalibrated)
		{
			//save calibration
        	saveCameraParams( s, imageSize, cameraMatrix, distCoeffs, rvecs ,tvecs, reprojErrs,
                            	imagePoints, totalAvgErr);			
		}
		
		if(key == 'r')
		{	
			//reset calibration, ie remove all captured frames
			isCalibrated = false;
			totalAvgErr = 0;
			imagePoints.clear();
			rvecs.clear();
			tvecs.clear();
			reprojErrs.clear();
			cameraMatrix.release();
			distCoeffs.release();
		}
		
		if(key == 'c' && imagePoints.size() > 0)
		{
			bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, 
										rvecs, tvecs, reprojErrs, totalAvgErr);
			
			cout << (ok? "Calibration succeeded" : "Calibration failed") 
				<< ". avg re error = " << totalAvgErr << endl;
			if( ok )	
				isCalibrated = true;
		}
		
	}
	return 1;
}

