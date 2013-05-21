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
}
