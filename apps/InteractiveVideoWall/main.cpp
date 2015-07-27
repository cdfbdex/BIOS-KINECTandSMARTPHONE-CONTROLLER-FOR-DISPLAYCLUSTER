/**
* Interactive Wall Application
*
* Author:	Center of Bioinformatics and Computational Biology
* Version:	1.0
*/

#pragma region LibraryDeclaration

// Constants definition
#define	MAX_POSSIBLE_SENSORS	10


// Definition of necessary libraries
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <cvblob.h>
#include <OpenNI.h>

#include <iostream>
#include <fstream>
#include <vector>

#include "VRPNAnalogServer_IVW.h"
#include "VRPNButtonServer_IVW.h"

#include "TUIOListener_IVW.h"
#include "TuioClient.h"
#include <math.h>

// Detect IP
#include <WinSock.h>



// Namespaces definition
using namespace cv;
using namespace cvb;
using namespace std;
using namespace openni;
using namespace TUIO;

#pragma endregion LibraryDeclaration


#pragma region VariableDeclaration

// Global variables definition
int morph_elem = 0;					// Type (or shape) of structuring element
int morph_size = 3;					// Same as kernel_size. Size of structuring element
int morph_operator = 1;				// Define morphological operation between structuring element and image

int threshold_value = 55;			// Lower threshold limit for binarization
int threshold_value2 = 110;			// Upper threshold limit for binarization

int MinSize = 1;					// Lower limit to filter blob by area
int MaxSize = 30;					// Upper limit to filter blob by area

int DistanceCroppingUnderneath = 0; // Number of pixels to filter in the image bottom

Mat finalDepthImage, finalDepthImage_gray;			// Final joined depth images
Mat depthImage[MAX_POSSIBLE_SENSORS];				// 3-Channel gotten image from depth sensor
Mat depthImage_gray[MAX_POSSIBLE_SENSORS];			// 1-Channel gotten image from depth sensor
Mat finalBinaryImage;								// Final processed image


IplImage* bwImage;					// Black-White 3-channel image 
IplImage* bwImage1C;				// Black-White 1-channel image for blob labeling
IplImage* labelImg;					// Image with labeled blobs

CvBlobs blobs;						// Structure to identify blobs in binary image
CvTracks tracks;					// Structure to extract characteristics of the blobs

// Variable to detect posture
int postureDetected;
// Variables to calculate zoom image
int valueCentroid = 0;					// Get depth value in centroid pixel
int minDistance, maxDistance, middleDistance, nearLim, farLim;	//Limits of depth distance

// Define resolution of displayed windows
int DepthImageWidth;
int DepthImageHeight;

int ScaleHeight = 1;				// Resize images decreasing height of the image in ScaleHeight factor
int detectedDevices;				// Number of devices that are connected

float P0 = 0.0;						// Centroid of biggest found Blob

char* main_window_name = "TUIO+Kinect: VRPN Controller (Exit: ESC)";

#pragma endregion VariableDeclaration


#pragma region GetColorAndDepthImage

/// <summary>
/// Get color image from sensor, and generate a matrix with its data.</summary>
/// <param name="color_frame"> Encapsulates all data related to a single frame read from a VideoStream</param>
/// <returns>Matrix with color image data</returns>
Mat getColorImage(VideoFrameRef& color_frame)
{
	// Determine wheter a VideoFrameRef contains actual valid data
	if (!color_frame.isValid())
	{
		return Mat();
	}

	//VideoMode: Stores resolution, framerate and pixel format of a frame
	VideoMode video_mode = color_frame.getVideoMode();

	//Create a matrix to save the image of size ResolutionY,ResolutionX
	Mat color_img = Mat(video_mode.getResolutionY(),
		video_mode.getResolutionX(),
		CV_8UC3,						// Type of matrix: 8bit-3Channel unsigned integers (0..255)
		(char*)color_frame.getData());	// Returns a void pointer to the data to be stored

	Mat ret_img;
	cv::cvtColor(color_img, ret_img, CV_RGB2BGR);				//Converts color_img from RGB2BGR and save it in ret_img

	return ret_img;
}


/// <summary>
/// Converts VideoFrameRef depth image from sensor, and converts it to a matrix for processing.</summary>
/// <param name="depth_image"> Data related to a single frame read from a VideoStream</param>
/// <returns>Matrix with depth image data </returns>
Mat getDepthImage(VideoFrameRef& depth_frame)
{
	// Determine wheter a VideoFrameRef contains actual valid data
	if (!depth_frame.isValid())
	{
		return Mat();
	}

	//VideoMode: Stores resolution, framerate and pixel format of a frame
	VideoMode video_mode = depth_frame.getVideoMode();

	//Create a matrix to save the image of size ResolutionY,ResolutionX
	Mat depth_img = Mat(video_mode.getResolutionY(),
		video_mode.getResolutionX(),
		CV_16U,							// Type of matrix: 16bit-1Channel unsigned integers ( 0..65535 )
		(char*)depth_frame.getData());	// Returns a void pointer to the data to be stored

	return depth_img.clone();									// clone() method creates a full copy of the array
}


/// <summary>
/// Convert image to lower resolution and scale it.</summary>
/// <param name="depth_image"> Matrix that contains the initial depth image from sensor</param>
/// <returns>Scaled and modified image </returns>
Mat getDepthDrawableImage(Mat depth_image)
{
	Mat drawable;
	depth_image.convertTo(drawable, CV_8UC1, 255.0 / 10000);		// Converts depth_image to Unsigned8bit-1Channel (UCHAR) with scale factor
	return drawable;
}


#pragma endregion GetColorAndDepthImage


#pragma region PostureDetection1

// Function to detect posture of person. If its posture is drag and drop returns 1, if is normal returns 2, otherwise returns 0.
int PostureDetection1(Mat Image)
{
	// Convert matrix data in finalBinaryImage into an IplImage using temporal variables
	//Mat  temp;
	//cvtColor(Image, temp, CV_GRAY2BGR);

	IplImage ipltemp = Image;
	IplImage *src = cvCreateImage(cvSize(Image.cols, Image.rows), 8, 3);
	cvCopy(&ipltemp, src);


/////////////////////////////////////////////////////////////////////////////////////////////
	//INICIANDO VARIABLES
	int c = 0;
	int resultado = 0;
	CvSeq* a = 0;
	
	CvSize sz = cvGetSize(src);

	IplImage* hsv_image = cvCreateImage(sz, 8, 3);
	IplImage* hsv_image_gray = cvCreateImage(sz, 8, 1);
	IplImage* hsv_mask = cvCreateImage(sz, 8, 1);
	IplImage* hsv_edge = cvCreateImage(sz, 8, 1);

	//CvScalar  hsv_min = cvScalar(0, 30, 80, 0);
	//CvScalar  hsv_max = cvScalar(20, 150, 255, 0);
	CvScalar  hsv_min = cvScalar(100, 100, 100, 0);
	CvScalar  hsv_max = cvScalar(255, 255, 255, 0);


	CvMemStorage* storage = cvCreateMemStorage(0);
	CvMemStorage* areastorage = cvCreateMemStorage(0);
	CvMemStorage* minStorage = cvCreateMemStorage(0);
	CvMemStorage* dftStorage = cvCreateMemStorage(0);

	CvSeq* contours = NULL;
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
	//ANALIZANDO IMAGEN
	IplImage* bg = cvCreateImage(sz, 8, 3);
	cvRectangle(bg, cvPoint(0, 0), cvPoint(bg->width, bg->height), CV_RGB(255, 255, 255), -1, 8, 0);
	bg->origin = 1;

	for (int b = 0; b< int(bg->width / 10); b++)
	{
		cvLine(bg, cvPoint(b * 20, 0), cvPoint(b * 20, bg->height), CV_RGB(200, 200, 200), 1, 8, 0);
		cvLine(bg, cvPoint(0, b * 20), cvPoint(bg->width, b * 20), CV_RGB(200, 200, 200), 1, 8, 0);
	}


	//cvCvtColor(src, hsv_image, CV_BGR2HSV);
	cvCvtColor(src, hsv_image_gray, CV_BGR2GRAY);
	cvCvtColor(hsv_image_gray, hsv_image, CV_GRAY2BGR);

	cvInRangeS(hsv_image, hsv_min, hsv_max, hsv_mask);
	cvSmooth(hsv_mask, hsv_mask, CV_MEDIAN, 27, 0, 0, 0);
	cvCanny(hsv_mask, hsv_edge, 1, 3, 5);

	cvFindContours(hsv_mask, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));
	CvSeq* contours2 = NULL;
	double result = 0, result2 = 0;
	while (contours)
	{
		result = fabs(cvContourArea(contours, CV_WHOLE_SEQ));
		if (result > result2) 
		{
			result2 = result; contours2 = contours; 
		}
		contours = contours->h_next;
	}
	if (contours2)
	{
		CvRect rect = cvBoundingRect(contours2, 0);
		cvRectangle(bg, cvPoint(rect.x, rect.y + rect.height), cvPoint(rect.x + rect.width, rect.y), CV_RGB(200, 0, 200), 1, 8, 0);
		int checkcxt = cvCheckContourConvexity(contours2);
		CvSeq* hull = cvConvexHull2(contours2, 0, CV_CLOCKWISE, 0);
		CvSeq* defect = cvConvexityDefects(contours2, hull, dftStorage);

		if (defect->total <= 20) 
		{
			//cout << " Drag and Drop ";
			resultado = 1;
		}
		//else if( defect->total >=10 && defect->total <40 ) {cout << " Open Palm " << endl;}
		else{
			//cout << " NORMAL ";
			resultado = 2;
		}
		//cout << " - Defects: " << defect->total << endl;

		CvBox2D box = cvMinAreaRect2(contours2, minStorage);
		cvCircle(bg, cvPoint(box.center.x, box.center.y), 3, CV_RGB(200, 0, 200), 2, 8, 0);
		cvEllipse(bg, cvPoint(box.center.x, box.center.y), cvSize(box.size.height / 2, box.size.width / 2), box.angle, 0, 360, CV_RGB(220, 0, 220), 1, 8, 0);
	}

	cvDrawContours(bg, contours2, CV_RGB(0, 200, 0), CV_RGB(0, 100, 0), 1, 1, 8, cvPoint(0, 0));
	
	//cvShowImage("src", hsv_image_gray);
	//cvNamedWindow("bg", 0);
	//cvShowImage("bg", bg);
	
	cvReleaseImage(&hsv_image);
	cvReleaseImage(&hsv_image_gray);
	cvReleaseImage(&hsv_mask);
	cvReleaseImage(&hsv_edge);
	cvReleaseImage(&bg);
	cvReleaseImage(&src);
	
	cvReleaseMemStorage(&storage);
	cvReleaseMemStorage(&areastorage);
	cvReleaseMemStorage(&minStorage);
	cvReleaseMemStorage(&dftStorage);
	
	return resultado;
/////////////////////////////////////////////////////////////////////////////////////////////

}

#pragma endregion PostureDetection1


#pragma region TrackObjects
/// <summary>
/// Process the image obtained from depth sensor and prepare the window to show the tracked blobs.</summary>
void TrackObjects(int, void*)
{
	// Threshold finalDepthImage_gray with threshold values and save it in finalBinaryImage.
	// If is in range finalBinaryImage is set to 255, 0 otherwise
	inRange(finalDepthImage_gray, Scalar(threshold_value), Scalar(threshold_value2), finalBinaryImage);

	// Create and structuring element for morphological operations
	// morph_elem: 0=MORPH_RECT; 1=MORPH_CROSS; 2=MORPH_ELLIPSE
	// Size(cols,rows): Size of structuring element - Point(x,y): Anchor position within the element
	Mat element = getStructuringElement(morph_elem, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));

	// Apply the specified morphological operation to finalBinaryImage matrix, and replace it
	// operation: 0=MORPH_ERODE; 1=MORPH_DILATE; 2=MORPH_OPEN; 3=MORPH_CLOSE; 4=MORPH_GRADIENT; 5=MORPH_TOPHAT; 6=MORPH_BLACKHAT;
	int operation = morph_operator;
	morphologyEx(finalBinaryImage, finalBinaryImage, operation, element);

	//Converts finalBinaryImage to 3-Channel image
	cv::cvtColor(finalBinaryImage, finalBinaryImage, CV_GRAY2BGR);

	// Convert matrix data in finalBinaryImage into an IplImage using temporal variables
	IplImage ipltemp = finalBinaryImage;

	cvCopy(&ipltemp, bwImage);

	// Apply fixed-level threshold to each array element in source image of type IplImage
	// Constructor: cvThreshold(InputArray src, OutputArray finalBinaryImage, double thresh, double maxval, int type)
	// type: 0: Binary - 1: Binary Inverted - 2: Threshold Truncated - 3: Threshold to Zero - 4: Threshold to Zero Inverted
	cvThreshold(bwImage, bwImage, 100, 255, 0);


	// Divide multichannel array into several single-channel arrays in IplImages [replaces cvtColor()]
	cvSplit(bwImage, bwImage1C, NULL, NULL, NULL);

	// Get the blobs from binarized image bwImage1C, and save it in labelImg
	unsigned int result = cvLabel(bwImage1C, labelImg, blobs);

	// Filter blobs whose area is not in the range defined in the trackbar. The area computed as the percentage of the container window
	cvFilterByArea(blobs, int((float(MinSize) / 100.0)*(DepthImageWidth*detectedDevices * DepthImageHeight / ScaleHeight)), int((float(MaxSize) / 100.0)*(DepthImageWidth*detectedDevices * DepthImageHeight/ScaleHeight)));

	// Updates list of tracks based on current blobs, converting the detected blobs into structures with
	// info related to lifetime and position. Tracks assure that one blob has the same label while it is
	// existing in different frames
	// Constructor: cvUpdateTracks(CvBlobs const &b, CvTracks &t, const double thDistance, const unsigned int thInactive, const unsigned int thActive=0)
	cvUpdateTracks(blobs, tracks, 5., 10);


	// Draws or prints information about blobs contained in third parameter, with the next properties:
	/* Render mode:
	CV_BLOB_RENDER_COLOR            ///< Render each blob with a different color.
	CV_BLOB_RENDER_CENTROID         ///< Render centroid.
	CV_BLOB_RENDER_BOUNDING_BOX     ///< Render bounding box.
	CV_BLOB_RENDER_ANGLE            ///< Render angle.
	*/
	cvRenderBlobs(labelImg, blobs, bwImage, bwImage, CV_BLOB_RENDER_CENTROID | CV_BLOB_RENDER_BOUNDING_BOX);


	// Prints track information of each blob in third argument, with the next properties:
	/* Render mode:
	CV_TRACK_RENDER_ID					///< Print the ID of each track in the image.
	CV_TRACK_RENDER_BOUNDING_BOX		///< Draw bounding box of each track in the image.
	CV_TRACK_RENDER_TO_LOG				///< Print track info to log out.
	CV_TRACK_RENDER_TO_STD				///< Print track info to log out.
	*/
	cvRenderTracks(tracks, bwImage, bwImage, CV_TRACK_RENDER_ID | CV_TRACK_RENDER_BOUNDING_BOX);

	float P1 = 0.0;
	// Find the blob whose area is the biggest and get its centroid's position
	if (blobs.size() > 0)
	{
		CvLabel largestBlobLabel = cvLargestBlob(blobs);
		// Look in the blob's list
		for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it)
		{
			CvLabel comparingBlobLabel = (*it).second->label;
			int sideblob = abs(int((*it).second->maxx - (*it).second->minx)) < abs(int((*it).second->maxy - (*it).second->miny)) ? abs(int((*it).second->maxx - (*it).second->minx)) : abs(int((*it).second->maxy - (*it).second->miny));
			sideblob = int(0.01*float(sideblob));
			sideblob = sideblob < 2 ? 2 : sideblob;
			
			if (largestBlobLabel == comparingBlobLabel)
			{
				// Get centroid coordinates of the blob and use it to select the frame of the showing video
				CvPoint2D64f XY = (*it).second->centroid;
				P0 = (float)XY.x;
				P1 = (float)XY.y;
				//cout << "Largest Blob " << largestBlobLabel << " = ( " << XY.x << " , " << XY.y << " )" << endl;	/**/

				// Set a 20x20 square roi around centroid.
				Rect roiCentroid = Rect(int(P0) - sideblob, int(P1) - sideblob, 2 * sideblob, 2 * sideblob);
				Mat mask(finalDepthImage_gray.size(), CV_8UC1, Scalar::all(0));
				mask(roiCentroid).setTo(Scalar::all(255));

				// Get depth value based on the roi
				double minColor, maxColor;
				minMaxIdx(finalDepthImage_gray, &minColor, &maxColor, NULL, NULL, mask);
				valueCentroid = maxColor;

				//cout << "min: " << minColor << " - max: " << maxColor << endl;
			}
		}	
	}
	else
	{
		valueCentroid = 0;
		//cout << "No Blob found" << endl;	/**/
	}
	
	//cout << endl;
	//cout << "Centroid Depth: " << valueCentroid << endl; /**/


	// Clear all blob structures of the frame
	cvReleaseBlobs(blobs);

	// Copy matrix of first argument to finalBinaryImage
	finalBinaryImage = Mat(bwImage, true);


	postureDetected = PostureDetection1(finalBinaryImage);

	//imshow("Test", finalBinaryImage);	/**/


	// Draw a rectangle with the next constructor: rectangle(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
	// img: Image to draw the rectangle - pt1: Origin vertex - pt2: Vertex opposite to pt1
	int const rectangleThickness = 3;
	rectangle(finalDepthImage, Point(0, 0), Point(int(DepthImageWidth*detectedDevices - rectangleThickness), int(DepthImageHeight/ScaleHeight - rectangleThickness)), Scalar(0, 0, 255)/*BRG*/, rectangleThickness);
	rectangle(finalBinaryImage, Point(0, 0), Point(int(DepthImageWidth*detectedDevices - rectangleThickness),int(DepthImageHeight/ScaleHeight - rectangleThickness)), Scalar(0, 255, 0), rectangleThickness);

	// Design of video containers
	Size sizeLeft = finalDepthImage.size();													// Size of acquired gray-color image
	Size sizeRight = finalBinaryImage.size();												// Size of processed binary-color image

	Mat imageContainer(sizeLeft.height, sizeLeft.width + sizeRight.width, CV_8UC3);			// Create a matrix that contains both videos

	Mat leftImage(imageContainer, Rect(0, 0, sizeLeft.width, sizeLeft.height));				// Size of left container
	finalDepthImage.copyTo(leftImage);														// Assign image to left container

	Mat rightImage(imageContainer, Rect(sizeLeft.width, 0, sizeRight.width, sizeRight.height));			// Size of right container
	finalBinaryImage.copyTo(rightImage);																// Assing image to right container

	for (int i = 0; i < detectedDevices; i++)
	{
		// Write label in each image
		stringstream textT;
		textT << i+1;
		string text = textT.str();
		putText(imageContainer, text, Point(10+(i*DepthImageWidth),int(0.1*ScaleHeight*sizeLeft.height)), FONT_HERSHEY_SIMPLEX, 2, Scalar(0,0,255),2);
	}

	imshow(main_window_name, imageContainer);
}

#pragma endregion TrackObjects


#pragma region DetectIP
// Detect all IP interfaces in the computer
vector<char*> detectIPAddresses(void)
{
	WORD wVersionRequested;
	WSADATA wsaData;
	char name[255];
	PHOSTENT hostinfo;
	wVersionRequested = MAKEWORD(1, 1);
	char* ip;

	vector<char*> detectedIP;

	//if (WSAStartup(wVersionRequested, &wsaData) == 0)
	//if (gethostname(name, sizeof(name)) == 0)
	//{
	//	printf("Host name: %s\n", name);

	//	if ((hostinfo = gethostbyname(name)) != NULL)
	//	{
	//		int nCount = 0;
	//		while (hostinfo->h_addr_list[nCount])
	//		{
	//			ip = inet_ntoa(*(struct in_addr *)hostinfo->h_addr_list[nCount]);

	//			detectedIP.push_back(ip);

	//			printf("IP #%d: %s\n", ++nCount, ip);
	//		}
	//	}
	//}

	if (WSAStartup(wVersionRequested, &wsaData) == 0)
	if (gethostname(name, sizeof(name)) == 0)
	{
		if ((hostinfo = gethostbyname(name)) != NULL)
		{
			ip = inet_ntoa(*(struct in_addr *)hostinfo->h_addr_list[0]);
			detectedIP.push_back(ip);
		}
	}
	return detectedIP;
}

#pragma endregion DetectIP


// Main procedure
int main(int argc, char** argv)
{

#pragma region InitializeDrivers
	Device device[MAX_POSSIBLE_SENSORS];										// Interface to a sensor devices connected
	VideoStream** streams = new openni::VideoStream*[MAX_POSSIBLE_SENSORS];		// Save streams from depth sensor
	VideoStream color_stream[MAX_POSSIBLE_SENSORS];								// Save color stream // It is not used
	VideoStream depth_stream[MAX_POSSIBLE_SENSORS];								// Save depth stream
	VideoMode videoMode[MAX_POSSIBLE_SENSORS];									// Save video mode for sensor configuration

	const char* deviceURI[MAX_POSSIBLE_SENSORS];			// Specify Uniform Resource Identifier of each detected device


	// Check initialization of drivers
	Status rc;												// Variable to get status messages
	rc = OpenNI::initialize();								// Initialize drivers and make API aware of connected devices
	if (rc != STATUS_OK)
	{
		cout << "Driver initialization failed:" << OpenNI::getExtendedError() << endl;
		system("pause");
		return EXIT_FAILURE;
	}

	// Detect how many sensor are connected
	Array<DeviceInfo> deviceInfoList;
	OpenNI::enumerateDevices(&deviceInfoList);

	// List device that are connected
	detectedDevices = deviceInfoList.getSize();

	// Check number of detected devices
	if (detectedDevices > MAX_POSSIBLE_SENSORS)
	{
		cout << "Cannot detect more than " << MAX_POSSIBLE_SENSORS << " devices." << endl;
		system("pause");
		OpenNI::shutdown();
		return EXIT_FAILURE;
	}
	if (detectedDevices < 1)
	{
		cout << "Not connected devices." << endl;
		system("pause");
		OpenNI::shutdown();
		return EXIT_FAILURE;
	}
	else
	{
		cout << "Number of detected devices: " << detectedDevices << endl;
	}


	//Detect URI of connected devices
	for (int i = 0; i < detectedDevices; i++)
	{
		deviceURI[i] = deviceInfoList[i].getUri();
		//cout << deviceURI[i] << endl;
	}

#pragma endregion InitializeDrivers

#pragma region KinectOrder

	//// Change order of devices.
	// File reading for kinect order
	ifstream inFileKinect;

	inFileKinect.open("KinectOrder.txt", ios::in);

	int kinect1, kinect2;
	if (inFileKinect.is_open())
	{
		cout << "---Kinect configuration---" << endl;
		while (!inFileKinect.eof())
		{
			inFileKinect >> kinect1 >> kinect2;

			if (kinect1 == kinect2)
			{
				continue;
			}
			else if (kinect1<1 || kinect1>detectedDevices ||
				kinect2<1 || kinect2>detectedDevices)
			{
				cout << "Invalid data in settings file. ";
				cout << "Input values between 1 and " << detectedDevices << "." << endl;
				system("pause");
				OpenNI::shutdown();
				return EXIT_FAILURE;
			}
			else
			{
				const char* temp = deviceURI[kinect2 - 1];
				deviceURI[kinect2 - 1] = deviceURI[kinect1 - 1];
				deviceURI[kinect1 - 1] = temp;
				cout << "Kinect's image " << kinect1 << " and " << kinect2 << " were swapped correctly." << endl;
			}
			kinect1 = 0;
			kinect2 = 0;
		}
		inFileKinect.close();
	}

#pragma endregion KinectOrder

#pragma region Errors
	// Initialize and set up each one of the connected sensors
	for (int i = 0; i < detectedDevices; i++)
	{
		rc = device[i].open(deviceURI[i]);							// Connect to physical hardware device
		if (rc != STATUS_OK)
		{
			cout << "SimpleViewer device " << i + 1 << ": Device open failed:" << OpenNI::getExtendedError() << endl;
			system("pause");
			OpenNI::shutdown();										// Shutdown drivers and clean up properly
			return EXIT_FAILURE;
		}


		// Create VideoStream with depth sensor
		rc = depth_stream[i].create(device[i], SENSOR_DEPTH);			// Specify device and sensor type: (SENSOR_IR, SENSOR_COLOR, SENSOR_DEPTH)
		if (rc == STATUS_OK)
		{
			rc = depth_stream[i].start();
			// Check depth sensor video stream started properly
			if (rc != STATUS_OK)
			{
				cout << "SimpleViewer device " << i + 1 << ": Couldn't start depth stream:" << OpenNI::getExtendedError() << endl;
				depth_stream[i].destroy();							// Free memory used by the VideoStream
			}
		}
		else
		{
			cout << "SimpleViewer device " << i + 1 << ": Couldn't find depth stream:" << OpenNI::getExtendedError() << endl;
		}

		// Create VideoStream with color sensor
		rc = color_stream[i].create(device[i], SENSOR_COLOR);		// Specify device and sensor type: (SENSOR_IR, SENSOR_COLOR, SENSOR_DEPTH)
		if (rc == STATUS_OK)
		{
			rc = color_stream[i].start();
			// Check color sensor video stream started properly
			if (rc != STATUS_OK)
			{
				cout << "SimpleViewer device " << i + 1 << ": Couldn't start color stream:" << OpenNI::getExtendedError() << endl;
				color_stream[i].destroy();							// Free memory used by the VideoStream
			}
		}
		else
		{
			cout << "SimpleViewer device " << i + 1 << ": Couldn't find color stream.\n" << OpenNI::getExtendedError() << endl;
		}

		// Check if any of the streams is invalid
		if (!depth_stream[i].isValid() || !color_stream[i].isValid())
		{
			cout << "SimpleViewer device " << i + 1 << ": No valid streams." << endl;
			system("pause");
			OpenNI::shutdown();									// Shutdown drivers and clean up properly
			return EXIT_FAILURE;
		}
		else
		{
			cout << "Device " << i + 1 << "'s configuration was OK." << endl;
		}

		// Variable to allocate video streams
		streams[i] = &depth_stream[i];
		//streams[i].push_back(&color_stream[i]);
		videoMode[i] = depth_stream[i].getVideoMode();

	}
	cout << endl;

	// Verify that resolution's image match
	for (int i = 1; i < detectedDevices; i++)
	{
		if (videoMode[i].getResolutionX() != videoMode[i - 1].getResolutionX() ||
			videoMode[i].getResolutionY() != videoMode[i - 1].getResolutionY())
		{
			cout << "Streams of all sensors must have the same resolution." << endl;
			system("pause");
			return EXIT_FAILURE;
		}
	}

#pragma endregion Errors

	// Set width and height of each depth image
	DepthImageWidth = videoMode[0].getResolutionX();
	DepthImageHeight = videoMode[0].getResolutionY();

#pragma region WindowCreation
	/////////////////////// WINDOW CONFIGURATION

	// Constants defined for trackbar creation
	int const max_pixel_value = 255;	// Max value in a pixel
	int const max_operator = 6;			// Max value for operation between SE and image
	int const max_elem = 2;				// Max value for shape of SE
	int const max_kernel_size = 20;		// Max value for size of SE
	int const max_value_size = 100;		// Max value for filtering size of blobs in the image
	int const max_HeightCropping = DepthImageHeight / ScaleHeight;		// Max value for filter floor image
	int const max_Sensibility = (DepthImageWidth*detectedDevices) / 10;	// Max sensibility value in Pixels.

	// Definition of trackbar labels
	char* trackbar_MinDistValue = "Min. Dist:";			// Labels for depth image binarization thresholds
	char* trackbar_MaxDistValue = "Max. Dist:";
	char* trackbar_MinBlobSize = "Min. Area:";			// Labels for filtering blobs by area
	char* trackbar_MaxBlobSize = "Max. Area:";
	char* trackbar_Operator = "Operator:";				// Label for morphological operation
	char* trackbar_ShapeSE = "SE Shape:";				// Label for SE shape -- 0:Rect - 1:Cross - 2:Ellipse
	char* trackbar_SizeSE = "SE Size:";					// Label for SE size
	char* trackbar_HeightCropping = "Floor clr:";		// Label for floor filter
	char* trackbar_Sensibility = "X Sensib.:";			// Label for sensibility

	// Window Creation
	namedWindow(main_window_name, CV_WINDOW_NORMAL);
	startWindowThread();

	// File reading for settings values
	loadWindowParameters(main_window_name);

	ifstream inFile;
	inFile.open("Settings.txt", ios::in);
	if (inFile.is_open())
	{
		inFile >> threshold_value >> threshold_value2 >> MinSize >> MaxSize >> morph_operator >>
			morph_elem >> morph_size >> DistanceCroppingUnderneath;

		inFile.close();
	}

	// Trackbar Creation
	// Trackbar to modify Binarization Thresholds
	createTrackbar(trackbar_MinDistValue, main_window_name, &threshold_value, max_pixel_value, TrackObjects);
	createTrackbar(trackbar_MaxDistValue, main_window_name, &threshold_value2, max_pixel_value, TrackObjects);
	// Trackbar to choose size limits in the cvFilterbyArea function
	createTrackbar(trackbar_MinBlobSize, main_window_name, &MinSize, max_value_size, TrackObjects);
	createTrackbar(trackbar_MaxBlobSize, main_window_name, &MaxSize, max_value_size, TrackObjects);
	// Trackbar to select Morphology operation
	createTrackbar(trackbar_Operator, main_window_name, &morph_operator, max_operator, TrackObjects);
	// Trackbar to select kernel type
	createTrackbar(trackbar_ShapeSE, main_window_name, &morph_elem, max_elem, TrackObjects);
	// Trackbar to choose kernel size
	createTrackbar(trackbar_SizeSE, main_window_name, &morph_size, max_kernel_size, TrackObjects);
	// Trackbar to filter floor from image
	createTrackbar(trackbar_HeightCropping, main_window_name, &DistanceCroppingUnderneath, max_HeightCropping, TrackObjects);

#pragma endregion WindowCreation

	// Images used in the TrackObjects function
	bwImage = cvCreateImage(cvSize(DepthImageWidth*detectedDevices, DepthImageHeight / ScaleHeight), IPL_DEPTH_8U, 3);
	labelImg = cvCreateImage(cvSize(DepthImageWidth*detectedDevices, DepthImageHeight / ScaleHeight), IPL_DEPTH_LABEL, 1);
	bwImage1C = cvCreateImage(cvSize(DepthImageWidth*detectedDevices, DepthImageHeight / ScaleHeight), IPL_DEPTH_8U, 1);

	// Variables to allocate frames of video streams
	VideoFrameRef depth_frame[MAX_POSSIBLE_SENSORS];

	// Filter X Position and DragNDrop Event
	vector<int> lastPositionEvent(20);			//Vector to allocate last read events of drag and drop to stablish raising edge signal.
	for (int i = 0; i < 20; i++)
	{
		lastPositionEvent.push_back(0);
	}
	float DragAndDrop = 0.0;					//Signal to send. 1 when drag, 0 when drop

#pragma region InitTUIO

	///Initialize TUIO Client
	int portTUIO = 3333;
	TUIOListener_IVW Listener;
	TUIO::TuioClient client(portTUIO);
	client.addTuioListener(&Listener);
	client.connect(false);
	cout << "Created TUIO Listener" << endl;

#pragma endregion InitTUIO


#pragma region InitVRPN

	// Initialize VRPN Server
	int portVRPN = 3883;
	vrpn_Connection_IP* m_Connection = new vrpn_Connection_IP();
	//vrpn_Connection* m_Connection = vrpn_create_server_connection(portVRPN);

	// Creating the servers
	int numChannelsVRPNAnalog = 9;
	VRPNAnalogServer_IVW* serverAnalog = new VRPNAnalogServer_IVW(m_Connection, numChannelsVRPNAnalog);
	const char* VRPNname = serverAnalog->GetName();
	//VRPNButtonServer_IVW* serverButton = new VRPNButtonServer_IVW(m_Connection);
	cout << "Created VRPN server." << endl;

	// Vector to send analog data: (X,Y) position from TUIO application
	vector<float> sentDataAnalog;
	for (int i = 0; i < numChannelsVRPNAnalog; i++)
	{
		sentDataAnalog.push_back(0);
	}

#pragma endregion InitVRPN


#pragma region StatusBarMessage

	// Write hostname and IP addresses
	vector<char*> IPs = detectIPAddresses();

	char statusBarText[200] = "Connect VRPN to:";
	for (int i = 0; i < IPs.size(); i++)
	{
		char* temp = statusBarText;
		sprintf(statusBarText, "%s - %s@%s", temp, VRPNname, IPs[i]);
	}

#pragma endregion StatusBarMessage


#pragma region MainLoop

	// Infinite loop for window updating and image processing
	while (true)
	{
		// Show message in status bar.
		displayStatusBar(main_window_name, statusBarText);
		
		// Loop for reading the images from all detected sensors
		bool captured_allkinects = false;
		bool kinectReaded[MAX_POSSIBLE_SENSORS] = { false };

#pragma region ReadAllKinects

		while (captured_allkinects == false)
		{
			int changedIndex = -1;

			// Wait for a new frame from any of the streams provided. And save the status message in rc
			// Reads all the streams in VideoStream variable and assign to changedIndex the index of the first stream that has a new frame available
			rc = OpenNI::waitForAnyStream(streams, detectedDevices, &changedIndex);

			// Check if has obtained the frame properly
			if (rc != STATUS_OK)
			{
				cout << ("Wait failed\n");
				system("pause");
				return EXIT_FAILURE;
			}

			// Depending on the index, read the frame from different sensor
			if (changedIndex >= 0 && changedIndex < detectedDevices)
			{
				depth_stream[changedIndex].readFrame(&depth_frame[changedIndex]);
				kinectReaded[changedIndex] = true;
			}

			// Calculate if all detected sensor have been readed
			captured_allkinects = true;
			for (int i = 0; i < detectedDevices; i++)
			{
				captured_allkinects = captured_allkinects & kinectReaded[i];
			}
		}

#pragma endregion ReadAllKinects

#pragma region JoinImages

		Mat depthImage_resized[MAX_POSSIBLE_SENSORS];

		for (int i = 0; i < detectedDevices; i++)
		{
			if (depth_frame[i].isValid())
			{
				// Get and process depth image
				depthImage_gray[i] = getDepthImage(depth_frame[i]);								//Get gray image from depth sensor: Resolution 16 bits
				depthImage_gray[i] = getDepthDrawableImage(depthImage_gray[i]);					//Get gray image from depth sensor: Resolution 8 bits
				cv::cvtColor(depthImage_gray[i], depthImage[i], CV_GRAY2BGR);						//Create 3-Channel image from gray image
				//imshow("Depth Image", depthImage[i]);
			}
			resize(depthImage_gray[i], depthImage_resized[i], Size(DepthImageWidth, DepthImageHeight / ScaleHeight));		// Draw and image with the WIDTH*detectedDevices and reduce its height
			//resize(depthImage[i], depthImage_resized[i], Size(DepthImageWidth, DepthImageHeight / ScaleHeight));		// Draw and image with the WIDTH*detectedDevices and reduce its height
		}


		// Join images from different sensor into one matrix
		Mat ImTotal(depthImage_resized[0].rows, depthImage_resized[0].cols*detectedDevices, CV_8UC1);
		for (int i = 0; i < detectedDevices; i++)
		{
			// Join images
			Mat joinImages(ImTotal, Rect(i*depthImage_resized[0].cols, 0, depthImage_resized[0].cols, depthImage_resized[0].rows));
			depthImage_resized[i].copyTo(joinImages);
			//imshow("Depth Image", joinImages);
		}


		// Crop floor area
		Mat mask(ImTotal, Rect(0, (DepthImageHeight / ScaleHeight) - DistanceCroppingUnderneath, ImTotal.cols, DistanceCroppingUnderneath));				// Size of floor filter
		if (ImTotal.type() == CV_8UC1)
		{
			mask.setTo(Scalar(255));
		}
		else
		{
			mask.setTo(Scalar(255, 255, 255));
		}

		// Convert images in necessary color scales
		cv::cvtColor(ImTotal, finalDepthImage, CV_GRAY2BGR);
		cv::cvtColor(finalDepthImage, finalDepthImage_gray, CV_RGB2GRAY);

#pragma endregion JoinImages

		// Call the function to initialize
		TrackObjects(0, 0);

#pragma region DragAndDrop
		////// DETECT DURATION OF DRAG AND DROP AND SEND SIGNAL
		lastPositionEvent.erase(lastPositionEvent.begin());
		lastPositionEvent.push_back(int(postureDetected));

		//// Test drag and drop event
		//for (int i = 0; i < lastPositionEvent.size(); i++)
		//{
		//	cout << lastPositionEvent[i];
		//}
		//cout << endl;

		bool allOnes = false;
		for (int i = 0; i < lastPositionEvent.size(); i++)
		{
			if (lastPositionEvent[i] != 1)
				break;
			if (i == lastPositionEvent.size() - 1)
			{
				allOnes = true;
				// If detect event change, clear vector
				for (unsigned int i = 1; i < lastPositionEvent.size(); i++)
				{
					lastPositionEvent[i] = 0;
				}
			}
		}

		if (allOnes && DragAndDrop == 0.0)
		{
			DragAndDrop = 1.0;
			//cout << "Drag" << endl;
			displayOverlay(main_window_name, "DRAG");
		}
		else if (allOnes && DragAndDrop == 1.0)
		{
			DragAndDrop = 0.0;
			//cout << "Drop" << endl;
			displayOverlay(main_window_name, "DROP", 2000);
		}

#pragma endregion DragAndDrop

#pragma region ZoomEvents
		////// ZOOM EVENTS
		/*
		Calculate zones where zoom out, zoom in and noZoom are detected.
		*/
		minDistance = threshold_value;						// Minimum value distance
		maxDistance = threshold_value2;						// Maximum value distance
		middleDistance = (minDistance + maxDistance) / 2;	// Middle distance
		nearLim = (minDistance + middleDistance) / 2;		// Lower limit to no-zoom area
		farLim = (maxDistance + middleDistance) / 2;		// Upper limit to no-zoom area

		float zoomIn = 0.0, zoomOut = 0.0;

		// Stablish three interaction areas to send commands based on depth distance
		// Zoom in area
		if (valueCentroid > minDistance && valueCentroid < nearLim)
		{
			//cout << "Zoom IN --- ";
			zoomIn = 1.0;
			zoomOut = 0.0;

		}
		// Zoom out area
		else if (valueCentroid > farLim && valueCentroid < maxDistance)
		{
			//cout << "Zoom OUT--- ";
			zoomIn = 0.0;
			zoomOut = 1.0;
		}

#pragma endregion ZoomEvents

		// Test depth detection
		//cout << "Value Centroid: " << valueCentroid << endl;

#pragma region SendViaVRPN
		///// VRPN
		// Captura data from application to VRPN
		sentDataAnalog[0] = 20000.0*(float(Listener.GetSignSpeedY()));		// Y movement in TUIO
		sentDataAnalog[1] = 20000.0*(float(Listener.GetSignSpeedX()));		// X movement in TUIO
		sentDataAnalog[2] = 0.0;											// Y movement for pan content
		sentDataAnalog[3] = 0.0;											// Y movement for pan content
		sentDataAnalog[4] = DragAndDrop;									// Drag and drop event
		sentDataAnalog[5] = 0.0;											// Zoom out content
		sentDataAnalog[6] = 0.0;											// Zoom in content
		sentDataAnalog[7] = zoomOut;										// Zoom out window
		sentDataAnalog[8] = zoomIn;											// Zoom in window

		serverAnalog->CaptureData(sentDataAnalog);

		// Send data via VRPN
		serverAnalog->mainloop();
		m_Connection->mainloop();

#pragma endregion SendViaVRPN

		// Exit of the program
		int key = waitKey(1);
		if (key == 27)
		{
			//exit = true;						// Enable exit flag to close application
			cout << "Press Ctrl+C to exit..." << endl;
			break;
		}
	}

#pragma endregion MainLoop


#pragma region CreateFiles

	// File writing for settings values
	saveWindowParameters(main_window_name);

	ofstream outFile;
	outFile.open("Settings.txt",ios::out);
	if (outFile.is_open())
	{
		outFile << (int)threshold_value << '\n' << (int)threshold_value2 << '\n'
			<< (int)MinSize << '\n' << (int)MaxSize << '\n' << (int)morph_operator << '\n'
			<< (int)morph_elem << '\n' << (int)morph_size << '\n' << (int)DistanceCroppingUnderneath;

		outFile.close();
	}

	// Create KinectOrder file if it is not created
	inFileKinect.open("KinectOrder.txt", ios::in);
	if (!inFileKinect.is_open())
	{
		ofstream outFileKinect;
		outFileKinect.open("KinectOrder.txt", ios::out);
		outFileKinect << "0 0\n";
		outFileKinect.close();
	}

	


#pragma endregion CreateFiles


#pragma region FreeMemory
	destroyAllWindows();				// Close cvWindows

	// Free memory of saved images
	cvReleaseImage(&bwImage);
	cvReleaseImage(&labelImg);
	cvReleaseImage(&bwImage1C);

	color_stream->stop();
	depth_stream->stop();

	color_stream->destroy();
	depth_stream->destroy();

	delete[] streams;
	
	OpenNI::shutdown();					// Finish using openni

#pragma endregion FreeMemory

	return EXIT_SUCCESS;
}
