#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <math.h>

//#include "/usr/local/include/opencv/cv.h"
//#include "/usr/local/include/opencv/highgui.h"
#include "/usr/local/include/opencv2/opencv.hpp"

#include <sstream>

#define xPixels 640 //Number of x pixels
#define yMin 200 //Lower y limit
#define yMax 280 //Upper y limit

#define camLaserDist 12.7 //Distance from camera to laser in cm

double getDistance(CvCapture* capture);
double findLaser(IplImage* frame);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance");

  ros::NodeHandle n;

  ros::Publisher distance_pub = n.advertise<std_msgs::Float64>("distance", 1000);
  
  CvCapture* capture = cvCreateCameraCapture(0); //Capture from the webcam
  assert(capture != NULL); //Make sure that webcam is connected

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::Float64 msg;
    
    msg.data = getDistance(capture);

    ROS_INFO("%f", msg.data);

    distance_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

double getDistance(CvCapture* capture){

    double laserx = 0, pixdist, angle; //The x pixel coordinate of the laser dot, the x distance from the dot to the center, the angle createdlibv4l2: error setting pixformat: Device or resource busy
    
    IplImage * frame;

    frame = cvQueryFrame(capture); //Query webcam
    assert(frame); //Make sure an image is there

    IplImage* r = cvCreateImage(cvSize(frame->width,frame->height),frame->depth,1); //Red
    IplImage* hsvframe = cvCreateImage(cvSize(frame->width,frame->height),frame->depth,3); //HSV
    IplImage* h = cvCreateImage(cvSize(frame->width,frame->height),frame->depth,1); //Hue
    IplImage* v = cvCreateImage(cvSize(frame->width,frame->height),frame->depth,1); //Value
    IplImage* laser = cvCreateImage(cvSize(frame->width,frame->height),frame->depth,1); //Laser dot

    cvSplit(frame, NULL, NULL, r, NULL); //Split the frame into b, g, and r
    cvCvtColor(frame, hsvframe, CV_RGB2HSV); //Convert the color from bgr to hsv
    cvSplit(hsvframe, h, NULL, v, NULL); //Split the HSV image
    
    cvThreshold(v,v,250,255,CV_THRESH_BINARY); //Value between 250 and 255
    cvThreshold(r,r,250,255,CV_THRESH_BINARY); //Red between 250 and 255
    cvThreshold(h,h,0,6,CV_THRESH_BINARY); //Hue between 0 and 6
    
    cvAnd(r,v,laser); //All three of the above conditions have to be met
    cvAnd(h,laser,laser);

    laserx = findLaser(laser); //Find the laser in the black and white thresholded image
    pixdist = laserx-xPixels/2; //Pixdist is laserx - center

    cvReleaseImage(&r);
    cvReleaseImage(&hsvframe);
    cvReleaseImage(&h);
    cvReleaseImage(&v);
    cvReleaseImage(&laser);

    if (laserx == -1) return -1; //Laser not found

    //This is a function based on calibrations that approximates the angle based on the pixels from the center
    //1.1452+0.0904481*x+0.0000341542*x^2-1.72975x10^-7*x^3
    angle = 1.1452+0.0904481*pixdist+0.0000341542*pixdist*pixdist-1.72975*.0000001*pixdist*pixdist*pixdist;
    angle *= .0175; //Convert degrees to radians
    return camLaserDist/(tan(angle)); //distance = camLaserdist/tan(angle)
}

double findLaser(IplImage* frame){
    double x, y; //X and Y indecies
	CvScalar pixel;
	for(x=xPixels/2; x<xPixels; x++) {

		for(y=yMin; y<yMax; y++) {

			pixel = cvGet2D(frame, y, x); //Get the pixel value at x,y

			if(pixel.val[0]!=0) return x; //Laser dot found, return x-coord

		}
	}
	return -1; //If the laser dot is not found return -1

}


