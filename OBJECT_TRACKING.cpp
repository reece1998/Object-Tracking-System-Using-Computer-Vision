#include "opencv2/opencv.hpp"
#include <stdint.h>
#include <iostream>
#include <sstream>
#include <chrono>

using namespace cv;
using namespace std;

/// Creating vectors
vector<vector<Point>> contours;
vector<Vec4i> hierarchy;
vector<double>timeStamp;
vector<float>xLists;
///creating varibles to use
float velocity;
int sampleSize, distance_dropletTrench, totalDistance, totalLengthOfRulerVis;

int main(int argv, char** argc){
    ///get start time
    auto start = std::chrono::steady_clock::now(); // starts clock for calculating volocity

    ///Create Background Subtractor object using pointers
    Ptr<BackgroundSubtractor> backgroundSubtractor = createBackgroundSubtractorMOG2();
    ///Creates mats for use in object detection and tracking
    Mat threshold_output, frame, frame2, fgMask;

    VideoCapture liveFeed(0); // capture videos from raspberry pi camera

    /// Creation of control panel
    namedWindow("Camera", WINDOW_AUTOSIZE);
    createTrackbar("Total length of ruler visible(cm)","Camera",0,100);
    createTrackbar("Distance to droplet trench(cm)","Camera",0,100);
    createTrackbar("Distance to background ruler(cm)","Camera",0,100);
    createTrackbar("Measurments taken before calculation of volocity","Camera",0,20);
    /// check to see if camera is initialised
    if (!liveFeed.isOpened())
    {
        ///error in opening the video input
        return -1;
    }
    /// start of endless while loop
    while(liveFeed.read(frame)) // while frame can be read from camera
    {

        if (frame.empty())
        {
            break;
        }
        /// sets up control panel to be implemented into the system
        totalLengthOfRulerVis=  getTrackbarPos("Total length of ruler visible(cm)", "Camera");
        distance_dropletTrench= getTrackbarPos("Distance to droplet trench(cm)", "Camera");
        totalDistance=          getTrackbarPos("Distance to background ruler(cm)", "Camera");
        sampleSize=             getTrackbarPos("Measurments taken before calculation of volocity", "Camera");
        /// Convert colour to greyscale
        cvtColor(frame,frame2, COLOR_RGB2GRAY,0);
        /// apply guassian blur with kernel of 3x3
        GaussianBlur(frame2,frame2,Size(3,3),0);
        /// apply background mask to create foreground mask
        backgroundSubtractor-> apply(frame2, fgMask);
        ///create threshold and output as threshhold_output
        threshold(fgMask, threshold_output, 160, 225, THRESH_BINARY);
        ///find the contours of threshold_output and output as contours
        findContours(threshold_output, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE,Point(0, 0));

        vector<vector<Point>> hull(contours.size() ); // initialise vector for hull
        /// Create convex hull outline for detected object and store the position & time
        for( int i = 0; i < contours.size(); i++ ) // iterate through all contours
        {
            if ( contourArea(contours[i])>300) // preprogrammed sensitivey filter
            {
                    convexHull(contours[i], hull[i], false );
                    auto placer = std::chrono::steady_clock::now();
                    double internalTime = double (std::chrono::duration_cast<
                    std::chrono::nanoseconds>(placer-start).count());
                    internalTime = internalTime/1e9;
                    xLists.push_back(hull[i][0].x); // store position of the right most point in convex hull
                    timeStamp.push_back(internalTime); //store time of when the object was detected
            }
        }

        RNG rng(12345); // random number generator

        /// Draw contours + hull results + volocity
        for( int i = 0; i< contours.size(); i++ ){ // iterate through all contours

            if (contourArea(contours[i])>300){ // same preprogrammed sensitivey filter to stop errors
                Scalar colour = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) ); // create colour for outline
                // for convex hull and foreground mask
                drawContours(fgMask, hull, i, colour/4, 3, 8, vector<Vec4i>(), 0, Point() ); // convex hull outlined on foreground mask
                drawContours(frame, hull, i, colour, 3, 8, vector<Vec4i>(), 0, Point() ); // convex hull outlined on current frame
                circle(frame,Point(hull[i][0]),4,(100,10,100),3); // draw cired on right most point on convex hull
                /// calculations for volocity including  accounting for parallax error
                while(sampleSize>=2){ // only run code if there is a 2 or more for the samplesize
                    if(xLists.size() == sampleSize){ // if the number of positions recored is equal to sample size
                    // calculate actual position and then the volcity
                        cout<<sampleSize<<" positions recorded"<<endl;
                        for(int i = 0; i < xLists.size(); i++){

                            xLists[i] =(xLists[i]/(639/totalLengthOfRulerVis)); //Calculate perceived position in cm
                            xLists[i] =(distance_dropletTrench*xLists[i])/(totalDistance); //Calculate actual postion in cm
                            /// Output useful information about position of object
                            cout<<"The position of measurement"<<i<<" is: "<<xLists[i]<<endl;
                        }
                        /// Output useful information about position of object
                        cout<<"The smallest position in group: " <<xLists[0]<<" Recorded at:"<<timeStamp[0]<<endl;
                        cout<<"The largest position in  group: " <<xLists.back()<<" Recroded at:"<<timeStamp.back()<<endl;
                        ////////
                        /// Calculate volocity
                        velocity = ((xLists.back()-xLists[0]) / (timeStamp.back()-timeStamp[0]));
                        cout<<"The velocity is of the object is: "<<velocity<<" cm/s"<<endl;
                        cout<<endl;

                        /// clear vectors holding position of object and time position was measured
                        xLists.clear();
                        timeStamp.clear();
                    }
                    // account for errors
                    else if(xLists.size()>sampleSize){
                        cout<<"error in finding velocity"<<endl;
                        xLists.clear();
                        timeStamp.clear();
                    }
                    else{
                        break;
                    }

                }
            }
        }

        /// Show results
        imshow("Camera", frame);
        imshow("Foreground Mask", fgMask);
        int Exit = waitKey(1);
        if (Exit == 'q' || Exit == 27)
            break;
    }
    ///Display runtime of whole system
    auto End = std::chrono::steady_clock::now();
    double Elapsed_time = double (std::chrono::duration_cast<
            std::chrono::nanoseconds>(End-start).count()); // take time in nanoseconds
    cout<<"total = "<<Elapsed_time/1e9<< endl; // convert to seconds
    return 1;

}

