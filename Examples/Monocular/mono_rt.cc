/**
* This file is part of ORB-SLAM2.
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include"System.h"

using namespace std;
using namespace cv;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }






    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    //#ifdef COMPILEDWITHC11
    //        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //#else
    //        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    //#endif
    // Using time point and system_clock 
    std::chrono::time_point<std::chrono::system_clock> start, end; 

    cout << ".........>>>>>........"  << argv[3] << endl;

    cv::VideoCapture cap(argv[3]);
    if ( !cap.isOpened() )  // isOpened() returns true if capturing has been initialized.
    {
        cout << "Cannot open the video file. \n";
        return -1;
    }

    // double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    // The function get is used to derive a property from the element.
    // Example:
    // CV_CAP_PROP_POS_MSEC :  Current Video capture timestamp.
    // CV_CAP_PROP_POS_FRAMES : Index of the next frame.

    // first argument: name of the window.
    // second argument: flag- types: 
    // WINDOW_NORMAL : The user can resize the window.
    // WINDOW_AUTOSIZE : The window size is automatically adjusted to fitvthe displayed image() ), and you cannot change the window size manually.
    // WINDOW_OPENGL : The window will be created with OpenGL support.

    start = std::chrono::system_clock::now();
    double time_stamp = 0;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    clahe->setTilesGridSize(Size(8,8));
    cv::Mat dst;
    while(1)
    {
        cv::Mat frame;
        cv::Mat gray;
        // Mat object is a basic image container. frame is an object of Mat.

        if (!cap.read(frame)) // if not success, break loop
        // read() decodes and captures the next frame.
        {
            cout<<"\n Cannot read the video file. \n";
            break;
        }
        cvtColor(frame, gray, CV_BGR2GRAY);
        clahe->apply(gray, dst);
        // apply the CLAHE algorithm
      //  cv::equalizeHist(gray, gray);



//        #ifdef COMPILEDWITHC11
//                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//        #else
//                std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
//        #endif
//
//        double tframe = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        
        // Pass the image to the SLAM system
        //end = std::chrono::system_clock::now();
        //std::chrono::duration<double> elapsed_seconds = end - start;
        // cout << "passing ...."  << endl;
        time_stamp = cap.get(cv::CAP_PROP_POS_MSEC) / 1000;
        SLAM.TrackMonocular(dst, time_stamp );
        // cout << "timestamp....... " << time_stamp << "time_el" << elapsed_seconds.count() << endl;


    }

    // Stop all threads
    SLAM.Shutdown();

    cout << "-------" << endl << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");    

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}
