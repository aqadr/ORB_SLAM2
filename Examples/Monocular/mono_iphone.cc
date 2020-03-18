/**
* This file is part of ORB-SLAM2.
*
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

#include<opencv2/core/core.hpp>

#include"System.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/types.h>
using namespace std;

//! \brief get file extension
string getFileExt(const string& s)
{
   size_t i = s.rfind('.', s.length());
   if (i != string::npos)
   {
      return(s.substr(i+1, s.length() - i));
   }

   return("");
}

string getFrameNumber(const string& s)
{
     size_t i = s.rfind('.', s.length());
     if (i != string::npos)
     {
        return(s.substr(5,i-5));
     }
}


std::vector<std::string>imagePathsVec;

std::vector<cv::Mat> imageVec;
void LoadImages(const string &strSequence, std::vector<string> &vstrImageFilenames)
{

    std::vector<std::string> imageNameVec;
    struct dirent *entry;
    DIR *dir = opendir(strSequence.c_str());

    if (dir == NULL) {
        return;
    }
    int cnt=0;
    while ((entry = readdir(dir)) != NULL)
    {
        cout << entry->d_name << endl;
        std::string s(entry->d_name);
        string ext=getFileExt(s);

        if(ext=="png")
        {
            ++cnt;
            string output=strSequence+"/"+s;
            std::cout<<"print path: "<<output<<std::endl;
            imageNameVec.push_back(s);
        }
    }

    std::cout<<"checdk cnt:"<<cnt<<std::endl;
    if(cnt<=0)
        return;

    vstrImageFilenames=std::vector<std::string>(cnt);
    for (int i=0;i<cnt;++i)
    {
        std::string s=imageNameVec.at(i);
        string frameNum=getFrameNumber(s);
        string output=strSequence+"/"+s;
        std::cout<<"print path: "<<output<<std::endl;
        std::cout<<"frame number: "<<frameNum<<std::endl;
        int frameInt=std::atoi(frameNum.c_str());
        vstrImageFilenames.at(frameInt)=output;
    }
    closedir(dir);
}


//high precision time in seconds since epoch
static double getTimeSinceEpoch(std::chrono::high_resolution_clock::time_point* t = nullptr)
{
    using Clock = std::chrono::high_resolution_clock;
    return std::chrono::duration<double>((t != nullptr ? *t : Clock::now() ).time_since_epoch()).count();
}



int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    LoadImages(string(argv[3]), vstrImageFilenames );

    int nImages = vstrImageFilenames.size();
    std::cout<<"check number of images: "<<nImages<<std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);


        double tframe = getTimeSinceEpoch();

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();



        // Wait to load the next frame
        double T=0;
        usleep(1000000);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");    

    return 0;
}
