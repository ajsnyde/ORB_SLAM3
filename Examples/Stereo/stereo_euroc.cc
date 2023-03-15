/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/
#include <cstdlib>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;
using namespace cv;

int c = 0;
string int2str(int &);
void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

int main(int argc, const char *argv[])
{  
	if (argc < 4) {
		cerr << endl
				<< "Usage: ./mono_euroc path_to_vocabulary path_to_settings path_to_left_video_file path_to_right_video_file left_frames_to_discard_may_be_negative) (trajectory_file_name)"
				<< endl;
		return 1;
	}

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO,
			true);

	std::string leftVideoFilePath = argv[3];
	std::string rightVideoFilePath = argv[4];
	VideoCapture leftCap(leftVideoFilePath); // video
	sleep(1);
	if (!leftCap.isOpened()) {
		cout << "Cannot open the video file: " + leftVideoFilePath << endl;
		return -1;
	}
	double leftFps = leftCap.get(CAP_PROP_FPS); //get the frames per seconds of the video

	VideoCapture rightCap(rightVideoFilePath); // video
	if (!rightCap.isOpened()) {
		cout << "Cannot open the video file: " + rightVideoFilePath << endl;
		return -1;
	}
	double rightFps = rightCap.get(CAP_PROP_FPS); //get the frames per seconds of the video

	cout << "Frame per seconds - Left: " << leftFps << " Right: " << rightFps << endl;

	//namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

	//std::string offsetString << argv[4];
	int offset = atoi(argv[4]);

	cv::Mat leftFrame;
	cv::Mat rightFrame;
	// offset frame stream by x frames; negative indicates to toss right-frames, positive to toss left
	for(int i = 0; i < abs(offset); i++){
		if(offset < 0){
			rightCap.read(rightFrame);
		} else if (offset > 0){
			leftCap.read(leftFrame);
		}
	}

	for (double i = 0; true; i++) {
		bool leftSuccess = leftCap.read(leftFrame); // read a new frame from video
		bool rightSuccess = rightCap.read(rightFrame); // read a new frame from video

		if (!leftSuccess) {
			cout << "Cannot read the frame from video file: " + leftVideoFilePath
					<< endl;
			break;
		}
		if (!rightSuccess) {
			cout << "Cannot read the frame from video file: " + rightVideoFilePath
					<< endl;
			break;
		}

		//imshow("MyVideo", frame); //show the frame in "MyVideo" window
		//imwrite("ig" + std::to_string(i) + ".jpg", frame);

		// Pass the image to the SLAM system
		SLAM.TrackStereo(leftFrame, rightFrame, i * (1 / leftFps));

		if (waitKey(30) == 27) //esc key
				{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}


//
//    // Read rectification parameters
//    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
//    if(!fsSettings.isOpened())
//    {
//        cerr << "ERROR: Wrong path to settings" << endl;
//        return -1;
//    }
//
//    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
//    fsSettings["LEFT.K"] >> K_l;
//    fsSettings["RIGHT.K"] >> K_r;
//
//    fsSettings["LEFT.P"] >> P_l;
//    fsSettings["RIGHT.P"] >> P_r;
//
//    fsSettings["LEFT.R"] >> R_l;
//    fsSettings["RIGHT.R"] >> R_r;
//
//    fsSettings["LEFT.D"] >> D_l;
//    fsSettings["RIGHT.D"] >> D_r;
//
//    int rows_l = fsSettings["LEFT.height"];
//    int cols_l = fsSettings["LEFT.width"];
//    int rows_r = fsSettings["RIGHT.height"];
//    int cols_r = fsSettings["RIGHT.width"];
//
//    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
//            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
//    {
//        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
//        return -1;
//    }
//
//    cv::Mat M1l,M2l,M1r,M2r;
//    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
//    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
//
//
//    // Vector for tracking time statistics
//    vector<float> vTimesTrack;
//    vTimesTrack.resize(tot_images);
//
//    cout << endl << "-------" << endl;
//    cout.precision(17);
//
//    // Create SLAM system. It initializes all system threads and gets ready to process frames.
//    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO, true);
//
//    cv::Mat imLeft, imRight, imLeftRect, imRightRect;
//    for (seq = 0; seq<num_seq; seq++)
//    {
//
//        // Seq loop
//
//        double t_rect = 0;
//        double t_track = 0;
//        int num_rect = 0;
//        int proccIm = 0;
//        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
//        {
//            // Read left and right images from file
//            imLeft = cv::imread(vstrImageLeft[seq][ni],cv::IMREAD_UNCHANGED);
//            imRight = cv::imread(vstrImageRight[seq][ni],cv::IMREAD_UNCHANGED);
//
//            if(imLeft.empty())
//            {
//                cerr << endl << "Failed to load image at: "
//                     << string(vstrImageLeft[seq][ni]) << endl;
//                return 1;
//            }
//
//            if(imRight.empty())
//            {
//                cerr << endl << "Failed to load image at: "
//                     << string(vstrImageRight[seq][ni]) << endl;
//                return 1;
//            }
//
//#ifdef REGISTER_TIMES
//    #ifdef COMPILEDWITHC11
//            std::chrono::steady_clock::time_point t_Start_Rect = std::chrono::steady_clock::now();
//    #else
//            std::chrono::monotonic_clock::time_point t_Start_Rect = std::chrono::monotonic_clock::now();
//    #endif
//#endif
//            cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
//            cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);
//
//#ifdef REGISTER_TIMES
//    #ifdef COMPILEDWITHC11
//            std::chrono::steady_clock::time_point t_End_Rect = std::chrono::steady_clock::now();
//    #else
//            std::chrono::monotonic_clock::time_point t_End_Rect = std::chrono::monotonic_clock::now();
//    #endif
//            t_rect = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Rect - t_Start_Rect).count();
//            SLAM.InsertRectTime(t_rect);
//#endif
//            double tframe = vTimestampsCam[seq][ni];
//    #ifdef COMPILEDWITHC11
//            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//    #else
//            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
//    #endif
//
//            // Pass the images to the SLAM system
//            SLAM.TrackStereo(imLeftRect,imRightRect,tframe, vector<ORB_SLAM3::IMU::Point>(), vstrImageLeft[seq][ni]);
//
//    #ifdef COMPILEDWITHC11
//            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//    #else
//            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
//    #endif
//
//#ifdef REGISTER_TIMES
//            t_track = t_rect + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
//            SLAM.InsertTrackTime(t_track);
//#endif
//
//            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
//
//            vTimesTrack[ni]=ttrack;
//
//            // Wait to load the next frame
//            double T=0;
//            if(ni<nImages[seq]-1)
//                T = vTimestampsCam[seq][ni+1]-tframe;
//            else if(ni>0)
//                T = tframe-vTimestampsCam[seq][ni-1];
//
//            if(ttrack<T)
//                usleep((T-ttrack)*1e6);
//        }
//
//        if(seq < num_seq - 1)
//        {
//            cout << "Changing the dataset" << endl;
//
//            SLAM.ChangeDataset();
//        }
//
//    }
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory

	const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
	const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
	SLAM.SaveTrajectoryEuRoC(f_file);
	SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);


    return 0;
}

string int2str(int &i) {
    string s;
    stringstream ss(s);
    ss << i;
    return ss.str();
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}
