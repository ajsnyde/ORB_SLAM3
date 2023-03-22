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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>

#include<opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"

using namespace std;
using namespace cv;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<ORB_SLAM3::IMU::Point> &vImuMeas);

double ttrack_tot = 0;

double timeRelativeToStart = 0;
const double* ptrTimeRelativeToStart = &timeRelativeToStart;
double fps;
const double* fpsPtr = &fps;

int frame_skip;
const int* frame_skipPtr = &frame_skip;



int main(int argc, char *argv[])
{
	printf("OpenCV: %s", cv::getBuildInformation().c_str());

	if (argc < 2) {
		cerr << endl
				<< "Usage: ./mono_euroc path_to_vocabulary path_to_settings (path_to_video_file) (path_to_imu_file) (trajectory_file_name) (frame_skip)"
				<< endl;
		return 1;
	}

	string path_to_vocab = argv[1];
	string path_to_settings = argv[2];
	string path_to_video = argv[3];
	string path_to_imu = argv[4];
	string path_to_trajectory_output = argv[5];
	frame_skip = atoi(argv[6]);

    cout << "Loaded args" << endl;
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM3::System SLAM(path_to_vocab, path_to_settings, ORB_SLAM3::System::IMU_MONOCULAR,
			true);

	vector< vector<double> > vTimestampsImu;

	vector<ORB_SLAM3::IMU::Point> vImuMeas;
	vector<ORB_SLAM3::IMU::Point> vImuMeasSlice;

    LoadIMU(path_to_imu, vImuMeas);
    cout << "Loaded IMU!" << endl;




	VideoCapture cap(path_to_video); // video
	if (!cap.isOpened()) {
		cout << "Cannot open the video file: " + path_to_video << endl;
		return -1;
	}
	fps = cap.get(cv::CAP_PROP_FPS); //get the frames per seconds of the video

	cout << "Frame per seconds : " << fps << endl;

	//namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

	cv::Mat frame;


	for (int i = 1; true; i++) {
		//imshow("MyVideo", frame); //show the frame in "MyVideo" window
		//imwrite("ig" + std::to_string(i) + ".jpg", frame);

		// if skip = 1, i=0,2,4,6
		// if skip = 2, i=3,6,9

		auto predicate = [](ORB_SLAM3::IMU::Point point)
				{return (point.t/1e+9 <= *ptrTimeRelativeToStart && point.t/1e+9 >= (*ptrTimeRelativeToStart - ((1+*frame_skipPtr)/ *fpsPtr)));};


		// Pass the image to the SLAM system

		if ((i+1) % (frame_skip + 1) == 0) {
			std::copy_if(vImuMeas.begin(), vImuMeas.end(), std::back_inserter(vImuMeasSlice), predicate);
			cout << "vImuMeasSlice size: " << vImuMeasSlice.size() << endl;

			bool bSuccess = cap.read(frame); // read a new frame from video

			if (!bSuccess) {
				cout << "Cannot read the frame from video file: " + path_to_video << endl;
				break;
			}


			SLAM.TrackMonocular(frame, timeRelativeToStart, vImuMeasSlice);

			timeRelativeToStart = (i+1) * (1 / fps);
			vImuMeasSlice.clear();
		} else {
			if (!cap.grab()) {
				cout << "Cannot read the frame from video file: " + path_to_video << endl;
				break;
			}
		}

		if (waitKey(30) == 27) //esc key
				{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
	const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
	const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
	SLAM.SaveTrajectoryEuRoC(f_file);
	SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

void LoadIMU(const string &strImuPath, vector<ORB_SLAM3::IMU::Point> &vImuMeas)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vImuMeas.reserve(5000);

    double startTime = -1;

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            if(startTime == -1){
            	startTime = data[0];
            } else {
            	data[0] -= startTime - .00001;
            }


            // acc.x,y,z , gyro.x,y,z , time (nanoseconds)
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(data[4],data[5],data[6],
            		data[1],data[2],data[3],data[0]));
        }
    }
}
