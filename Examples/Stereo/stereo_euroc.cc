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
				<< "Usage: ./mono_euroc path_to_vocabulary path_to_settings path_to_left_video_file path_to_right_video_file left_frames_to_discard_may_be_negative) (frame_skip) (trajectory_file_name)"
				<< endl;
		return 1;
	}

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO,
			true);

	std::string leftVideoFilePath = argv[3];
	std::string rightVideoFilePath = argv[4];
	int frame_skip = atoi(argv[6]);
	VideoCapture leftCap(leftVideoFilePath); // video
	sleep(1);
	if (!leftCap.isOpened()) {
		cout << "Cannot open the video file: " + leftVideoFilePath << endl;
		return -1;
	}
	double leftFps = leftCap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video

	VideoCapture rightCap(rightVideoFilePath); // video
	if (!rightCap.isOpened()) {
		cout << "Cannot open the video file: " + rightVideoFilePath << endl;
		return -1;
	}
	double rightFps = rightCap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video

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




	for (int i = 0; true; i++) {
		if ((i + 1) % (frame_skip + 1) == 0) {

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

		} else {
			if (!leftCap.grab()) {
				cout
						<< "Cannot read the frame from video file: "
								+ leftVideoFilePath << endl;
				break;
			}
			if (!rightCap.grab()) {
				cout
						<< "Cannot read the frame from video file: "
								+ rightVideoFilePath << endl;
				break;
			}
		}


		char c = (char) waitKey(10);
		if (c == 27 || c == 'q' || c == 'Q'){
			cout << "Quitting!" << endl;
		    // Save camera trajectory

			const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
			const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
			SLAM.SaveTrajectoryEuRoC(f_file);
			SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);

			break;
		}
		if (c == 'v' || c == 'v'){
			SLAM.ResetViewer();
		}
	}

    // Stop all threads
    SLAM.Shutdown();



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
