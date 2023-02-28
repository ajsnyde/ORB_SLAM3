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

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;
using namespace cv;
int c = 0;
string int2str(int &);

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

int main(int argc, const char *argv[])
{
	if (argc < 2) {
		cerr << endl
				<< "Usage: ./mono_euroc path_to_vocabulary path_to_settings path_to_video_file) (trajectory_file_name) (frame_skip)"
				<< endl;
		return 1;
	}

	int frame_skip = atoi(argv[4]);

    string s;





	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR,
			true);
	std::string videoFilePath = argv[3];
	VideoCapture cap(videoFilePath); // video
	if (!cap.isOpened()) {
		cout << "Cannot open the video file: " + videoFilePath << endl;
		return -1;
	}
	double fps = cap.get(cv::CAP_PROP_FPS); //get the frames per seconds of the video

	cout << "Frame per seconds : " << fps << endl;

	//namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

	cv::Mat frame;

	for (int i = 0; true; i++) {
		Mat Gray_frame;
		bool bSuccess = cap.read(frame); // read a new frame from video

		if (!bSuccess) {
			cout << "Cannot read the frame from video file: " + videoFilePath
					<< endl;
			break;
		}

		//imshow("MyVideo", frame); //show the frame in "MyVideo" window
		//imwrite("ig" + std::to_string(i) + ".jpg", frame);

		if (i % (frame_skip + 1) == 0) {
			// Pass the image to the SLAM system
			SLAM.TrackMonocular(frame, i * (1 / fps));
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

string int2str(int &i) {
    string s;
    stringstream ss(s);
    ss << i;
    return ss.str();
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
