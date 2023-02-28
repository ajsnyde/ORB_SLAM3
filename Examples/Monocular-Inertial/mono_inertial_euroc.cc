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

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

double ttrack_tot = 0;
int main(int argc, char *argv[])
{

    if(argc < 5)
    {
        cerr << endl << "Usage: ./mono_inertial_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) " << endl;
        return 1;
    }




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
	int frame_skip = atoi(argv[6]);

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM3::System SLAM(path_to_vocab, path_to_settings, ORB_SLAM3::System::IMU_MONOCULAR,
			true);

	vector< vector<double> > vTimestampsImu;

	vector<ORB_SLAM3::IMU::Point> vImuMeas;

    LoadIMU(path_to_imu, vImuMeas);
    cout << "Loaded IMU!" << endl;




	VideoCapture cap(path_to_video); // video
	if (!cap.isOpened()) {
		cout << "Cannot open the video file: " + path_to_video << endl;
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
			cout << "Cannot read the frame from video file: " + path_to_video
					<< endl;
			break;
		}

		//imshow("MyVideo", frame); //show the frame in "MyVideo" window
		//imwrite("ig" + std::to_string(i) + ".jpg", frame);




		if (i % (frame_skip + 1) == 0) {
			// Pass the image to the SLAM system
			SLAM.TrackMonocular(frame, i * (1 / fps), vImuMeas[i]);
		}

		if (waitKey(30) == 27) //esc key
				{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}


//
//    const int num_seq = (argc-3)/2;
//    cout << "num_seq = " << num_seq << endl;
//    bool bFileName= (((argc-3) % 2) == 1);
//    string file_name;
//    if (bFileName)
//    {
//        file_name = string(argv[argc-1]);
//        cout << "file name: " << file_name << endl;
//    }
//
//    // Load all sequences:
//    int seq;
//    vector< vector<string> > vstrImageFilenames;
//    vector< vector<double> > vTimestampsCam;
//    vector< vector<cv::Point3f> > vAcc, vGyro;
//    vector< vector<double> > vTimestampsImu;
//    vector<int> nImages;
//    vector<int> nImu;
//    vector<int> first_imu(num_seq,0);
//
//    vstrImageFilenames.resize(num_seq);
//    vTimestampsCam.resize(num_seq);
//    vAcc.resize(num_seq);
//    vGyro.resize(num_seq);
//    vTimestampsImu.resize(num_seq);
//    nImages.resize(num_seq);
//    nImu.resize(num_seq);
//
//    int tot_images = 0;
//    for (seq = 0; seq<num_seq; seq++)
//    {
//        cout << "Loading images for sequence " << seq << "...";
//
//        string pathSeq(argv[(2*seq) + 3]);
//        string pathTimeStamps(argv[(2*seq) + 4]);
//
//        string pathCam0 = pathSeq + "/mav0/cam0/data";
//        string pathImu = pathSeq + "/mav0/imu0/data.csv";
//
//        LoadImages(pathCam0, pathTimeStamps, vstrImageFilenames[seq], vTimestampsCam[seq]);
//        cout << "LOADED!" << endl;
//
//        cout << "Loading IMU for sequence " << seq << "...";
//        LoadIMU(pathImu, vTimestampsImu[seq], vImuMeas);
//        cout << "LOADED!" << endl;
//
//        nImages[seq] = vstrImageFilenames[seq].size();
//        tot_images += nImages[seq];
//        nImu[seq] = vTimestampsImu[seq].size();
//
//        if((nImages[seq]<=0)||(nImu[seq]<=0))
//        {
//            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
//            return 1;
//        }
//
//        // Find first imu to be considered, supposing imu measurements start first
//
//        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
//            first_imu[seq]++;
//        first_imu[seq]--; // first imu measurement to be considered
//
//    }
//
//    // Vector for tracking time statistics
//    vector<float> vTimesTrack;
//    vTimesTrack.resize(tot_images);
//
//    cout.precision(17);
//
//    // Create SLAM system. It initializes all system threads and gets ready to process frames.
//    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR, true);
//
//    int proccIm=0;
//    for (seq = 0; seq<num_seq; seq++)
//    {
//        cv::Mat im;
//        vector<ORB_SLAM3::IMU::Point> vImuMeas;
//        proccIm = 0;
//        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
//        {
//            // Read image from file
//            im = cv::imread(vstrImageFilenames[seq][ni],cv::IMREAD_UNCHANGED);
//
//            double tframe = vTimestampsCam[seq][ni];
//
//            if(im.empty())
//            {
//                cerr << endl << "Failed to load image at: "
//                     <<  vstrImageFilenames[seq][ni] << endl;
//                return 1;
//            }
//
//            // Load imu measurements from previous frame
//            vImuMeas.clear();
//

//
//
//    #ifdef COMPILEDWITHC11
//            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//    #else
//            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
//    #endif
//
//            // Pass the image to the SLAM system
//            SLAM.TrackMonocular(im,tframe,vImuMeas);
//
//    #ifdef COMPILEDWITHC11
//            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//    #else
//            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
//    #endif
//
//#ifdef REGISTER_TIMES
//            double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
//            SLAM.InsertTrackTime(t_track);
//#endif
//
//            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
//            ttrack_tot += ttrack;
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
//        if(seq < num_seq - 1)
//        {
//            cout << "Changing the dataset" << endl;
//
//            SLAM.ChangeDataset();
//        }
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
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

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

            // acc.x,y,z , gyro.x,y,z , time
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(data[4],data[5],data[6],
            		data[1],data[2],data[3],data[0]));
        }
    }
}
