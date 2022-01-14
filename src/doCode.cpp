#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>

#include "System.h"

#pragma pack(push, 1)
typedef struct points_s
{
    float x, y, z, i;
} points_t;
#pragma pack(pop)

using namespace std;

void LoadData(const string &strPathToSequence, vector<string> &vstrImageRGB,
              vector<string> &vstrPointCloudLiDAR, string &vstrPose, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        cerr << endl
             << "Usage: ./rgbl_pose_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageRGB;
    vector<string> vstrPointCloudLiDAR;
    string vstrPose;
    vector<double> vTimestamps;
    LoadData(string(argv[3]), vstrImageRGB, vstrPointCloudLiDAR, vstrPose, vTimestamps);

    int nImages = vstrImageRGB.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    MT_VSLAM::System SLAM(MT_VSLAM::System::RGBL_POSE, argv[1], argv[2]);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    // Read true pose
    std::ifstream fpose(vstrPose);
    if (!fpose.is_open())
    {
        cerr << endl
             << "Failed to load pose at: " << vstrPose << endl;
        return 1;
    }

    std::vector<cv::Mat> true_pose;
    true_pose.reserve(nImages);

    while (true)
    {
        std::string pose;
        stringstream ss;

        float p00, p01, p02, p03;
        float p10, p11, p12, p13;
        float p20, p21, p22, p23;

        getline(fpose, pose);
        if (fpose.eof())
            break;

        ss.str(pose);
        ss >> p00;
        ss >> p01;
        ss >> p02;
        ss >> p03;
        ss >> p10;
        ss >> p11;
        ss >> p12;
        ss >> p13;
        ss >> p20;
        ss >> p21;
        ss >> p22;
        ss >> p23;

        cv::Mat Rwc = (cv::Mat_<float>(3, 3) << p00, p01, p02, p10, p11, p12, p20, p21, p22);
        cv::Mat twc = (cv::Mat_<float>(3, 1) << p03, p13, p23);
        cv::Mat Rcw = Rwc.t();
        cv::Mat tcw = -Rcw * twc;

        cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
        Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
        tcw.copyTo(Tcw.rowRange(0, 3).col(3));
        true_pose.push_back(Tcw);
    }

    cout << endl
         << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl
         << endl;

    // Main loop
    cv::Mat imRGB;
    string pcLiDAR;
    for (int ni = 0; ni < nImages; ni++)
    {
        // Read image from file
        imRGB = cv::imread(vstrImageRGB[ni], 1);
        pcLiDAR = vstrPointCloudLiDAR[ni];
        double tframe = vTimestamps[ni];

        if (imRGB.empty())
        {
            cerr << endl
                 << "Failed to load image at: " << vstrImageRGB[ni] << endl;
            return 1;
        }

        std::ifstream fcloud(pcLiDAR, std::ios::binary);
        if (!fcloud.good())
        {
            cerr << endl
                 << "Failed to load point cloud at: " << vstrPointCloudLiDAR[ni] << endl;
            return 1;
        }

        // Obtain file size
        fcloud.seekg(0, std::ios::end);
        size_t cloudSize = fcloud.tellg();
        fcloud.seekg(0, std::ios::beg);

        points_t point;
        vector<cv::Mat> cloud;
        cloud.reserve(cloudSize / sizeof(point));
        for (size_t i = 0; i < cloudSize / sizeof(point); ++i)
        {
            fcloud.read((char *)&point, sizeof(point));
            cloud.push_back((cv::Mat_<float>(3, 1) << point.x, point.y, point.z));
        }
        fcloud.close();

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBL_POSE(imRGB, cloud, true_pose[ni], tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe;
        else if (ni > 0)
            T = tframe - vTimestamps[ni - 1];

        if (ttrack < T)
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T - ttrack) * 1e6)));
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl
         << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("Trajectory.txt");
    SLAM.SaveMapPoints();
    SLAM.SaveMap();

    return 0;
}

void LoadData(const string &strPathToSequence, vector<string> &vstrImageRGB,
              vector<string> &vstrPointCloudLiDAR, string &vstrPose, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/data_odometry_gray/sequences/00/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixRGB = strPathToSequence + "/data_odometry_gray/sequences/00/image_0/";
    string strPrefixLiDAR = strPathToSequence + "/data_odometry_velodyne/sequences/00/velodyne/";
    string strPrefixPose = strPathToSequence + "/data_odometry_poses/poses/00.txt";

    const int nTimes = vTimestamps.size();
    vstrImageRGB.resize(nTimes);
    vstrPointCloudLiDAR.resize(nTimes);
    vstrPose = strPrefixPose;

    for (int i = 0; i < nTimes; ++i)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageRGB[i] = strPrefixRGB + ss.str() + ".png";
        vstrPointCloudLiDAR[i] = strPrefixLiDAR + ss.str() + ".bin";
    }
}
