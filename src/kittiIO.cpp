#include "kittiIO.hpp"

kittiIO::kittiIO(std::string &kittiPath)
{

    // Retrieve paths to images
    std::vector<std::string> vstrImageRGB;
    std::vector<std::string> vstrImageGray;
    std::vector<std::string> vstrPointCloudLiDAR;
    std::string vstrPose;
    std::string vstrCalib;
    std::vector<double> vTimestamps;

    loadData(kittiPath, vstrImageGray, vstrImageRGB, vstrPointCloudLiDAR, vstrPose, vstrCalib, vTimestamps);
    poseTransform(vstrPose);
    makeMap(vstrCalib, vstrPose, vstrPointCloudLiDAR, vstrImageRGB, vstrImageGray);
}

kittiIO::~kittiIO()
{
}

int kittiIO::loadData(const std::string &strPathToSequence, std::vector<std::string> &vstrImageGray, std::vector<std::string> &vstrImageRGB,
                      std::vector<std::string> &vstrPointCloudLiDAR, std::string &vstrPose, std::string &vstrCalib, std::vector<double> &vTimestamps)
{
    std::ifstream fTimes;
    std::string strPathTimeFile = strPathToSequence + "/data_odometry_gray/dataset/sequences/00/times.txt";
    fTimes.open(strPathTimeFile.c_str());

    if (!fTimes.is_open())
    {
        std::cerr << std::endl
                  << "Failed to load path \n";
        std::cerr << "Check args path : " << strPathToSequence << "\n";
        return -1;
    }

    while (!fTimes.eof())
    {
        std::string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    std::string strPrefixGray = strPathToSequence + "/data_odometry_gray/dataset/sequences/00/image_0/";
    std::string strPrefixRGB = strPathToSequence + "/data_odometry_color/dataset/sequences/00/image_2/";
    std::string strPrefixLiDAR = strPathToSequence + "/data_odometry_velodyne/dataset/sequences/00/velodyne/";
    std::string strPrefixPose = strPathToSequence + "/data_odometry_poses/dataset/poses/00.txt";
    std::string strPrefixCalib = strPathToSequence + "/data_odometry_calib/dataset/sequences/00/calib.txt";

    const int nTimes = vTimestamps.size();
    vstrImageRGB.resize(nTimes);
    vstrImageGray.resize(nTimes);
    vstrPointCloudLiDAR.resize(nTimes);
    vstrPose = strPrefixPose;
    vstrCalib = strPrefixCalib;

    for (int i = 0; i < nTimes; ++i)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        vstrImageGray[i] = strPrefixGray + ss.str() + ".png";
        vstrImageRGB[i] = strPrefixRGB + ss.str() + ".png";
        vstrPointCloudLiDAR[i] = strPrefixLiDAR + ss.str() + ".bin";
    }
    return 1;
}
int kittiIO::makeMap(std::string &vstrCalib, std::string &vstrPose, std::vector<std::string> &vstrPointCloudLiDAR,
                     std::vector<std::string> &vstrImageRGB, std::vector<std::string> &vstrImageGray)
{

    std::ifstream fCalib(vstrCalib);

    if (!fCalib.is_open())
    {
        std::cerr << std::endl
                  << "Failed to load path \n";
        return -1;
    }

    int calibInx = 0;

    struct calibDataSt
    {
        std::string naming;
        cv::Mat intrinsicMatrix;
        cv::Mat rotationMatrix;
        cv::Mat tranlationMatrix;
        cv::Mat transformMatrix;
    };

    std::vector<calibDataSt> calVec;

    while (true)
    {
        std::string calib;
        std::stringstream ss;

        std::string naming;
        double f00, f01, f02, f03;
        double f10, f11, f12, f13;
        double f20, f21, f22, f23;
        double f30, f31, f32, f33;

        getline(fCalib, calib);

        if (fCalib.eof())
        {
            break;
        }

        ss.str(calib);
        ss >> naming;
        ss >> f00;
        ss >> f01;
        ss >> f02;
        ss >> f03;
        ss >> f10;
        ss >> f11;
        ss >> f12;
        ss >> f13;
        ss >> f20;
        ss >> f21;
        ss >> f22;
        ss >> f23;
        calibDataSt calSt;
        cv::Mat transformMatrix = (cv::Mat_<double>(3, 4) << f00, f01, f02, f03, f10, f11, f12, f13, f20, f21, f22, f23);

        if (naming == "Tr:")
        {
            calSt.naming = naming;
            calSt.transformMatrix = transformMatrix;
            calVec.push_back(calSt);
            continue;
        }

        calSt.naming = naming;
        cv::decomposeProjectionMatrix(transformMatrix, calSt.intrinsicMatrix, calSt.rotationMatrix, calSt.tranlationMatrix);
        calSt.tranlationMatrix.at<double>(0, 0) = calSt.tranlationMatrix.at<double>(0, 0) / calSt.tranlationMatrix.at<double>(0, 3);
        calSt.tranlationMatrix.at<double>(0, 1) = calSt.tranlationMatrix.at<double>(0, 1) / calSt.tranlationMatrix.at<double>(0, 3);
        calSt.tranlationMatrix.at<double>(0, 2) = calSt.tranlationMatrix.at<double>(0, 2) / calSt.tranlationMatrix.at<double>(0, 3);
        calSt.tranlationMatrix.at<double>(0, 3) = calSt.tranlationMatrix.at<double>(0, 3) / calSt.tranlationMatrix.at<double>(0, 3);

        calVec.push_back(calSt);
    }

    for (int i = 0; i < calVec.size(); i++)
    {
        std::cout << calVec[i].naming << "\n";
        std::cout << calVec[i].intrinsicMatrix << "\n";
        std::cout << calVec[i].rotationMatrix << "\n";
        std::cout << calVec[i].tranlationMatrix << "\n";
        std::cout << calVec[i].transformMatrix << "\n";
        std::cout << "======================="
                  << "\n";
    }

    for (int idx = 0; idx < vstrPointCloudLiDAR.size(); idx++)
    {
        std::cout<< std::fixed <<std::setprecision(6) << "Kitti index : " << idx << "\n";
        std::string pcLiDAR;
        pcLiDAR = vstrPointCloudLiDAR[idx];
        std::ifstream fcloud(pcLiDAR, std::ios::binary);
        fcloud.seekg(0, std::ios::end);
        size_t cloudSize = fcloud.tellg();
        fcloud.seekg(0, std::ios::beg);

        pcl::PointCloud<pcl::PointXYZI>::Ptr binCloud(new pcl::PointCloud<pcl::PointXYZI>());

        typedef struct points_s
        {
            float x, y, z, i;
        } points_t;

        points_t point;

        // std::cout << "sizeof(point); : " << sizeof(point) << "\n";
        for (size_t i = 0; i < cloudSize / sizeof(point); ++i)
        {
            pcl::PointXYZI cloud;

            fcloud.read((char *)&point, sizeof(point));
            // cloud.push_back((cv::Mat_<float>(3, 1) << point.x, point.y, point.z));
            cloud.x = point.x;
            cloud.y = point.y;
            cloud.z = point.z;
            cloud.intensity = point.i;
            binCloud->push_back(cloud);
        }
        fcloud.close();

        cv::Mat p0_img = cv::imread(vstrImageGray[idx]);
        cv::Mat p2_img = cv::imread(vstrImageRGB[idx]);
        // pcl::io::savePCDFileBinary("test.pcd",*binCloud);

        // std::cout << "img size : " << p0_img.size() <<"\n";

        cv::Mat p0_to_lidar = p0_img.clone();
        cv::Mat p2_to_lidar = p2_img.clone();
        for (const auto &point : *binCloud)
        {

            cv::Mat lidarPoints = (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1);
            cv::Mat L2ImgPixel;

            // Eigen::Matrix<double,4,1> lP;
            // lp<< point.x point.ym

            // using Eigen3x3d = Eigen::Matrix<double,3,3>;
            // using Eigen3x1d = Eigen::Matrix<double,3,1>;

            // std::cout << "calVec[4].transformMatrix size : " << calVec[4].transformMatrix.size() <<"\n";

            cv::Mat l2c = calVec[4].transformMatrix * lidarPoints;
            // std::cout << "calVec[4].transformMatrix size : " << calVec[4].transformMatrix.size() <<"\n";
            // std::cout << "l2c size : " << l2c.size() <<"\n";

            // std::cout << " l2c :" << l2c << "\n";
            // exit(1);

            float imgCols = p0_img.cols;
            float imgRows = p0_img.rows;

            // 뒤에있는 z값 처리
            if (l2c.at<double>(2) <= 0)
                continue;

            // std::cout <<"fx : " << calVec[0].intrinsicMatrix.at<double>(0,0) <<"\n";
            // std::cout <<"cx : " << calVec[0].intrinsicMatrix.at<double>(0,2) <<"\n";
            // std::cout <<"fy : " << calVec[0].intrinsicMatrix.at<double>(1,1) <<"\n";
            // std::cout <<"cy : " << calVec[0].intrinsicMatrix.at<double>(1,2) <<"\n";

            // const float &u = calVec[0].intrinsicMatrix.at<float>(0,0);
            // const float &v = calVec[0].intrinsicMatrix.at<float>(0,0);

            // P0 기준 Projection

            const double &p0_u = calVec[0].intrinsicMatrix.at<double>(0, 0) * l2c.at<double>(0) / l2c.at<double>(2) + calVec[0].intrinsicMatrix.at<double>(0, 2);
            const double &p0_v = calVec[0].intrinsicMatrix.at<double>(1, 1) * l2c.at<double>(1) / l2c.at<double>(2) + calVec[0].intrinsicMatrix.at<double>(1, 2);

            if ((0.f <= p0_u) && (p0_u < imgCols) && (0.f <= p0_v) && (p0_v < imgRows) && (0.f < l2c.at<float>(2)) && (p0_v * imgCols + p0_u < imgCols * imgRows))
            {
                int colIdx;
                colIdx = sqrt(l2c.at<double>(0) * l2c.at<double>(0) + l2c.at<double>(1) * l2c.at<double>(1) + l2c.at<double>(2) * l2c.at<double>(2)) * 2;

                int b = colorMap[colIdx][0] * 255;
                int g = colorMap[colIdx][1] * 255;
                int r = colorMap[colIdx][2] * 255;

                // colIdx = round(point.intensity)* 18;
                // int bi = colorMap[colIdx][0] * 255;
                // int gi = colorMap[colIdx][1] * 255;
                // int ri = colorMap[colIdx][2] * 255;
                cv::circle(p0_to_lidar, cv::Point(p0_u, p0_v), 2, cv::Scalar(b, g, r), 1);
            }

            const double &p2_u = calVec[2].intrinsicMatrix.at<double>(0, 0) * l2c.at<double>(0) / l2c.at<double>(2) + calVec[2].intrinsicMatrix.at<double>(0, 2) + calVec[2].tranlationMatrix.at<double>(0, 0);
            // const double &p2_u = calVec[0].intrinsicMatrix.at<double>(0,0) * l2c.at<double>(0) / l2c.at<double>(2) + calVec[0].intrinsicMatrix.at<double>(0,2) - calVec[2].tranlationMatrix.at<double>(0,0);
            const double &p2_v = calVec[2].intrinsicMatrix.at<double>(1, 1) * l2c.at<double>(1) / l2c.at<double>(2) + calVec[2].intrinsicMatrix.at<double>(1, 2);

            if ((0.f <= p2_u) && (p2_u < imgCols) && (0.f <= p2_v) && (p2_v < imgRows) && (0.f < l2c.at<float>(2)) && (p2_v * imgCols + p2_u < imgCols * imgRows))
            {
                int colIdx;
                colIdx = sqrt(l2c.at<double>(0) * l2c.at<double>(0) + l2c.at<double>(1) * l2c.at<double>(1) + l2c.at<double>(2) * l2c.at<double>(2)) * 2;

                int b = colorMap[colIdx][0] * 255;
                int g = colorMap[colIdx][1] * 255;
                int r = colorMap[colIdx][2] * 255;

                // colIdx = round(point.intensity)* 18;
                // int bi = colorMap[colIdx][0] * 255;
                // int gi = colorMap[colIdx][1] * 255;
                // int ri = colorMap[colIdx][2] * 255;
                cv::circle(p2_to_lidar, cv::Point(p2_u, p2_v), 2, cv::Scalar(b, g, r), 1);
            }

            // cv::Mat testes = (cv::Mat_<double>(3,1) << calVec[4].transformMatrix.at<double>(0,0) -  point.x,
            //                 calVec[4].transformMatrix.at<double>(0,1) -  point.y,
            //                 calVec[4].transformMatrix.at<double>(0,2) -  point.z);

            // L2ImgPixel = calVec[0].intrinsicMatrix * calVec[4].transformMatrix * lidarPoints;
            // cv::Mat test;
            // test =  calVec[4].transformMatrix * lidarPoints;
            // L2ImgPixel = calVec[0].intrinsicMatrix * test;

            // std::cout<<"L2ImgPixel.at<double>(0,1) : " << L2ImgPixel.at<double>(0,0) <<"\n";
            // std::cout<<"L2ImgPixel.at<double>(0,2) : " << L2ImgPixel.at<double>(0,1) <<"\n";
            // std::cout<<"L2ImgPixel.at<double>(0,3) : " << L2ImgPixel.at<double>(0,3) <<"\n";
            // std::cout<<"L2ImgPixel.at<double>(0,3) : " << L2ImgPixel.at<double>(1,0) <<"\n";
            // std::cout<<"L2ImgPixel.at<double>(0,3) : " << L2ImgPixel.at<double>(1,1) <<"\n";
            // std::cout<<"L2ImgPixel.at<double>(0,3) : " << L2ImgPixel.at<double>(1,2) <<"\n";
            // std::cout<<"L2ImgPixel.at<double>(0,3) : " << L2ImgPixel.at<double>(1,1) <<"\n";

            // std::cout << calVec[0].intrinsicMatrix << "\n";
            // std::cout << calVec[4].transformMatrix << "\n";

            // if (L2ImgPixel.at<double>(0,0) >= 0 && L2ImgPixel.at<double>(0,0) < p0_img.cols && L2ImgPixel.at<double>(0,1) >= 0 && L2ImgPixel.at<double>(0,1) < p0_img.rows)
            // {
            //     cv::circle(p0_img, cv::Point(L2ImgPixel.at<double>(0,0), L2ImgPixel.at<double>(0,1)), 1, cv::Scalar(255, 255, 255), 2);
            //     std::cout << "L2ImgPixel.at<double>(0,0) : " << L2ImgPixel.at<double>(0,0) <<"\n";
            //     std::cout << "L2ImgPixel.at<double>(0,1) : " << L2ImgPixel.at<double>(0,1) <<"\n";
            //     std::cout << "=======================================================" << "\n";
            // }
        }
        cv::imshow("p0_to_lidar", p0_to_lidar);
        cv::imshow("p2_to_lidar", p2_to_lidar);
        cv::waitKey(0);
    }

    return 1;
}

int kittiIO::readOdometry(std::string &path, std::vector<std::string> &trjVec)
{
    std::cout << "file name : " << path << "\n";

    std::ifstream inputFile(path);
    std::string line;

    while (!inputFile.eof())
    {
        double r11, r12, r13, t14 = 0.0;
        double r21, r22, r23, t24 = 0.0;
        double r31, r32, r33, t34 = 0.0;

        // std::getline(inputFile, line);
        // std::cout<<std::fixed << line << "\n";

        Eigen::Matrix4f kittiTrj;

        sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               &kittiTrj(0, 0), &kittiTrj(0, 1), &kittiTrj(0, 2), &kittiTrj(0, 3),
               &kittiTrj(1, 0), &kittiTrj(1, 1), &kittiTrj(1, 2), &kittiTrj(1, 3),
               &kittiTrj(2, 0), &kittiTrj(2, 1), &kittiTrj(2, 2), &kittiTrj(2, 3));

        std::cout << std::fixed << "r11 : " << r11 << "\n";
        std::cout << kittiTrj << "\n";
        std::cout << "============================="
                  << "\n";
    }
}

int kittiIO::poseTransform(std::string &posePath)
{

    std::ifstream fpose(posePath);
    if (!fpose.is_open())
    {
        std::cerr << std::endl
                  << "Failed to load path \n";
        return -1;
    }

    // std::vector<cv::Mat> true_pose;
    // true_pose.reserve(nImages);

    while (true)
    {

        std::string pose;
        std::stringstream ss;

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

        // std::cout << ss.str() << "\n";

        // Eigen::Matrix3f Rwc;
        // Rwc << p00, p01, p02, p10, p11, p12, p20, p21, p22;

        // Eigen::Matrix<double, 3, 1> Twc;
        // Twc << p03, p13, p23;

        // cv::decomposetransformMatrix()

        // std::cout << std::fixed << "Rwc : " << Rwc << "\n";
        // std::cout << std::fixed << "Twc : " << Twc << "\n";
        // std::cout << "====================================" << "\n";
        // cv::Mat Rwc = (cv::Mat_<float>(3, 3) << p00, p01, p02, p10, p11, p12, p20, p21, p22);
        // cv::Mat twc = (cv::Mat_<float>(3, 1) << p03, p13, p23);
        // cv::Mat Rcw = Rwc.t();
        // cv::Mat tcw = -Rcw * twc;

        // cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
        // Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
        // tcw.copyTo(Tcw.rowRange(0, 3).col(3));
        // true_pose.push_back(Tcw);
    }
    return 1;
}
