#include "kittiOdom.hpp"

KITTI::KITTI(std::string &kittiPath, std::string &seqNum)
{

    // Retrieve paths to images
    std::vector<std::string> vstrCam0;
    std::vector<std::string> vstrCam1;

    std::vector<std::string> vstrCam2;
    std::vector<std::string> vstrCam3;
    
    std::vector<std::string> vstrPointCloudLiDAR;
    std::string vstrPose;
    std::string vstrCalib;
    std::vector<double> vTimestamps;

    colorMap_ = pcl::PointCloud<pcl::PointXYZRGBI>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBI>);
    seqNum_ = seqNum;
    loadData(kittiPath, vstrCam0, vstrCam1, vstrCam2, vstrCam3, vstrPointCloudLiDAR, vstrPose, vstrCalib, vTimestamps);
    projection(vstrCalib, vstrPose, vstrPointCloudLiDAR, vstrCam0, vstrCam1, vstrCam2, vstrCam3);
}

KITTI::~KITTI()
{
}

int KITTI::loadData(const std::string &strPathToSequence, std::vector<std::string> &vstrCam0, std::vector<std::string> &vstrCam1 ,std::vector<std::string> &vstrCam2,std::vector<std::string> &vstrCam3,
                      std::vector<std::string> &vstrPointCloudLiDAR, std::string &vstrPose, std::string &vstrCalib, std::vector<double> &vTimestamps)
{
    std::ifstream fTimes;
    std::string strPathTimeFile = strPathToSequence + "/data_odometry_gray/dataset/sequences/"+ seqNum_ +"/times.txt";
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

    std::string strPrefixCam0 = strPathToSequence + "/data_odometry_gray/dataset/sequences/"+ seqNum_ +"/image_0/";
    std::string strPrefixCam1 = strPathToSequence + "/data_odometry_gray/dataset/sequences/"+ seqNum_ +"/image_1/";
    std::string strPrefixCam2 = strPathToSequence + "/data_odometry_color/dataset/sequences/"+ seqNum_ +"/image_2/";
    std::string strPrefixCam3 = strPathToSequence + "/data_odometry_color/dataset/sequences/"+ seqNum_ +"/image_3/";

    std::string strPrefixLiDAR = strPathToSequence + "/data_odometry_velodyne/dataset/sequences/"+ seqNum_ +"/velodyne/";
    std::string strPrefixPose = strPathToSequence + "/data_odometry_poses/dataset/poses/"+ seqNum_ +".txt";
    std::string strPrefixCalib = strPathToSequence + "/data_odometry_calib/dataset/sequences/"+ seqNum_ +"/calib.txt";

    const int nTimes = vTimestamps.size();
    vstrCam0.resize(nTimes);
    vstrCam1.resize(nTimes);
    vstrCam2.resize(nTimes);
    vstrCam3.resize(nTimes);
    
    vstrPointCloudLiDAR.resize(nTimes);
    vstrPose = strPrefixPose;
    vstrCalib = strPrefixCalib;

    for (int i = 0; i < nTimes; ++i)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        vstrCam0[i] = strPrefixCam0 + ss.str() + ".png";
        vstrCam1[i] = strPrefixCam1 + ss.str() + ".png";
        vstrCam2[i] = strPrefixCam2 + ss.str() + ".png";
        vstrCam3[i] = strPrefixCam3 + ss.str() + ".png";
        vstrPointCloudLiDAR[i] = strPrefixLiDAR + ss.str() + ".bin";
    }
    return 1;
}
int KITTI::projection(std::string &vstrCalib, std::string &vstrPose, std::vector<std::string> &vstrPointCloudLiDAR,
                     std::vector<std::string> &vstrCam0, std::vector<std::string> &vstrCam1, std::vector<std::string> &vstrCam2, std::vector<std::string> &vstrCam3)
{

    std::ifstream fCalib(vstrCalib);

    if (!fCalib.is_open())
    {
        std::cerr << std::endl
                  << "Failed to load path \n";
        return -1;
    }

    int calibInx = 0;

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


    int idx = 0;
    while(true)
    {
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

        const cv::Mat p0_img = cv::imread(vstrCam0[idx]);
        const cv::Mat p1_img = cv::imread(vstrCam1[idx]);
        const cv::Mat p2_img = cv::imread(vstrCam2[idx]);
        const cv::Mat p3_img = cv::imread(vstrCam3[idx]);

        std::vector<cv::Mat> camImgVec;

        camImgVec.resize(4);
        camImgVec[0] = p0_img.clone();
        camImgVec[1] = p1_img.clone();
        camImgVec[2] = p2_img.clone();
        camImgVec[3] = p3_img.clone();

    
        std::vector<std::string> strVecTokenP0;
        std::vector<std::string> strVecTokenP1;
        std::vector<std::string> strVecTokenP2;
        std::vector<std::string> strVecTokenP3;

        tokeNize(vstrCam0[idx], strVecTokenP0, "/");
        tokeNize(vstrCam1[idx], strVecTokenP1, "/");
        tokeNize(vstrCam2[idx], strVecTokenP2, "/");
        tokeNize(vstrCam3[idx], strVecTokenP3, "/");

        std::vector<cv::Mat> projectionImg;


        // std::cout << "sizeof(point); : " << sizeof(point) << "\n";
        for (size_t i = 0; i < cloudSize / sizeof(point); ++i)
        {
            fcloud.read((char *)&point, sizeof(point));

            cv::Mat lidarPoints = (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1);
            cv::Mat L2ImgPixel;

            cv::Mat l2c = calVec[4].transformMatrix * lidarPoints;

            // 뒤에있는 z값 처리
            if (l2c.at<double>(2) <= 0)
                continue;
            
            img2LProection(l2c, calVec,camImgVec ,projectionImg);

        }
        fcloud.close();

        
        int num =0 ;
        
        float imgCols = p0_img.cols;
        float imgRows = p0_img.rows;
        
        cv::namedWindow(strVecTokenP0[strVecTokenP0.size()-3] + "/" + strVecTokenP0[strVecTokenP0.size()-2]);
        cv::namedWindow(strVecTokenP1[strVecTokenP1.size()-3] + "/" + strVecTokenP1[strVecTokenP1.size()-2]);
        cv::namedWindow(strVecTokenP2[strVecTokenP2.size()-3] + "/" + strVecTokenP2[strVecTokenP2.size()-2]);
        cv::namedWindow(strVecTokenP3[strVecTokenP3.size()-3] + "/" + strVecTokenP3[strVecTokenP3.size()-2]);

        cv::moveWindow(strVecTokenP0[strVecTokenP0.size()-3] + "/" + strVecTokenP0[strVecTokenP0.size()-2],0,0);
        cv::moveWindow(strVecTokenP1[strVecTokenP1.size()-3] + "/" + strVecTokenP1[strVecTokenP1.size()-2],imgCols,0);
        cv::moveWindow(strVecTokenP2[strVecTokenP2.size()-3] + "/" + strVecTokenP2[strVecTokenP2.size()-2],0,imgRows);
        cv::moveWindow(strVecTokenP3[strVecTokenP3.size()-3] + "/" + strVecTokenP3[strVecTokenP3.size()-2],imgCols,imgRows);

        cv::rectangle(projectionImg[0], cv::Point(0, 0), cv::Point(100, 25), cv::Scalar(0, 0, 0), -1);
        cv::rectangle(projectionImg[1], cv::Point(0, 0), cv::Point(100, 25), cv::Scalar(0, 0, 0), -1);
        cv::rectangle(projectionImg[2], cv::Point(0, 0), cv::Point(100, 25), cv::Scalar(0, 0, 0), -1);
        cv::rectangle(projectionImg[3], cv::Point(0, 0), cv::Point(100, 25), cv::Scalar(0, 0, 0), -1);

        cv::putText(projectionImg[0], strVecTokenP0[strVecTokenP0.size()-1], cv::Point(0, 18), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
        cv::putText(projectionImg[1], strVecTokenP1[strVecTokenP1.size()-1], cv::Point(0, 18), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
        cv::putText(projectionImg[2], strVecTokenP2[strVecTokenP2.size()-1], cv::Point(0, 18), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
        cv::putText(projectionImg[3], strVecTokenP3[strVecTokenP3.size()-1], cv::Point(0, 18), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));

        cv::imshow(strVecTokenP1[strVecTokenP1.size()-3] + "/" + strVecTokenP1[strVecTokenP1.size()-2], projectionImg[1]);
        cv::imshow(strVecTokenP0[strVecTokenP0.size()-3] + "/" + strVecTokenP0[strVecTokenP0.size()-2], projectionImg[0]);
        cv::imshow(strVecTokenP2[strVecTokenP2.size()-3] + "/" + strVecTokenP2[strVecTokenP2.size()-2], projectionImg[2]);
        cv::imshow(strVecTokenP3[strVecTokenP3.size()-3] + "/" + strVecTokenP3[strVecTokenP3.size()-2], projectionImg[3]);
        
        int key = cv::waitKey(10);


        if (key == 27 || key == 'q' || key == 'Q')
        {
            std::cout << "*****************************" << std::endl;
            std::cout << "*                           *" << std::endl;
            std::cout << "*            EXIT           *" << std::endl;
            std::cout << "*                           *" << std::endl;
            std::cout << "*****************************" << std::endl;
            break;
        }
        else if (key == 'n' || key == 'N')
        {
            idx--;

            if (idx < 0)
            {
                idx = 0;
            }
            // cv::destroyAllWindows();
        }

        else if (key == 'm' || key == 'M')
        {
            // cv::destroyAllWindows();
            idx++;
        }

        else if(key == 's' || key == 'S')
        {
            cv::destroyAllWindows();
            makeColorMap(vstrPointCloudLiDAR, calVec, vstrCam2, vstrPose);   
        }

        if (idx == vstrPointCloudLiDAR.size())
        {
            std::cout << "*****************************" << std::endl;
            std::cout << "*                           *" << std::endl;
            std::cout << "*           FINISH          *" << std::endl;
            std::cout << "*                           *" << std::endl;
            std::cout << "*****************************" << std::endl;
            break;
        }
    }

    return 1;
}
void KITTI::tokeNize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters)
{
	std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
	std::string::size_type pos = str.find_first_of(delimiters, lastPos);

	while (std::string::npos != pos || std::string::npos != lastPos)
	{
		tokens.push_back(str.substr(lastPos, pos - lastPos));
		lastPos = str.find_first_not_of(delimiters, pos);
		pos = str.find_first_of(delimiters, lastPos);
	}
}

void KITTI::img2LProection(cv::Mat &l2c, const std::vector<calibDataSt> &calVec, const std::vector<cv::Mat> &kittiImgVec, std::vector<cv::Mat> &projectionImg)
{

    // std::cout <<"in"<<"\n";

    int imgCols = kittiImgVec[0].cols;
    int imgRows = kittiImgVec[0].rows;
    // projectionImg.resize(kittiImgVec.size());

    for (int i  = 0; i < 4; i++)
    {
        projectionImg.push_back(kittiImgVec[i]);

        double u;
        double v;

        if (i == 1 || i == 3)
        {
            cv::Mat translation2P0 = (cv::Mat_<double>(3,1) << -calVec[i].tranlationMatrix.at<double>(0) ,0, 0);
            cv::Mat fTranslation = l2c + translation2P0;

            u = calVec[i].intrinsicMatrix.at<double>(0, 0) * fTranslation.at<double>(0) / fTranslation.at<double>(2) + calVec[i].intrinsicMatrix.at<double>(0, 2);
            v = calVec[i].intrinsicMatrix.at<double>(1, 1) * fTranslation.at<double>(1) / fTranslation.at<double>(2) + calVec[i].intrinsicMatrix.at<double>(1, 2);
        }
        else if(i == 2)
        {
            cv::Mat translation2P0 = (cv::Mat_<double>(3,1) << - calVec[i].tranlationMatrix.at<double>(0) ,0, 0);
            cv::Mat fTranslation = l2c + translation2P0;

            u = calVec[i].intrinsicMatrix.at<double>(0, 0) * fTranslation.at<double>(0) / fTranslation.at<double>(2) + calVec[i].intrinsicMatrix.at<double>(0, 2);
            v = calVec[i].intrinsicMatrix.at<double>(1, 1) * fTranslation.at<double>(1) / fTranslation.at<double>(2) + calVec[i].intrinsicMatrix.at<double>(1, 2);
            // exit(1)
        }
        
        else
        {
            u = calVec[i].intrinsicMatrix.at<double>(0, 0) * l2c.at<double>(0) / l2c.at<double>(2) + calVec[i].intrinsicMatrix.at<double>(0, 2);
            v = calVec[i].intrinsicMatrix.at<double>(1, 1) * l2c.at<double>(1) / l2c.at<double>(2) + calVec[i].intrinsicMatrix.at<double>(1, 2);
        }
        
        if ((0.f <= u) && (u < imgCols) && (0.f <= v) && (v < imgRows) && (0.f < l2c.at<float>(2)) && (v * imgCols + u < imgCols * imgRows))
        {
            int colIdx;
            colIdx = sqrt(l2c.at<double>(0) * l2c.at<double>(0) + l2c.at<double>(1) * l2c.at<double>(1) + l2c.at<double>(2) * l2c.at<double>(2)) * 2;

            int b = colorMap[colIdx][0] * 255;
            int g = colorMap[colIdx][1] * 255;
            int r = colorMap[colIdx][2] * 255;
            
            cv::circle(projectionImg[i], cv::Point(u, v), 3, cv::Scalar(b, g, r), 0.1);
        }
    }
}

int KITTI::makeColorMap(std::vector<std::string> &vstrPointCloudLiDAR, const std::vector<calibDataSt> &calVec, const std::vector<std::string> &vstrCam2, std::string &vstrPose)
{
    int idx = 0;
    int saveIdx = 0;

    std::ifstream fpose(vstrPose);
    if (!fpose.is_open())
    {
        std::cerr << std::endl
                << "Failed to load path \n";
        return -1;
    }

    while (true)
    {
        tqdm bar;
        bar.progress(idx, vstrPointCloudLiDAR.size());
        cv::Mat p2Img = cv::imread(vstrCam2[idx]);
        int imgCols = p2Img.cols;
        int imgRows = p2Img.rows;


        std::ifstream fcloud(vstrPointCloudLiDAR[idx], std::ios::binary);
        fcloud.seekg(0, std::ios::end);
        size_t cloudSize = fcloud.tellg();
        fcloud.seekg(0, std::ios::beg);

        std::string pose;
        std::stringstream ss;

        float p00, p01, p02, p03;
        float p10, p11, p12, p13;
        float p20, p21, p22, p23;

        getline(fpose, pose);

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

        cv::Mat poseRT = (cv::Mat_<double>(3, 4) << p00, p01, p02, p03, p10, p11, p12, p13, p20, p21, p22, p23);


        typedef struct points_s
        {
            float x, y, z, intensity;
        } points_t;

        points_t point;
        pcl::PointCloud<pcl::PointXYZI>::Ptr binCloud(new pcl::PointCloud<pcl::PointXYZI>());

        for (size_t i = 0; i < cloudSize / sizeof(point); ++i)
        {
            fcloud.read((char *)&point, sizeof(point));

            cv::Mat lidarPoints = (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1);
            cv::Mat L2ImgPixel;

            cv::Mat l2c = calVec[4].transformMatrix * lidarPoints;
            cv::Mat translation2P2 = (cv::Mat_<double>(3,1) << - calVec[2].tranlationMatrix.at<double>(0) ,0, 0);
            l2c = l2c + translation2P2;

            // 뒤에있는 z값 처리
            if (l2c.at<double>(2) <= 0)
            {
                continue;
            }

            double u  = calVec[2].intrinsicMatrix.at<double>(0, 0) * l2c.at<double>(0) / l2c.at<double>(2) + calVec[2].intrinsicMatrix.at<double>(0, 2);;
            double v  = calVec[2].intrinsicMatrix.at<double>(1, 1) * l2c.at<double>(1) / l2c.at<double>(2) + calVec[2].intrinsicMatrix.at<double>(1, 2);;

            if ((0.f <= u) && (u < imgCols) && (0.f <= v) && (v < imgRows) && (0.f < l2c.at<float>(2)) && (v * imgCols + u < imgCols * imgRows))
            {

                pcl::PointXYZRGBI pt;                
                cv::Mat l2c2 = (cv::Mat_<double>(4, 1)<< l2c.at<double>(0) , l2c.at<double>(1) ,l2c.at<double>(2), 1);



                cv::Vec3b camColor = p2Img.at<cv::Vec3b>(cv::Point(u, v));

                cv::Mat c2w = poseRT * l2c2;

                pt.x = c2w.at<double>(0);
                pt.y = c2w.at<double>(1);
                pt.z = c2w.at<double>(2);
                pt.intensity = point.intensity;
                pt.b = camColor[0]; 
                pt.g = camColor[1];
                pt.r = camColor[2];

                colorMap_->push_back(pt);
            }
            
            
        }
        fcloud.close();
        std::string savePath = "KITTI_colormap_p2Cam_" + std::to_string(saveIdx);

        if (idx%100 == 0 && idx !=0)
        {
            pcl::io::savePCDFileBinary("../kittiMap/KITTI_colormap_p2Cam_" + seqNum_+"_" + std::to_string(saveIdx) +".pcd", *colorMap_);
            saveIdx++;
            colorMap_->clear();
        }
        idx++;        

        if (idx == vstrPointCloudLiDAR.size())
        {
            pcl::io::savePCDFileBinary("../kittiMap/KITTI_colormap_p2Cam_" + seqNum_+"_" + std::to_string(saveIdx) +".pcd", *colorMap_);
            std::cout << "*****************************" << std::endl;
            std::cout << "*                           *" << std::endl;
            std::cout << "*       Save color map      *" << std::endl;
            std::cout << "*                           *" << std::endl;
            std::cout << "*****************************" << std::endl;
            exit(1);
        }
    }
  
}