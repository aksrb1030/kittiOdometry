#include "common.hpp"

class KITTI
{

    
private:
    /* data */
    using Eigen3x3d = Eigen::Matrix<double,3,3>;
    using Eigen3x1d = Eigen::Matrix<double,3,1>;
    using Eigen4x1d = Eigen::Matrix<double,4,1>;
    using Eigen3x4d = Eigen::Matrix<double,3,4>;

    struct calibDataSt
    {
        std::string naming;
        cv::Mat intrinsicMatrix;
        cv::Mat rotationMatrix;
        cv::Mat tranlationMatrix;
        cv::Mat transformMatrix;
    };
    std::string seqNum_;

    pcl::PointCloud<pcl::PointXYZRGBI>::Ptr colorMap_;
    // pcl::PointCloud<pcl::PointXYZRGBI>::Ptr intensityMap_;


    double colorMap[64][3] = {
            {0,	0,	0.562500000000000},
            {0,	0,	0.625000000000000},
            {0,	0,	0.687500000000000},
            {0,	0,	0.750000000000000},
            {0,	0,	0.812500000000000},
            {0,	0,	0.875000000000000},
            {0,	0,	0.937500000000000},
            {0,	0,	1},
            {0,	0.0625000000000000,	1},
            {0,	0.125000000000000,	1},
            {0,	0.187500000000000,	1},
            {0,	0.250000000000000,	1},
            {0,	0.312500000000000,	1},
            {0,	0.375000000000000,	1},
            {0,	0.437500000000000,	1},
            {0,	0.500000000000000,	1},
            {0,	0.562500000000000,	1},
            {0,	0.625000000000000,	1},
            {0,	0.687500000000000,	1},
            {0,	0.750000000000000,	1},
            {0,	0.812500000000000,	1},
            {0,	0.875000000000000,	1},
            {0,	0.937500000000000,	1},
            {0,	1,	1},
            {0.0625000000000000,	1,	0.937500000000000},
            {0.125000000000000,	1,	0.875000000000000},
            {0.187500000000000,	1,	0.812500000000000},
            {0.250000000000000,	1,	0.750000000000000},
            {0.312500000000000,	1,	0.687500000000000},
            {0.375000000000000,	1,	0.625000000000000},
            {0.437500000000000,	1,	0.562500000000000},
            {0.500000000000000,	1,	0.500000000000000},
            {0.562500000000000,	1,	0.437500000000000},
            {0.625000000000000,	1,	0.375000000000000},
            {0.687500000000000,	1,	0.312500000000000},
            {0.750000000000000,	1,	0.250000000000000},
            {0.812500000000000,	1,	0.187500000000000},
            {0.875000000000000,	1,	0.125000000000000},
            {0.937500000000000,	1,	0.0625000000000000},
            {1,	1,	0},
            {1,	0.937500000000000,	0},
            {1,	0.875000000000000,	0},
            {1,	0.812500000000000,	0},
            {1,	0.750000000000000,	0},
            {1,	0.687500000000000,	0},
            {1,	0.625000000000000,	0},
            {1,	0.562500000000000,	0},
            {1,	0.500000000000000,	0},
            {1,	0.437500000000000,	0},
            {1,	0.375000000000000,	0},
            {1,	0.312500000000000,	0},
            {1,	0.250000000000000,	0},
            {1,	0.187500000000000,	0},
            {1,	0.125000000000000,	0},
            {1,	0.0625000000000000,	0},
            {1,	0,	0},
            {0.937500000000000,	0,	0},
            {0.875000000000000,	0,	0},
            {0.812500000000000,	0,	0},
            {0.750000000000000,	0,	0},
            {0.687500000000000,	0,	0},
            {0.625000000000000,	0,	0},
            {0.562500000000000,	0,	0},
            {0.500000000000000,	0,	0}
        };

public:
    KITTI(std::string &path, std::string &seqNum);
    ~KITTI();

    int loadData(const std::string &strPathToSequence, std::vector<std::string> &vstrCam0, std::vector<std::string> &vstrCam1 ,std::vector<std::string> &vstrCam2,std::vector<std::string> &vstrCam3,
                  std::vector<std::string> &vstrPointCloudLiDAR, std::string &vstrPose, std::string &vstrCalib, std::vector<double> &vTimestamps);
    int projection(std::string &vstrCalib, std::string &vstrPose, std::vector<std::string> &vstrPointCloudLiDAR,
                std::vector<std::string> &vstrCam0, std::vector<std::string> &vstrCam1, std::vector<std::string> &vstrCam2, std::vector<std::string> &vstrCam3);
    int makeColorMap(std::vector<std::string> &vstrPointCloudLiDAR, const std::vector<calibDataSt> &calVec, const std::vector<std::string> &vstrCam2, std::string &vstrPose);
    void tokeNize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters);
    void img2LProection(cv::Mat &l2c, const std::vector<calibDataSt> &calVec, const std::vector<cv::Mat> &kittiImgVec, std::vector<cv::Mat> &projectionImg);
};