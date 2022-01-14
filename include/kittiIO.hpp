#include "common.hpp"

class kittiIO
{
private:
    /* data */

public:
    kittiIO(std::string &path);
    ~kittiIO();

    int readOdometry(std::string &path, std::vector<std::string> &vecPath);

    void loadData(const std::string &strPathToSequence, std::vector<std::string> &vstrImageRGB,
                  std::vector<std::string> &vstrPointCloudLiDAR, std::string &vstrPose, std::string &vstrCalib, std::vector<double> &vTimestamps);
                  
    int poseTransform(std::string &path);
    
    int makeMap(std::string &vstrCalib, std::string &vstrPose, std::vector<std::string> &vstrPointCloudLiDAR,
                std::vector<std::string> &vstrImageRGB);
};
