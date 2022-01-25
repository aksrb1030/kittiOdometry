#include <kittiOdom.hpp>



int main(int argc, char const *argv[])
{

    std::string inputPath;
    std::string inputSeqNum;
    if (argc == 3 )
    {
        inputPath= argv[1];
        inputSeqNum= argv[2];    
    }
    else
    {
        std::cout << "\n" <<"Check your input!!"<<"\n";
        std::cout << "Ex) ./kittiOdometry <Kitti path> <sequence Number> "<< "\n" <<"\n";
        return -1;
    }
    KITTI kitti(inputPath, inputSeqNum);
}

/*
    ./kittiOdometry '/home/mobiltech/Desktop/Data-ssd/kitti'
*/
