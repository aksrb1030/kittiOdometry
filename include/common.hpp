#define PCL_NO_PRECOMPILE
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <tqdm.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv/cv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <liblas/liblas.hpp>

namespace pcl
{

    struct PointXYZRGBI
    {
        PCL_ADD_POINT4D;
        PCL_ADD_RGB;
        PCL_ADD_INTENSITY;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

}

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBI,
                                  (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(float, intensity, intensity))