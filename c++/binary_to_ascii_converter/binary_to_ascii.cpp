#include <iostream>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

using namespace std;

int
main (int argc, char** argv)
{
    pcl::PCLPointCloud2 cloud;
    Eigen::Vector4f origin; Eigen::Quaternionf orientation;
    pcl::PCDWriter writer;
    for (int i=1; i<argc; i++){
        if (pcl::io::loadPCDFile (string (argv[i]), cloud, origin, orientation) < 0)
        {
            std::cerr << "Unable to load file " << argv[i] << std::endl;
            return 0;
        }
        std::cout << "Converting file " << argv[i] << " to ASCII." << std::endl;
        writer.writeASCII (string (argv[i]), cloud, origin, orientation);
    }
}

