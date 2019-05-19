#include <iostream>
#include <vector>

// Include point cloud library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

// Include tinkerforge IMU2.0
#include "../Tinkerforge_IMU2.0/ip_connection.h"
#include "../Tinkerforge_IMU2.0/brick_imu_v2.h"

// Set up for imu
#define HOST "localhost"
#define PORT 4223
#define UID "64tUkb" // Change XXYYZZ to the UID of your IMU Brick 2.0

// Point Type
// pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGBA
typedef pcl::PointXYZI PointType;

void
print_help (const char* prog_name)
{
  std::cout << "\n\nUsage: "<<prog_name<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------------------------------------------\n"
            << "-h             This help message\n"
            << "-p <*.pcap>    Velodyne capture from pcap file\n"
            << "-s             Velodyne capture from sensor\n"
            << "-f             Use space to start and stop writing pcds\n"
            << "Default:       Use space to capture frame and write one pcd\n"
            << "-d <dir_name>  Directory name of where to save the pcds\n"
            << "\n\n";
}


int main( int argc, const char *const *argv )
{
    // -------------------------------------------
    // -------COMMAND-LINE ARGUMENT PARSING-------
    // -------------------------------------------

    bool read_from_pcap(false), read_from_sensor(false), write_with_space(false), start_stop_with_space(false);

    if( pcl::console::find_switch( argc, argv, "-h" ) ){
        print_help ( argv[0] );
        return 0;
    }
    if( pcl::console::find_switch( argc, argv, "-p" ) ){
        read_from_pcap = true;
    }
    else if( pcl::console::find_switch( argc, argv, "-s" ) ) {
        read_from_sensor = true;
    }
    if( !pcl::console::find_switch( argc, argv, "-f" ) ){
        write_with_space = true;
    }
    if( !pcl::console::find_switch( argc, argv, "-d" ) ){
        cout << "\033[1;31mSpecify directory name\033[0m\n";
        return 0;
    }

    std::string ipaddress( "192.168.1.201" );
    std::string port( "2368" );
    std::string pcap;
    std::string directory;
    std::string path;
    std::ofstream quaternion;
    std::ofstream imu_data;
    std::vector <std::string> pcds;

    pcl::console::parse_argument( argc, argv, "-p", pcap );
    pcl::console::parse_argument( argc, argv, "-d", directory );

    std::string usage =  "Usage:\n"
            "--------------------------------------------------------\n"
            "Press n to capture fragments (use space to write pcds)\n"
            "Press m to capture odometry (use space to write pcds)\n"
            "\n\n";


    // -----------------------------------
    // -------SET UP IMU CONNECTION-------
    // -----------------------------------

    // Create IP connection
    IPConnection ipcon{};
    ipcon_create(&ipcon);

    // Create device object
    IMUV2 imu{};
    imu_v2_create(&imu, UID, &ipcon);

    // Connect to brickd
    if(ipcon_connect(&ipcon, HOST, PORT) < 0) {
        std::cerr << "Could not connect\n";
        return 1; // Don't use device before ipcon is connected
    }

    // ----------------------------------
    // --------SET UP POINT CLOUD--------
    // ----------------------------------

    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;


    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) );
    viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
    viewer->initCameraParameters();
    viewer->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 );

    // Point Cloud Color Handler
    pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;
    const std::type_info& type = typeid( PointType );
    if( type == typeid( pcl::PointXYZ ) ){
        std::vector<double> color = { 255.0, 255.0, 255.0 };
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerCustom<PointType>( color[0], color[1], color[2] ) );
        handler = color_handler;
    }
    else if( type == typeid( pcl::PointXYZI ) ){
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<PointType>( "intensity" ) );
        handler = color_handler;
    }
    else if( type == typeid( pcl::PointXYZRGBA ) ){
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerRGBField<PointType>() );
        handler = color_handler;
    }
    else{
        throw std::runtime_error( "This PointType is unsupported." );
    }

    int max_num = 0;
    int number = 0;
    // Retrieved Point Cloud Callback Function

    boost::mutex mutex;
    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
            [ &cloud, &mutex, &quaternion, &imu, &write_with_space, &start_stop_with_space, &max_num, &path, &number, &pcds, &imu_data]
                    ( const pcl::PointCloud<PointType>::ConstPtr& ptr ) { boost::mutex::scoped_lock lock(mutex);
                /* Point Cloud Processing */

                size_t num_ptr = ptr->size();
                if (num_ptr > max_num){
                    max_num = static_cast<int>(num_ptr);
                }
                if (num_ptr < 0.7*max_num) {
                    return;
                }
                cloud = ptr;

                if(start_stop_with_space && !path.empty()){

                    std::ostringstream filename;
                    filename << "scan" << std::to_string(number++) << ".pcd";
                    std::cout << "\033[1;32mSaved point cloud "<< std::to_string(number) <<" -> " << num_ptr << " points\033[0m" <<std::endl;
                    // pcl::io::savePCDFileASCII("../pcd_file/" + filename.str(), *cloud);
                    pcl::io::savePCDFileBinary(path + filename.str(), *cloud);

                    // Vector with path and filename of all pcds
                    pcds.push_back(path + filename.str());

                    // Get quaternions from imu
                    int16_t q_w, q_x, q_y, q_z ;
                    if(imu_v2_get_quaternion(&imu, &q_w, &q_x, &q_y, &q_z) < 0) {
                        std::cerr << "Could not get quaternion, probably timeout" << "\n";
                    }

                    // Write quaternion data to csv file
                    quaternion.open(path + "quaternions_fragment.csv", std::ios_base::app);
                    quaternion << q_w / 16383.0 << "," << q_x / 16383.0 << "," << q_y / 16383.0 << "," << q_z / 16383.0 << "\n";
                    quaternion.close();

                    // Get angular velocity from imu
                    int16_t angv_x, angv_y, angv_z;
                    if(imu_v2_get_angular_velocity(&imu, &angv_x, &angv_y, &angv_z) < 0) {
                        std::cerr << "Could not get angular velocity, probably timeout" << "\n";
                    }
                    // Get linear acceleration from imu
                    int16_t lina_x, lina_y, lina_z;
                    if(imu_v2_get_linear_acceleration(&imu, &lina_x, &lina_y, &lina_z) < 0){
                        std::cerr << "Could not get linear acceleration, probably timeout" << "\n";
                    }
                    // Write angular velocity and linear acceleration to file
                    imu_data.open(path + "imu_data.csv", std::ios_base::app);
                    imu_data << angv_x / 16.0 << "," << angv_y / 16.0 << "," << angv_z / 16.0 << ","
                             << lina_x / 100.0 << "," << lina_y / 100.0 << "," << lina_z / 100.0 <<"\n";
                    imu_data.close();

                    if(write_with_space){
                        start_stop_with_space = !start_stop_with_space;
                    }

                }
            };

    // VLP Grabber
    boost::shared_ptr<pcl::VLPGrabber> grabber;

    if( read_from_pcap && !pcap.empty() ){
        std::cout << "Capture from PCAP..." << std::endl;
        std::cout << "pcap file: " << pcap << std::endl;
        std::cout << "\n\n";
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( pcap ) );

    }
    else if( read_from_sensor && !ipaddress.empty()  ){
        std::cout << "Capture from Sensor..." << std::endl;
        std::cout << "ipadress : " << ipaddress << std::endl;
        std::cout << "port : " << port << std::endl;
        std::cout << "\n\n";
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( boost::asio::ip::address::from_string( ipaddress ), boost::lexical_cast<unsigned short>( port ) ) );
    }
    else{
        print_help ( argv[0] );
        return 0;
    }
    std::cout << usage;
    // Keyboard Callback Function
    boost::function<void( const pcl::visualization::KeyboardEvent& )> keyboard_function =
            [&cloud, &mutex, &start_stop_with_space, &directory, &path, &quaternion, &number, &imu_data]
                    ( const pcl::visualization::KeyboardEvent& event ){
                // Callback when pressing spacebar
                if (event.getKeyCode () == ' ' && event.keyDown ()){
                    start_stop_with_space = !start_stop_with_space;
                }
                static unsigned int n = 0;
                if (event.getKeyCode () == 'n' && event.keyDown ()){
                    number = 0;
                    std::cout << "Writing to: fragment" << std::to_string(n) <<"\n\n";
                    path = "../../../data/" + directory + "/fragments/fragment" + std::to_string(n++) + "/";
                    boost::filesystem::create_directories(path);
                    // Open and write header to quaternion csv file
                    quaternion.open(path + "quaternions_fragment.csv");
                    quaternion << "q_w,q_x,q_y,q_z\n";
                    quaternion.close();

                    // Open and write header to imu data csv file
                    imu_data.open(path + "imu_data.csv");
                    imu_data << "angv_x,angv_y,angv_z,lina_x,lina_y,lina_z\n";
                    imu_data.close();
                }
                static unsigned int m = 0;
                if (event.getKeyCode () == 'm' && event.keyDown ()){
                    number = 0;
                    std::cout << "Writing to: odometry" << std::to_string(m) <<"\n\n";
                    path = "../../../data/" + directory + "/odometry/odometry" + std::to_string(m++) + "/";
                    boost::filesystem::create_directories(path);

                    // Open and write header to quaternion csv file
                    quaternion.open(path + "quaternions_fragment.csv");
                    quaternion << "q_w,q_x,q_y,q_z\n";
                    quaternion.close();

                    // Open and write header to imu data csv file
                    imu_data.open(path + "imu_data.csv");
                    imu_data << "angv_x,angv_y,angv_z,lina_x,lina_y,lina_z\n";
                    imu_data.close();
                }
            };


    // Register Callback Function
    viewer->registerKeyboardCallback( keyboard_function );


    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );


    // Start Grabber
    grabber->start();

    while( !viewer->wasStopped() ){
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock(mutex);
        if (lock.owns_lock() && cloud) {
            // Update Point Cloud
            handler->setInputCloud(cloud);
            if (!viewer->updatePointCloud(cloud, *handler, "cloud")) {
                viewer->addPointCloud(cloud, *handler, "cloud");

            }
        }
    }
    // Stop Grabber
    grabber->stop();

    // Disconnect Callback Function
    if( connection.connected() ){
        connection.disconnect();
        imu_v2_destroy(&imu);
        ipcon_destroy(&ipcon); // Calls ipcon_disconnect internally

        pcl::PCLPointCloud2 clouds;
        Eigen::Vector4f origin; Eigen::Quaternionf orientation;
        pcl::PCDWriter writer;
        std::cout << "Converting binary pcds to ASCII..." << std::endl;
        for (const auto &pcd : pcds) {
            pcl::io::loadPCDFile(pcd, clouds, origin, orientation);
            writer.writeASCII(pcd, clouds, origin, orientation);
        }
    }
    return 0;
}
