cmake_minimum_required(VERSION 3.9 FATAL_ERROR)

project(interpolation_vlp)

set(CMAKE_CXX_STANDARD 11)


add_executable (interpolation_vlp
                ../Tinkerforge_IMU2.0/brick_imu_v2.cpp
                ../Tinkerforge_IMU2.0/brick_imu_v2.h
                ../Tinkerforge_IMU2.0/ip_connection.cpp
                ../Tinkerforge_IMU2.0/ip_connection.h
                interpolation_vlp.cpp)


# Find Package Thread
find_package( Threads REQUIRED )

# Find Package Boost
set( BOOST_ROOT "/usr/local/Cellar/boost/" )
find_package( Boost COMPONENTS system date_time regex )

# If Direct Capture from Sensors, VelodyneCapture are required Boost.Asio.
set( HAVE_BOOST )
if( Boost_FOUND )
    set( HAVE_BOOST "-DHAVE_BOOST" )
endif()

# Find Package PCAP
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )
set( PCAP_DIR "/usr/local/Cellar/libpcap" )
find_package( PCAP )

# If Capture from PCAP Files, VelodyneCapture are required PCAP.
set( HAVE_PCAP )
if( PCAP_FOUND )
    set( HAVE_PCAP "-DHAVE_PCAP" )
endif()

# Find Package OpenCV
set( OpenCV_DIR "/usr/local" )
set( OpenCV_STATIC OFF )
find_package( OpenCV REQUIRED )

# Find Package PCL
find_package(PCL 1.9 REQUIRED )

# Set Properties
if( PCAP_FOUND OR Boost_FOUND OR PCL_FOUND )
    # Additional Include Directories
    include_directories( ${Boost_INCLUDE_DIRS} )
    include_directories( ${PCAP_INCLUDE_DIRS} )
    include_directories(${PCL_INCLUDE_DIRS})

    # Add Definitions
    add_definitions( ${Boost_LIB_DIAGNOSTIC_DEFINITIONS} )
    add_definitions( ${HAVE_BOOST} )
    add_definitions( ${HAVE_PCAP} )
    add_definitions( ${PCL_DEFINITIONS} )

    # Other Definitions
    add_definitions( -DNO_EMPTY_RETURNS )  # Don't push null returns to the lases queue
  #  add_definitions( -DHAVE_GPSTIME )  # Use GPS timestamps insead of Unix epoch
#    add_definitions( -DUSE_MILLIMETERS )   # Use millimeters instead of the default, centimeters.
    add_definitions( -DPUSH_SINGLE_PACKETS )   # Push data packets as soon as possible
  #  add_definitions( -DHAVE_FAST_PCAP )    # Disable sleep while reading a pcap

    # Additional Library Directories
    link_directories( ${Boost_LIBRARY_DIRS} )
    link_directories( ${OpenCV_LIB_DIR} )
    link_directories( ${PCL_LIBRARY_DIRS} )

    # Additional Dependencies
    target_link_libraries( interpolation_vlp ${CMAKE_THREAD_LIBS_INIT} )
    target_link_libraries( interpolation_vlp ${Boost_LIBRARIES} )
    target_link_libraries( interpolation_vlp ${PCAP_LIBRARIES} )
    target_link_libraries( interpolation_vlp ${OpenCV_LIBS} )
    target_link_libraries(interpolation_vlp ${PCL_LIBRARIES})

else()
    message( WARNING "VelodyneCapture need at least either Boost or PCAP." )
endif()