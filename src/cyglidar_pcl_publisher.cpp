#include <cyglidar_pcl.h>
#include "PointCloudMaker.h"
#include "Constants_CygLiDAR_D1.h"
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>


int RunMode;        // select run mode {2D(0), 3D(1), Dual(2)}
int setAutoDuration; // select Auto (1) or Fixed(0)
int DATABUFFER_SIZE_2D, DATABUFFER_SIZE_3D, DATASET_SIZE_2D, DATASET_SIZE_3D;

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointXYZRGBA;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan_2D;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan_3D;
sensor_msgs::msg::LaserScan::Ptr scan_laser;
//ros::Publisher pub_2D, pub_3D, pub_scan;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_3D;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_2D;
rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan;

rclcpp::Node::SharedPtr node;

// 3D point cloud 
uint16_t DistanceBuffer[CygLiDARD1::Sensor::numPixel];
float param_x[CygLiDARD1::Sensor::numPixel];
float param_y[CygLiDARD1::Sensor::numPixel];
float param_z[CygLiDARD1::Sensor::numPixel];
PointCloudMaker PointCloud3D(param_x, param_y, param_z, CygLiDARD1::Sensor::numPixel);

uint8_t *cloudBuffer;
float *cloudSetBuffer;
uint8_t *cloudBuffer_2D;
float *cloudSetBuffer_2D;

uint8_t* bufferPtr_2D;
uint8_t* bufferPtr01;
uint8_t* bufferPtr02;
uint8_t* bufferPtr;

uint8_t currentBuffer;

static std::vector<uint32_t> colorArray;
void colorBuffer()
{
	uint8_t r_setup = 255;
	uint8_t g_setup = 0;
	uint8_t b_setup = 0;

	// Iterate for-loop of adding RGB value to an array
	for (int i = 0; i < 3; i++)
	{
		for (int colorCnt = 0; colorCnt < 256; colorCnt++)
		{
			switch (i)
			{
				// RED -> YELLOW
			case 0:
				g_setup++;
				break;
				// YELLOW -> BLUE
			case 1:
				r_setup--;
				g_setup--;
				b_setup++;
				break;
				// BLUE -> RED
			case 2:
				r_setup++;
				b_setup--;
				break;
			}

			uint32_t rgb_setup = ((uint32_t)r_setup << 16 | (uint32_t)g_setup << 8 | (uint32_t)b_setup);
			colorArray.push_back(rgb_setup);
		}
	}
}

bool drawing = false;
float tempX_2D, tempY_2D;
void cloudScatter_2D(rclcpp::Time start, rclcpp::Time end, const double ANGLE_STEP_2D)
{
    if (!drawing)
    {
        // Make the drawing state TRUE so that it's not interrupted while drawing
        drawing = true;

        // Copy the memory of the rawdata array to the working array
        memcpy(cloudBuffer_2D, &bufferPtr_2D[PAYLOAD_SIZE], sizeof(uint8_t) * DATABUFFER_SIZE_2D);

        int distanceCnt_2D = 0;

        // Convert rawdata to measured distance appending both separated data (LSB and MSB)
        for (int dataLength = 0; dataLength < (DATABUFFER_SIZE_2D - PAYLOAD_SIZE) - 1; dataLength += 2)
        {
            uint8_t MSB = cloudBuffer_2D[dataLength];
            uint8_t LSB = cloudBuffer_2D[dataLength + 1];

            float rawDistance = (float)((MSB << 8) | LSB);

            cloudSetBuffer_2D[distanceCnt_2D++] = rawDistance;
        }

        // Update values of LaserScan
        double scan_duration = (start - end).seconds() * 1e-3;

        scan_laser->header.stamp = start;
        scan_laser->scan_time = scan_duration;
        scan_laser->time_increment = (scan_duration / (float)(DATASET_SIZE_2D - 1));

        // Initialize the current point of FoV at which a given distance is measured
        float point_angle_var_2D = 0.0;

        // Loop for drawing distances
        for (int idx = 0; idx < DATASET_SIZE_2D; idx++)
        {
            // Reverse data order of the array
            int data_idx = (DATASET_SIZE_2D - 1 - idx);
            float actualDistance = (cloudSetBuffer_2D[data_idx]);

            // Get the latest angle of the distance
            float point_angle_2D = (float)(((-HORIZONTAL_ANGLE / 2)+ point_angle_var_2D) * CygLiDARD1::Util::ToRadian);
            point_angle_var_2D += ANGLE_STEP_2D;

            // Calculate both X and Y coordinates
            float actualX = (sin(point_angle_2D) * actualDistance);
            float actualY = (cos(point_angle_2D) * actualDistance); // depth

            // Rotate the dataset 90 clockwise about the origin
            tempX_2D = actualX;
            tempY_2D = actualY;
            float RotationAngle = -90.0f * CygLiDARD1::Util::ToRadian;
            actualX = (tempX_2D * cos(RotationAngle)) + (tempY_2D * -sin(RotationAngle));
            actualY = (tempX_2D * sin(RotationAngle)) + (tempY_2D * cos(RotationAngle));

            // Store the computed coordinates to PointCloud2 and LaserScan
            scan_2D.get()->points[idx].x = actualX * MM2M;
            scan_2D.get()->points[idx].y = -actualY * MM2M;
            scan_2D.get()->points[idx].z = 0.0;

            // Turn data invisible when it's greater than the maximum
            if (cloudSetBuffer_2D[data_idx] < (float)(CygLiDARD1::Distance::Mode2D::Maximum_Depth_2D))
            {
                scan_2D.get()->points[idx].r = 255;
                scan_2D.get()->points[idx].g = 255;
                scan_2D.get()->points[idx].b = 0;
                scan_2D.get()->points[idx].a = 255;
                scan_laser->ranges[idx] = cloudSetBuffer_2D[data_idx] * MM2M;
            }
            else
            {
                scan_2D.get()->points[idx].a = 0;
                scan_laser->ranges[idx] = std::numeric_limits<float>::infinity();
            }
        }

        pcl_conversions::toPCL(node->now(), scan_2D->header.stamp);
        drawing = false;
        // Allow the latest dataset to enter in this constructor
    }
}

void cloudScatter_3D()
{
    if (!drawing)
    {
        // Make the drawing state TRUE so that it's not interrupted while drawing
        drawing = true;
        
        // Copy the memory of the rawdata array to the working array
        switch (currentBuffer)
        {
            case 0x00:
                cloudBuffer = &bufferPtr02[0];      // ref. to output buffer
                currentBuffer = 0x01;               // buffer swap
                break;
            case 0x01:
                cloudBuffer = &bufferPtr01[0];
                currentBuffer = 0x00;
                break;
        }

        // Convert rawdata to measured distance appending both separated data (LSB and MSB)
        int distanceCnt_3D = 0;
        for (int dataLength = 0; dataLength < (DATABUFFER_SIZE_3D - PAYLOAD_SIZE) - 1; dataLength+=3)
        {
            uint8_t FIRST = cloudBuffer[dataLength];
            uint8_t SECOND = cloudBuffer[dataLength + 1];
            uint8_t THIRD = cloudBuffer[dataLength + 2];

            // data1 is a combination of the first and left half of second
            // data2 is of the right half of second and third
            uint16_t data1 = (uint16_t)((FIRST << 4) | (SECOND >> 4));
            uint16_t data2 = (uint16_t)(((SECOND & 0xf) << 8) | THIRD);

            DistanceBuffer[distanceCnt_3D++] = data1;
            DistanceBuffer[distanceCnt_3D++] = data2;
        }

        int BufferIndex = 0;
        float pos_x, pos_y, pos_z;
      
  	    for (int y = 0; y < CygLiDARD1::Sensor::Height; y++)
  	    {
      		for (int x = 0; x < CygLiDARD1::Sensor::Width; x++)
  		    {
                  BufferIndex = x + (CygLiDARD1::Sensor::Width * y);
                  uint16_t distance = DistanceBuffer[BufferIndex];
                  if (distance > CygLiDARD1::Distance::Mode3D::Maximum_Depth_3D)  
                      distance = CygLiDARD1::Distance::Mode3D::Maximum_Depth_3D;
                  PointCloud3D.calcPointCloud(distance, BufferIndex, pos_x, pos_y, pos_z);
                  if  ( (y == 0) || (y == CygLiDARD1::Sensor::Height-1) || (x==0) || (x==CygLiDARD1::Sensor::Width-1) )
                  {
                    scan_3D.get()->points[BufferIndex].x = std::numeric_limits<float>::quiet_NaN();
                    scan_3D.get()->points[BufferIndex].y = std::numeric_limits<float>::quiet_NaN();
                    scan_3D.get()->points[BufferIndex].z = std::numeric_limits<float>::quiet_NaN();
                    scan_3D.get()->points[BufferIndex].rgb = 0.0;
                    scan_3D.get()->points[BufferIndex].a = 0;
                  }
                  else
                  {
                    scan_3D.get()->points[BufferIndex].x = pos_z * MM2M;
                    scan_3D.get()->points[BufferIndex].y = -pos_x * MM2M;
                    scan_3D.get()->points[BufferIndex].z = -pos_y * MM2M;
                    uint32_t rgb_3D = colorArray[((int)pos_y / 2) % colorArray.size()];
                    scan_3D.get()->points[BufferIndex].rgb = *reinterpret_cast<float*>(&rgb_3D);
                    scan_3D.get()->points[BufferIndex].a = 255;
                  }
   		    }
  	    }

        pcl_conversions::toPCL(node->now(), scan_3D->header.stamp);
        // Allow the latest dataset to enter in this constructor
        drawing = false;
    }
}

int FREQUENCY_LEVEL, PULSE_DURATION;
void running()
{
    // Create node handlers and local variables
    node = rclcpp::Node::make_shared("line_laser");

    node->declare_parameter("port", std::string("/dev/ttyUSB0"));
    node->declare_parameter("baud_rate", 3000000);
    node->declare_parameter("frame_id", std::string("laser_link"));
    node->declare_parameter("run_mode", 2);
    node->declare_parameter("frequency", 0);
    node->declare_parameter("set_auto_duration", 0);
    node->declare_parameter("duration", 10000);


    rclcpp::Parameter port_param = node->get_parameter("port");
    rclcpp::Parameter baud_rate_param = node->get_parameter("baud_rate");
    rclcpp::Parameter frame_id_param = node->get_parameter("frame_id");
    rclcpp::Parameter RunMode_param = node->get_parameter("run_mode");
    rclcpp::Parameter FREQUENCY_LEVEL_param = node->get_parameter("frequency");
    rclcpp::Parameter setAutoDuration_param = node->get_parameter("set_auto_duration");
    rclcpp::Parameter PULSE_DURATION_param = node->get_parameter("duration");

    std::string port = port_param.as_string();
    int baud_rate = baud_rate_param.as_int();
    std::string frame_id = frame_id_param.as_string();
    int RunMode = RunMode_param.as_int();
    int FREQUENCY_LEVEL = FREQUENCY_LEVEL_param.as_int();
    int setAutoDuration = setAutoDuration_param.as_int();
    int PULSE_DURATION = PULSE_DURATION_param.as_int();    

    // Assign the given values by users
    

    // injection run mode
    eRunMode LiDAR_RunMode;
    switch(RunMode)
    {
        case 0:
        LiDAR_RunMode = eRunMode::Mode2D;
        break;
        case 1:
        LiDAR_RunMode = eRunMode::Mode3D;
        break;
        case 2:
        LiDAR_RunMode = eRunMode::ModeDual;
        break;
        default:
        LiDAR_RunMode = eRunMode::Mode3D;
        break;
    }

    // Check whether the values are applied properly
    ROS_INFO("FREQUENCY: %d (%x) / PULSE CONTROL: %d, DURATION: %d (%x)", \
    FREQUENCY_LEVEL, FREQUENCY_LEVEL, \
    setAutoDuration, PULSE_DURATION, PULSE_DURATION);

    // Call the following function so as to store colors to draw 3D data
    colorBuffer();

    {
        using namespace CygLiDARD1;
	    PointCloud3D.initLensTransform(Sensor::PixelRealSize, Sensor::Width, Sensor::Height, Parameter::OffsetCenterPoint_x, Parameter::OffsetCenterPoint_y);
    }

    // Initialize variables
    //pub_scan = nh.advertise<sensor_msgs::msg::LaserScan>("scan_laser", SCAN_MAX_SIZE);
    //pub_2D = nh.advertise<sensor_msgs::msg::PointCloud2>("scan_2D", 1);
    //pub_3D = nh.advertise<sensor_msgs::msg::PointCloud2>("scan_3D", 1);
    auto pub_scan = node->create_publisher<sensor_msgs::msg::LaserScan>("scan_laser", SCAN_MAX_SIZE);
    auto pub_2D = node->create_publisher<sensor_msgs::msg::PointCloud2>("scan_2D", 1);
    auto pub_3D = node->create_publisher<sensor_msgs::msg::PointCloud2>("scan_3D", 1);

    scan_2D = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    scan_3D = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    scan_laser = sensor_msgs::msg::LaserScan::Ptr(new sensor_msgs::msg::LaserScan);

    currentBuffer = 0x00;
    drawing = false;

    bufferPtr = new uint8_t[SCAN_MAX_SIZE];
    uint8_t result[SCAN_MAX_SIZE];
    uint8_t inProgress = 0x00;

    rclcpp::Time current_time_laser = node->now();
    rclcpp::Time last_time_laser = node->now();

    bool buffer_setup_2d = false;
    bool buffer_setup_3d = false;

    // LaserScan
    scan_laser->header.frame_id = frame_id;

    // PointCloud2 for 2D
    scan_2D.get()->is_dense = false;
    scan_2D.get()->header.frame_id = frame_id;

    // PointCloud2 for 3D
    scan_3D.get()->width = CygLiDARD1::Sensor::Width;
    scan_3D.get()->height = CygLiDARD1::Sensor::Height;
    scan_3D.get()->is_dense = false;
    scan_3D.get()->header.frame_id = frame_id;

    boost::asio::io_service io;
    try
    {
        // Open the port
        cyglidar_pcl laser(port, baud_rate, io);

        // Send packets
        laser.packet_run(LiDAR_RunMode);
        rclcpp::Rate r(std::chrono::seconds(1));
        r.sleep();  // sleep for a second
        laser.packet_frequency(FREQUENCY_LEVEL);
        r.sleep();  // sleep for a second
        laser.packet_duration(LiDAR_RunMode, setAutoDuration, PULSE_DURATION);

        // Create variables used to organize data before drawing
        int DATA_1 = 4, DATA_2 = 3;
        double ANGLE_STEP_2D;

        // Keep receiving data unless the process is dead
        ROS_INFO("entering service loop");
        while (rclcpp::ok())
        {
            // Only allowed in case of not using the working array called bufferPtr
            if (inProgress == 0x00)
            {
				inProgress = 0x01;

                uint16_t numOfData = laser.rvData(result, SCAN_MAX_SIZE);

                if (numOfData > 0)
                {
                    for (int i = 0; i < numOfData; i++)
                    {
                        if(CygParser(bufferPtr, result[i]) == 1)
                        {
                            switch (bufferPtr[5])
                            {
                                case 0x01: // 2D
                                    if (!buffer_setup_2d)
                                    {
                                        // init
                                        DATABUFFER_SIZE_2D = (int)(bufferPtr[DATA_1] << 8 | bufferPtr[DATA_2]) + PAYLOAD_SIZE;
                                        DATASET_SIZE_2D = (int)((float)(DATABUFFER_SIZE_2D - PAYLOAD_SIZE) / 2.0);
                                        ANGLE_STEP_2D = static_cast<double>(CygLiDARD1::Sensor::AngleIncremet2D);
                                        double angleHalf = static_cast<double>(CygLiDARD1::Sensor::HorizontalAngle / 2.0f * CygLiDARD1::Util::ToRadian);
                                        cloudBuffer_2D = new uint8_t[DATABUFFER_SIZE_2D - PAYLOAD_SIZE];
                                        cloudSetBuffer_2D = new float[DATASET_SIZE_2D];
                                        scan_laser->header.frame_id = frame_id;
                                        scan_laser->angle_min = -angleHalf;
                                        scan_laser->angle_max = angleHalf;
                                        scan_laser->angle_increment = static_cast<double>(CygLiDARD1::Sensor::AngleIncremet2D * CygLiDARD1::Util::ToRadian);
                                        scan_laser->range_min = static_cast<double>(CygLiDARD1::Distance::Mode2D::Minimum_Depth_2D * CygLiDARD1::Util::MM_To_M);
                                        scan_laser->range_max = static_cast<double>(CygLiDARD1::Distance::Mode2D::Maximum_Depth_2D * CygLiDARD1::Util::MM_To_M);
                                        scan_laser->ranges.resize(DATASET_SIZE_2D);
                                        scan_laser->intensities.resize(DATASET_SIZE_2D);
                                        scan_2D.get()->points.resize(DATASET_SIZE_2D);
                                        buffer_setup_2d = true;
                                    }
                                    if (buffer_setup_2d)
                                    {
                                        last_time_laser = node->now();
                                        bufferPtr_2D = &bufferPtr[0];
                                        sensor_msgs::msg::PointCloud2 pc2_msg_2d_;
                                        cloudScatter_2D(current_time_laser, last_time_laser, ANGLE_STEP_2D);
                                        pcl::toROSMsg(*scan_2D, pc2_msg_2d_);
                                        pub_2D->publish(pc2_msg_2d_);
                                        pub_scan->publish(*scan_laser);
                                    }
                                    break;
                                case 0x08: // 3D
                                    if (!buffer_setup_3d)
                                    {
                                        // init
                                        DATABUFFER_SIZE_3D = (int)(bufferPtr[DATA_1] << 8 | bufferPtr[DATA_2]) + PAYLOAD_SIZE;
                                        float BYTESET_RATIO_3D = (2.0 / 3.0);
                                        DATASET_SIZE_3D = (int)((float)(DATABUFFER_SIZE_3D - PAYLOAD_SIZE) * BYTESET_RATIO_3D);
                                        // cloudBuffer = new uint8_t[DATABUFFER_SIZE_3D - PAYLOAD_SIZE];
                                        bufferPtr01 = new uint8_t[DATABUFFER_SIZE_3D - PAYLOAD_SIZE];       // memory allocation to buffer A
                                        bufferPtr02 = new uint8_t[DATABUFFER_SIZE_3D - PAYLOAD_SIZE];       // memory allocation to buffer B
                                        cloudSetBuffer = new float[DATASET_SIZE_3D];
                                        scan_3D.get()->points.resize(DATASET_SIZE_3D);
                                        buffer_setup_3d = true;
                                    }
                                    if (buffer_setup_3d)
                                    {
                                        switch (currentBuffer)
                                        {
                                            case 0x00:
                                                memcpy(bufferPtr01, bufferPtr + PAYLOAD_SIZE, sizeof(uint8_t) * DATABUFFER_SIZE_3D);   // copy to buffer
                                                break;
                                            case 0x01:
                                                memcpy(bufferPtr02, bufferPtr + PAYLOAD_SIZE, sizeof(uint8_t) * DATABUFFER_SIZE_3D);
                                                break;
                                        }
                                        cloudScatter_3D();
                                        sensor_msgs::msg::PointCloud2 pc2_msg_3d_;
                                        pcl::toROSMsg(*scan_3D, pc2_msg_3d_);
                                        pub_3D->publish(pc2_msg_3d_);
                                    }
                                    break;
                                }
                            }
                            else
                            {
                                // passed through header 1
                                current_time_laser = node->now();
                            }
                    }
                }
				inProgress = 0x00;
            }
            rclcpp::spin_some(node);
        }
        ROS_INFO("closing");
        laser.close();
        delete bufferPtr01;
        delete bufferPtr02;
        delete cloudSetBuffer;
        delete cloudBuffer_2D;
        delete cloudSetBuffer_2D;
    }
    catch (const boost::system::system_error& ex)
    {
        ROS_ERROR("[Error] instantiating laser object. \
        Are you sure you have the correct port and baud rate? \
        Error was %s", ex.what());
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS
    //ros::init(argc, argv, "line_laser");
    rclcpp::shutdown();
    rclcpp::init(argc, argv);

    std::thread first(running);
    first.join();

    return 0;
}
