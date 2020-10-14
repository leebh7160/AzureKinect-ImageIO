#ifndef __KINECT__
#define __KINECT__

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include <k4a/k4atypes.h>
#include <k4arecord/record.hpp>
#include <k4arecord/playback.h>
#include <opencv2/opencv.hpp>

class kinect
{
private:
    // Kinect

    k4a::device device;
    k4a::capture capture;
    k4a_device_configuration_t device_configuration;
	//=======================================================
	k4a_imu_sample_t device_imu_sample;
	k4a_transformation_t trasnformation_;
	k4a_calibration_t calibration_;
	k4a_image_t depth_t;
	k4a_image_t transformedDepth = NULL;


	cv::Mat cst;
	cv::Mat dst;
	std::string str1;
	//=======================================================
	uint32_t device_index;

	// Color
	k4a::image color_image;
	cv::Mat color;

    // Depth
    k4a::image depth_image;
    cv::Mat depth;

public:
    // Constructor
    kinect( const uint32_t index = K4A_DEVICE_DEFAULT );

    // Destructor
    ~kinect();

    // Run
    void run();

    // Update
    void update();

    // Draw
    void draw();

    // Show
    void show();

private:
    // Initialize
    void initialize();

    // Initialize Sensor
    void initialize_sensor();

    // Finalize
    void finalize();

    // Update Frame
    void update_frame();

	//20-10-05 - UpdateIMU
	void update_IMU();

	// Updatd Color,Depth
	void update_color_depth();

	// Show Color, Depth
	void show_color_depth();

    // Draw Color, Depth
    void draw_color_depth();

	void Test(k4a_image_t dImage, cv::Mat color);

	void InitCalibration();
};

#endif // __KINECT__
