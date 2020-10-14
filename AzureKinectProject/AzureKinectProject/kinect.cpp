#include "kinect.hpp"
#include "util.h"

#include <chrono>

#include <thread>

// Constructor
kinect::kinect( const uint32_t index )
    : device_index( index )
{
    // Initialize
    initialize();
}

kinect::~kinect()
{
    // Finalize
    finalize();
}

// Initialize
void kinect::initialize()
{
    // Initialize Sensor
    initialize_sensor();
}

// Initialize Sensor
inline void kinect::initialize_sensor()
{
    // Get Connected Devices
    const int32_t device_count = k4a::device::get_installed_count();
    if( device_count == 0 ){
        throw k4a::error( "Failed to found device!" );
    }

    // Open Default Device
    device = k4a::device::open( device_index );
    // Start Cameras with Configuration
    device_configuration = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    device_configuration.color_format             = k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32;
    device_configuration.color_resolution         = k4a_color_resolution_t::K4A_COLOR_RESOLUTION_1080P;
	device_configuration.depth_mode				  = k4a_depth_mode_t::K4A_DEPTH_MODE_WFOV_2X2BINNED;
	//device_configuration.camera_fps				  = k4a_fps_t::K4A_FRAMES_PER_SECOND_30;
	//device_configuration.depth_mode				  = k4a_depth_mode_t::K4A_DEPTH_MODE_NFOV_UNBINNED;
	device_configuration.synchronized_images_only = true;
    device_configuration.wired_sync_mode          = k4a_wired_sync_mode_t::K4A_WIRED_SYNC_MODE_STANDALONE;
    device.start_cameras( &device_configuration );

	//device.start_imu();


	InitCalibration();
	//==================================================================================================================
	//k4a_calibration_t calibration;

	//k4a_device_get_calibration(device.handle(), device_configuration.depth_mode, device_configuration.color_resolution, &calibration);

	/*calibration.color_camera_calibration.intrinsics.parameters.param.fx;
	calibration.color_camera_calibration.intrinsics.parameters.param.fy;
	calibration.color_camera_calibration.intrinsics.parameters.param.cx;
	calibration.color_camera_calibration.intrinsics.parameters.param.cy;

	calibration.depth_camera_calibration.resolution_height = 1080;
	calibration.depth_camera_calibration.resolution_width = 1920;
	/*calibration.color_camera_calibration.resolution_height;
	calibration.color_camera_calibration.resolution_width;*/
	//==================================================================================================================

	
	//trasnformation_ = k4a_transformation_create(&calibration);
	//trasnformation_ = k4a_transformation_create(&calibration);


}

// Finalize
void kinect::finalize()
{
	k4a_image_release(depth_t);

    // Stop Cameras
    device.stop_cameras();

	//device.stop_imu();

	// Close Device
    device.close();

    // Close Window
    cv::destroyAllWindows();

}

// Run
void kinect::run()
{
	std::cout << "================================================작동================================================" << std::endl;
	clock_t start, end;
	double result;
	int i, j;
	int sum = 0;
    // Main Loop
    while( true ){
	

        // Update
        update();

        // Draw
        draw();

		start = clock();

        // Show
        show();

        // Wait Key
        constexpr int32_t delay = 15;
        const int32_t key = cv::waitKey(delay);
        if( key == 'q' ){
            break;
        }

		end = clock();
		result = (double)(end - start);
		std::cout << ((result) / CLOCKS_PER_SEC) << std::endl;
		std::cout << "================================================종료================================================" << std::endl;

    }
}

// Update
void kinect::update()
{

    // Update Frame
    update_frame();

	//Update IMU
	//update_IMU();

	// Update Color, Depth
	update_color_depth();

	// Release Capture Handle
    capture.reset();
}

// Update Frame
inline void kinect::update_frame()
{
    // Get Capture Frame
    constexpr std::chrono::milliseconds time_out( K4A_WAIT_INFINITE );
    const bool result = device.get_capture( &capture, time_out );
    if( !result ){
        this->~kinect();
    }
}

inline void kinect::update_IMU()
{
	device.get_imu_sample(&device_imu_sample);

	/*std::cout << "row : " << device_imu_sample.acc_sample.xyz.x << std::endl
			  << "picth : " << device_imu_sample.acc_sample.xyz.y << std::endl
			  << "yaw : " << device_imu_sample.acc_sample.xyz.z << std::endl
			  << "v : " << device_imu_sample.acc_sample.v << std::endl;

	std::cout << "temp : " << device_imu_sample.temperature << std::endl;

	std::cout << "x : " << device_imu_sample.gyro_sample.xyz.x << std::endl
			  << "y : " << device_imu_sample.gyro_sample.xyz.y << std::endl
			  << "z : " << device_imu_sample.gyro_sample.xyz.z << std::endl
			  << "v : " << device_imu_sample.gyro_sample.v << std::endl;*/

}

// Update Color, Depth
inline void kinect::update_color_depth()
{
	// Get Color Image
	color_image = capture.get_color_image();

	// Get Depth Image
	depth_image = capture.get_depth_image();
}

// Draw
void kinect::draw()
{	
    // Draw Depth
	draw_color_depth();
}

void kinect::InitCalibration()
{
	if (k4a_device_get_calibration(device.handle(), device_configuration.depth_mode, device_configuration.color_resolution, &calibration_) == 1)
	{
		k4a_device_close(device.handle());
		return;
	}
	trasnformation_ = k4a_transformation_create(&calibration_);
	
}

void kinect::Test(k4a_image_t dImage, cv::Mat color1)
{
	depth_t = dImage;
	transformedDepth = NULL;

	if (k4a_image_create(depth_image.get_format(), color1.cols, color1.rows, color1.cols * 2, &transformedDepth) == K4A_RESULT_SUCCEEDED)
	{
		if (k4a_transformation_depth_image_to_color_camera(trasnformation_, depth_t, transformedDepth) == K4A_RESULT_SUCCEEDED)
		{
			depth = cv::Mat(
				k4a_image_get_height_pixels(transformedDepth),
				k4a_image_get_width_pixels(transformedDepth),
				CV_16UC1,
				(void*)k4a_image_get_buffer(transformedDepth)).clone();
			k4a_image_release(transformedDepth);
		}
	}
	else
	{
		std::cout << "K4A failed to register depth image" << std::endl;
		return;
	}
}

// Draw Depth
inline void kinect::draw_color_depth()
{
	if (!color_image.handle() || !depth_image.handle()) {
		return;
	}

	// Get cv::Mat from k4a::image
	color = k4a::get_mat(color_image);

    // Get cv::Mat from k4a::image
	//std::cout << depth_image.get_format() << std::endl;
    //depth = k4a::get_mat( depth_image );

	Test(depth_image.handle(), color);


	// Release Color Image Handle
	color_image.reset();

    // Release Depth Image Handle
    depth_image.reset();
}

// Show
void kinect::show()
{
    // Show Color, Depth
	show_color_depth();
}
static int i = 0;

bool writer(std::string _std, cv::Mat _color, cv::Mat _depth)
{
	cv::imwrite("C:\\Users\\User\\Desktop\\test\\color" + _std + ".jpg", _color);
	cv::imwrite("C:\\Users\\User\\Desktop\\test\\test" + _std + ".jpg", _depth);
	return true;
}

// Show Color
inline void kinect::show_color_depth()
{
	if (color.empty() && depth.empty()) {
		return;
	}

	i++;
	str1 = std::to_string(i);
	//cv::resize(color, cst, cv::Size(500, 500), 0, 0, CV_16UC1);
	cv::resize(depth, dst, cv::Size(500, 500), 0, 0, CV_16UC1);


	//t1.detach();

		// Show Image

		//cv::imwrite("C:\\Users\\User\\Desktop\\test\\test" + str1 + ".jpg", color);
		//const cv::String window_name = cv::format("color (kinect %d)", device_index);

		//cv::namedWindow("Color", 0);
		//c::Mat srm(512, 512, CV_8UC1);

	cv::imshow("Color", color);



	// Show Depth
		// Scaling Depth
		//depth.convertTo(depth, CV_16UC1, -255.0 / 5000.0, 255.0);
		//depth.convertTo(depth, CV_16UC1);

		//const cv::String window_name = cv::format( "depth (kinect %d)", device_index );

	cv::imshow("Depth", dst);

	//cst.release();
	dst.release();
	//writer(str1, color, depth);
	/*std::thread t1(writer, str1, color, depth);

	//t1.detach();
		t1.join();*/

}

