#ifndef __NUITRACK__
#define __NUITRACK__

#include <nuitrack/Nuitrack.h>
#include <opencv2/opencv.hpp>

#include <array>

#define USER_COUNT 3

class NuiTrack
{
public://era private
    // Depth Sensor
    tdv::nuitrack::DepthSensor::Ptr		depth_sensor;
    tdv::nuitrack::DepthFrame::Ptr		depth_frame;
	cv::Mat								depth_mat;			// mat de profundidad

	// RGB Sensor		viene de skeleton
	tdv::nuitrack::ColorSensor::Ptr		color_sensor;
	tdv::nuitrack::RGBFrame::Ptr		color_frame;
	cv::Mat								color_mat;			// de color
	cv::Mat								final_mat;			// final 1280 x 800

	uint32_t final_width	= 1280;							// 1280;
	uint32_t final_height	= 750;							// 800;

	uint32_t depth_width	= 848;							// 848 era 1280;
	uint32_t depth_height	= 480;							// 480 era 720;
	uint32_t color_width	= 848;							// 848 era 1280;	
	uint32_t color_height	= 480;							// 480 era 720;		de skeleton
	uint32_t max_distance	= 5000;							// 5000;
	uint32_t fps			= 90;							// 60 AGREGADA

    // User Tracker
    tdv::nuitrack::UserTracker::Ptr		user_tracker;
    tdv::nuitrack::UserFrame::Ptr		user_frame;
    cv::Mat								user_mat;			// mat de usuario 848 x 480
	std::array<cv::Vec3b, 10>			colors;				// USER_COUNT > colors;

	// Skeleton Tracker de skeleton
	tdv::nuitrack::SkeletonTracker::Ptr skeleton_tracker;
	tdv::nuitrack::SkeletonData::Ptr	skeleton_data;
	cv::Mat								skeleton_mat;		// mat de esqueleto 848 x 480
	
	// Align de skeleton
	bool align = true;


public:
    // Constructor
    NuiTrack( const std::string& config_json = "" );

    // Destructor
    ~NuiTrack();

    // Processing
    int run();

	// las saco del ambito privado
	void drawSkeleton();
	void drawUser();

private:
    // Initialize
    void initialize( const std::string& config_json );

    // Initialize Sensor
    inline void initializeSensor();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Frame
    inline void updateFrame();

    // Update Depth
    inline void updateDepth();

    // Update User
    inline void updateUser();

	// Update Color de skeleton
	inline void updateColor();

	// Update Skeleton de skeleton
	inline void updateSkeleton();


    // Draw Data
    void draw();

    // Draw Depth
    inline void drawDepth();

    // Draw User
    //inline void drawUser();

	// Draw Color de skeleton
	inline void drawColor();

	// Draw Skeleton de skeleton
	//inline void drawSkeleton();


    // Show Data
    void show();

    // Show User
    //inline void showUser();

	// Show Skeleton de skeleton
	inline void showSkeleton();

};

#endif // __NUITRACK__
