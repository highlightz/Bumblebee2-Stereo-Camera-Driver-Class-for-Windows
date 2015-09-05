#ifndef BB2_WRAPPER_H
#define BB2_WRAPPER_H

// System Includes
#include <fstream>
using std::ofstream;

#include <vector>
using std::vector;

// OpenCV Includes
#include <opencv2/opencv.hpp>
using cv::Mat;

// Structures used
#include "structures.h"

// Camera Base Class
#include "StereoCamera.h"

class bb2_wrapper : public CstereoCamera
{
public:
	// ---------------------------------------
	// Construction / Deconstruction
	bb2_wrapper( int sw = 320, int sh = 240 );
	~bb2_wrapper( );
	// ---------------------------------------
	
	// -------------------------------------------------------------------------------------------
	// Stereo camera setting and control
	void setDispRange( int minDisp, int maxDisp );  // Sets the disparity range of stereo matching
	bool StartCamera( );
	void showCameraInfo( );  // Print stereo camera intrinsics
	// -------------------------------------------------------------------------------------------

	// -----------------------------------------------------------
	// Stereo matching method, most important method of this class
	bool StereoMatch( ); 
	// -----------------------------------------------------------

	// ------------------------------------------------------------------------------------------------------
	// Data acquisition
	Mat getDepthImage16( ) const;  // 16-bit disparity image
	IplImage* getStereo( ) const;  // 8-bit disparity image, easy to display
	PointCloud getPointCloud( ) const;  // A point of frame point clouds collection
	vector< CvPoint3D32f > getNinePoints( ) const;  // Nine sampled points from frame point clouds collection
	vector< PointCloud > getFramePointClouds( ) const;  // Frame point clouds collection
	// ------------------------------------------------------------------------------------------------------

	// ------------------------------------------------------------------------------------------------------
	// Data display
	void showFifteenPointsDepth( ) const;  // Fifteen sampled points from point clouds, containing depth info
	void showFifteenPoints3D( ) const;  // Fifteen sampled points from point clouds, containing 3d info
	// ------------------------------------------------------------------------------------------------------

	// Others
	void showAvrgDepth( );

	void showSampledGround( );

	GroundPlane getGroundPlane( ) const;

	void useGround( );

	cv::Mat groundImg;
private:
	// ---------------------------
	// Stereo matching parameters
	int minDisparity;
	int maxDisparity;
	// ---------------------------

	// ------------------------------------------------------------
	// Data acquisition
	PointCloud pc;  // Point cloud data
	vector< PointCloud > pc_seq;  // Point clouds of a whole frame
	void bufferFramePointClouds( );
	// Sample 15 points from point clouds, containing depth info
	vector< float > fifteenPointsDepth;
	void bufferFifteenPointsDepth( );
	// Sample 15 points from point clouds, containing 3d info
	vector < CvPoint3D32f > fifteenPoints3D;
	void bufferFifteenPoints3D( );
	// Sample 9 points form point clouds
	vector< CvPoint3D32f > ninePoints;
	void sampleNinePoints( );
	// ------------------------------------------------------------

	// ------------------------------------------------------
	// Derived data
	// Sample three areas of points to compute a ground plane 
	GroundPlanePoints groundPlanePoints;
	void sampleGroundPoints( );

	// Ground plane computed from three fixed "points",
	// specified by prior experience.
	// Such a plane is determined by four parameters, namely,
	// coefs_0 * x + coefs_1 * y + coefs_2 * z + coefs_3 = 0.
	GroundPlane groundPlane;
	void genGroundPlane( );

	vector< DepthWindow > eP;
	void computeAvrgDepth( );
	// ------------------------------------------------------
};

#endif  // BB2_WRAPPER_H
