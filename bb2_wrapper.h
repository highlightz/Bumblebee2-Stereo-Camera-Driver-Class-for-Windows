#ifndef BB2_WRAPPER_H
#define BB2_WRAPPER_H

#include "StereoCamera.h"
#include <fstream>
#include <vector>

using std::ofstream;
using std::vector;
using cv::Mat;

// The point cloud refers to camera as a reference coordinate frame.
struct PointCloud
{
	float x, y, z;
	int r, g, b;
	int row, col;
};

// Such a struct object represents a neighborhood of an interest point,
// which serves to determine the ground plane.
// Three of them are needed.
struct EffectivePoint
{
	float avrgDepth;
	int numOfEffectivePoints;  // Number of effective point clouds in this area.
	
	// Number of near point clouds, used to determine a threshold,
	// which tells the car there is danger in that direction.
	int numPixelLessThanFixedDepthThreshold;   
};

class bb2_wrapper : public CstereoCamera
{
public:
	bb2_wrapper( int sw = 320, int sh = 240 );
	~bb2_wrapper( );

	void setDispRange( int minDisp, int maxDisp );

	bool StartCamera( );

	// Show camera info
	void showCameraInfo( );

	Mat getDepthImage16( ) const;

	// Only for display
	IplImage* getStereo( ) const;

	PointCloud getPointCloud( ) const;

	bool StereoMatch( ); 

	void showInterestPointsDepth( ) const;

	void showInterestPoints3D( ) const;

	void showAvrgDepth( );
	
	void showSampledGround( );

	vector< CvPoint3D32f >  getNinePoints( ) const;
	
	void useGround( );
	
	cv::Mat groundImg;  // Stores the ground plane info
private:
	int minDisparity;
	int maxDisparity;
	PointCloud pc;
	
	struct Plane3Points
	{
		CvPoint3D32f planePointA;
		int numOfEffectivePointsAourndA;

		CvPoint3D32f planePointB;
		int numOfEffectivePointsAourndB;

		CvPoint3D32f planePointC;
		int numOfEffectivePointsAourndC;
		
		Plane3Points( )
		{
			planePointA.x = 0;
			planePointA.y = 0;
			planePointA.z = 0;
			numOfEffectivePointsAourndA = 0;

			planePointB.x = 0;
			planePointB.y = 0;
			planePointB.z = 0;
			numOfEffectivePointsAourndB = 0;

			planePointC.x = 0;
			planePointC.y = 0;
			planePointC.z = 0;
			numOfEffectivePointsAourndC = 0;
		}
	} ground;
	
	// Stores the params of a plane equation
	struct FinalGround
	{
		float coefs_0;
		float coefs_1;
		float coefs_2;
		float coefs_3;
		FinalGround( )
		{
			coefs_0 = 0.0f;
			coefs_1 = 0.0f;
			coefs_2 = 0.0f;
			coefs_3 = 0.0f;
		}
	} finalGroundPlane;
	
	void sampleGround( );
	
	// Working mode: batch processing 
	void bufferGround( );
	
	vector< float > fifteenPointsDepth;
	void bufferFifteenPointsDepth( ); 

	vector < CvPoint3D32f > fifteenPoints3D;
	void bufferFifteenPoints3D( );

	vector< EffectivePoint > eP;
	void computeAvrgDepth( );

	vector< CvPoint3D32f > ninePoints;
	void sampleNinePoints( );
};

#endif
