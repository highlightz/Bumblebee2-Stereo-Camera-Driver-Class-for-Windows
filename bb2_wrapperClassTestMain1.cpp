// bb2_wrapper Class Test
// System Includes
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include <iostream>
using std::cout;
using std::endl;

#include <vector>
using std::vector;

// OpenCV Includes
#include <opencv2/opencv.hpp>

// Structures used
#include "structures.h"

// Camera derived class
#include "bb2_wrapper.h"

namespace miscellaneous
{
	//
	// Consider artificial forces composed of tens of thousands of point clouds.
	// Take ground plane as a constraint.
	// Returns an safest direction angle( in degrees ).
	float sumArtificialForces( vector< PointCloud > pc_seq, GroundPlane groundPlane )
	{
		// Initialize sum force.
		// y direction force not considered, 
		// this equals to projecting 3d point to x-z plane.
		float sum_force_x = 0.0f;
		float sum_force_z = 0.0f;

		const float belongToGroundThreshold = 0.03f;  // Unit: meter

		// Normalize the norm vector of ground plane
		float normed_vector = 1 / sqrt( groundPlane.coefs_0 * groundPlane.coefs_0 
			                + groundPlane.coefs_1 * groundPlane.coefs_1
			                + groundPlane.coefs_2 * groundPlane.coefs_2 );

		for ( int i = 0; i < pc_seq.size( ); i++ )
		{
			float distanceToGround = abs( groundPlane.coefs_0 * pc_seq[i].x
				                   + groundPlane.coefs_1 * pc_seq[i].y
				                   + groundPlane.coefs_2 * pc_seq[i].z 
				                   + groundPlane.coefs_3 ) * normed_vector;

			if ( distanceToGround > belongToGroundThreshold )  // If the point cloud is above the ground
			{
				sum_force_x += pc_seq[i].x;
				sum_force_z += pc_seq[i].z;
			}
		}

		// Coordinate transform
		float x = sum_force_z;
		float y = -sum_force_x;

		float cos_theta = y / sqrt( x * x + y * y );
		float angle_in_radian = asin( cos_theta );
		float angle_in_deg = angle_in_radian * 180 / 3.14;

		return angle_in_deg;
	}

	template < class DataType > 
	class Image
	{
	public:
		Image( IplImage *img = 0 ){ imgp = img; }
		~Image(  ){ imgp = 0; }
		void operator =( IplImage *img ){ imgp = img; }
		inline DataType* operator[]( const int rowIndx )
		{
			return ( ( DataType* )( imgp->imageData + rowIndx * imgp->widthStep ) );
		}
	private:
		IplImage *imgp;
	};

	typedef struct
	{
		unsigned char b, g, r;
	} RgbPixel;

	typedef struct
	{
		float b, g, r;
	} RgbPixelFloat;

	typedef Image<RgbPixel> RgbImage;
	typedef Image<RgbPixelFloat> RgbImageFloat;
	typedef Image<unsigned char> BwImage;
	typedef Image<float> BwImageFloat;

	void drawInterestPointsOnImage( IplImage* src, int flag )
	{
		if ( flag == 1 )
		{
			// First line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height / 4 ), 3, cvScalar(0, 0, 255) );

			// Second line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height / 2 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height / 2 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height / 2 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height / 2 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height / 2 ), 3, cvScalar(0, 0, 255) );

			// Third line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height - src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height - src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height - src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height - src->height / 4 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height - src->height / 4 ), 3, cvScalar(0, 0, 255) );
		}
		if ( flag == 0 )
		{
			// First line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height / 4 ), 3,cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height / 4 ), 3, cvScalar(255, 255, 255) );

			// Second line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height / 2 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height / 2 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height / 2 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height / 2 ), 3, cvScalar(0, 0, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height / 2 ), 3, cvScalar(255, 255, 255) );

			// Third line
			cvCircle( src, cvPoint( src->width / 4 - src->width / 5, src->height - src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 4, src->height - src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width / 2, src->height - src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4, src->height - src->height / 4 ), 3, cvScalar(255, 255, 255) );
			cvCircle( src, cvPoint( src->width - src->width / 4 + src->width / 5, src->height - src->height / 4 ), 3, cvScalar(255, 255, 255) );
		}
	}

	void drawInterestLinesOnImage( IplImage* src, int flag )
	{
		CvPoint top_left = cvPoint( 10, 10 );     CvPoint top_mid_left = cvPoint( 210, 10 );     CvPoint top_mid_right = cvPoint( 430, 10 );     CvPoint top_right = cvPoint( 630, 10 );
		CvPoint bottom_left = cvPoint( 10, 470 ); CvPoint bottom_mid_left = cvPoint( 210, 470 ); CvPoint bottom_mid_right = cvPoint( 430, 470 ); CvPoint bottom_right = cvPoint( 630, 470 );

		if ( flag == 1 )
		{
			cvLine( src, top_left, bottom_left, cvScalar(0, 255, 0), 2 );

			cvLine( src, top_mid_left, bottom_mid_left, cvScalar(0, 255, 0), 2 );

			cvLine( src, top_mid_right, bottom_mid_right, cvScalar(0, 255, 0), 2 );

			cvLine( src, top_right, bottom_right, cvScalar(0, 255, 0), 2 );

			cvLine( src, top_left, top_right, cvScalar(0, 255, 0), 2 );

			cvLine( src, bottom_left, bottom_right, cvScalar(0, 255, 0), 2 );
		}

		if ( flag == 0 )
		{
			cvLine( src, top_left, bottom_left, cvScalar(255, 255, 255), 2 );

			cvLine( src, top_mid_left, bottom_mid_left, cvScalar(255, 255, 255), 2 );

			cvLine( src, top_mid_right, bottom_mid_right, cvScalar(255, 255, 255), 2 );

			cvLine( src, top_right, bottom_right, cvScalar(255, 255, 255), 2 );

			cvLine( src, top_left, top_right, cvScalar(255, 255, 255), 2 );

			cvLine( src, bottom_left, bottom_right, cvScalar(255, 255, 255), 2 );
		}

	}

	// maxDisp is needed
	// 逐行统计计算，需确定V视差图的列数，列数就是maxDisp
	void computeVDisparity( IplImage* src, IplImage* dst )
	{
		for ( int rowComm = 0; rowComm < src->height; rowComm++ )
		{
			for ( int colDst = 0; colDst < dst->width; colDst++ )
			{
				// Do the counting of a row of the src disparity image
				int counter = 0;
				for ( int colSrc = 0; colSrc < src->width; colSrc++ )
				{
					if ( src->imageData[ rowComm * src->widthStep + colSrc ] == colDst )
					{
						counter++;
					}
				}

				dst->imageData[ rowComm * dst->widthStep + colDst ] = counter;
			}
		}
	}

	// maxDisp is needed
	// 逐列统计计算，需确定U视差图的行数，行数就是maxDisp
	void computeUDisparity( IplImage* src, IplImage* dst )
	{
		for ( int colComm = 0; colComm < src->width; colComm++ )
		{
			for ( int rowDst = 0; rowDst < dst->height; rowDst++ )
			{
				// Do the counting of a column of the src disparity image
				int counter = 0;
				for ( int rowSrc = 0; rowSrc < src->height; rowSrc++ )
				{
					if ( src->imageData[ rowSrc * src->widthStep + colComm ] == rowDst )
					{
						counter++;
					}
				}

				dst->imageData[ rowDst * dst->widthStep + colComm ] = counter;
			}
		}
	}

	void houghTransform( Mat& src )
	{
		const double pi = 3.14;

		Mat contours;
		Canny( src, contours, 125, 350 );
		vector< cv::Vec2f > lines;
		// Hough transform, gathering a series of parameters (rho, theta), 
		// in which each pair parameter corresponds to a line
		HoughLines( contours, lines, 1, pi / 180, 80 );

		vector< cv::Vec2f >::const_iterator it = lines.begin( );
		cout << lines.size( ) << endl;
		for ( it = lines.begin( ); it < lines.end( ); it++ )
		{
			float rho = (*it)[0];
			float theta = (*it)[1];
			if ( theta < pi / 4 || theta > pi * 3 / 4 )
			{
				cv::Point pt1( rho/cos(theta),0);
				cv::Point pt2( ( rho - src.rows*sin(theta))/cos(theta), src.rows );
				cv::line( src, pt1, pt2, cv::Scalar( 255 ), 1 );
			}
			else
			{
				cv::Point pt1( 0, rho/sin(theta));
				cv::Point pt2( src.cols, ( rho - src.cols*cos(theta))/sin(theta) );
				cv::line( src, pt1, pt2, cv::Scalar( 255 ), 1 );
			}
		}
	}

	void computeAverageDepth( IplImage* disparity, 
		                      double& left, double& middle, double& right )							
	{
		left = middle = right = 0;
		const int row_target = 240;
		for ( int row = row_target; row < disparity->height; row++ )
		{
			for ( int col = 10; col < 210; col++ )
			{
				left += static_cast< double >( disparity->imageData[ row * disparity->widthStep + col ] ) ;// ( disparity->height - row_target ) / ( 210 - 10 );
			}
		}

		for ( int row = row_target; row < disparity->height; row++ )
		{
			for ( int col = 210; col < 430; col++ )
			{
				cout << static_cast< double >( disparity->imageData[ row * disparity->widthStep + col ] );// ( disparity->height - row_target ) / ( 430 - 210 );
			}
		}
		//cout << middle << endl;
		for ( int row = row_target; row < disparity->height; row++ )
		{
			for ( int col = 430; col < 630; col++ )
			{
				right += static_cast< double >( disparity->imageData[ row * disparity->widthStep + col ] );// ( disparity->height - row_target ) / ( 630 - 430 );
			}
		}

		/*
		if ( row > row_target && pc.row < 470 && pc.col > 10 && pc.col < 210 )
		{
			avrgDepth[0] += pc.z;
		}

		if ( pc.row > row_target && pc.row < 470 && pc.col >= 210 && pc.col < 430 )
		{
			avrgDepth[1] += pc.z;
		}

		if ( pc.row > row_target && pc.row < 470 && pc.col >= 430 && pc.col < 630 )
		{
			avrgDepth[2] += pc.z;
		}*/
	}
}


bb2_wrapper m_camera(640,480);

const int maxDisp = 255;

IplImage* pfL;
IplImage* pfR;
IplImage* pframeL;
IplImage* pframeR;
IplImage* disp8;
IplImage* vdisp;
IplImage* udisp;
IplImage* edgesOfDepth;
IplImage* dilated;

// Function declarations
void miscellaneous::drawInterestPointsOnImage( IplImage* src, int flag );
void miscellaneous::drawInterestLinesOnImage( IplImage* src, int flag );
void miscellaneous::computeUDisparity( IplImage* src, IplImage* dst );
void miscellaneous::computeVDisparity( IplImage* src, IplImage* dst );
void miscellaneous::houghTransform( Mat& src );
void miscellaneous::computeAverageDepth( IplImage* disparity, double& left, double& middle, double& right );
float miscellaneous::sumArtificialForces( vector< PointCloud > pc_seq, GroundPlane groundPlane );

int main( )
{	
	pfL = cvCreateImage(cvSize(640,480),8,3);
	pfR = cvCreateImage(cvSize(640,480),8,3);
	disp8 = cvCreateImage(cvSize(640,480),8,1);
	udisp = cvCreateImage( cvSize( 640, maxDisp ), 8, 1 );
	vdisp = cvCreateImage( cvSize( maxDisp, 480 ), 8, 1 );
	edgesOfDepth = cvCreateImage( cvSize( 640, 480 ), 8, 1 );
	dilated = cvCreateImage( cvSize( 640, 480 ), 8, 1 );

	//m_camera.setDispRange( 0, 200 );

	if( !m_camera.StartCamera() )
	{
		cout<<"StartCamera failed!"<<endl;
		return -1;
	}
	else
	{
		//m_camera.showCameraInfo( );
		m_camera.EnableStereoMatch( );
		while (1)
		{
			if(m_camera.AcquireFrame()&&m_camera.StereoMatch())
			{
				pframeL = m_camera.GetRetfImgL();
				pframeR = m_camera.GetRetfImgR();
				m_camera.Four2Three(pframeL,pfL);
				m_camera.Four2Three(pframeR,pfR);
				
				disp8 = m_camera.getStereo( );

				CvPoint pt1 = cvPoint( 320, 420 );
				CvPoint pt2 = cvPoint( 280, 440 );
				CvPoint pt3 = cvPoint( 360, 440 );
				int thickness = 5;
				cvLine( disp8, pt1, pt2, cvScalar( 255, 255, 255 ), thickness );
				cvLine( disp8, pt1, pt3, cvScalar( 255, 255, 255 ), thickness );
				cvLine( disp8, pt3, pt2, cvScalar( 255, 255, 255 ), thickness );
				cvLine( pfL, pt1, pt2, cvScalar( 255, 0, 0 ), thickness );
				cvLine( pfL, pt1, pt3, cvScalar( 255, 0, 0 ), thickness );
				cvLine( pfL, pt3, pt2, cvScalar( 255, 0, 0 ), thickness );
				cvLine( pfR, pt1, pt2, cvScalar( 0, 255, 0 ), thickness );
				cvLine( pfR, pt1, pt3, cvScalar( 0, 255, 0 ), thickness );
				cvLine( pfR, pt3, pt2, cvScalar( 0, 255, 0 ), thickness );
				//double l, m, r;
				//miscellaneous::computeAverageDepth( disp8, l, m, r );
				//cout << l << ", " << m << ", " << r << endl;

				/*miscellaneous::BwImage disp8_wrapper( disp8 );
				for ( int i = 0; i < 640; i++ )
					for ( int j = 0; j < 480; j++ )
					{
						cout << (int)disp8_wrapper[j][i] << '\t';
					}
				*/
				
				/*
				miscellaneous::computeUDisparity( disp8, udisp );
				miscellaneous::computeVDisparity( disp8, vdisp );
				cvShowImage( "udisp", udisp );
				cvShowImage( "vdisp", vdisp );
				*/

				/*
				Mat iplWrapper( disp8 );
				miscellaneous::houghTransform( iplWrapper );
				imshow( "lines", iplWrapper );
				*/

				//m_camera.showInterestPointsDepth( );

				//m_camera.showInterestPoints3D();

				//m_camera.showAvrgDepth( );

				//vector< CvPoint3D32f > iA( 9 );
				//iA = m_camera.getNinePoints( );

				//m_camera.showSampledGround( );
				//cv::imshow( "ground", m_camera.groundImg );
				cv::Mat lraw( pfL );
				cv::imshow( "mixed", lraw+m_camera.groundImg );
				m_camera.groundImg.setTo( cv::Scalar(0) );
				
				//cout << iA[0].x << ", " << iA[0].y << ", " << iA[0].z << endl;

				vector< PointCloud > framePc = m_camera.getFramePointClouds( );
				GroundPlane frameGp = m_camera.getGroundPlane( );
				cout << miscellaneous::sumArtificialForces( framePc, frameGp ) << endl;
				//cout << framePc.size( ) << endl;
				
				//vector< int > randomIndex( 3 );
				//randomIndex = aR.genRandomIndex( );		
				//cout << randomIndex[0] << ": " << iA[randomIndex[0]].x << ", " << iA[randomIndex[0]].y << ", " << iA[randomIndex[0]].z << endl;

				//cvCanny(disp8, edgesOfDepth, 20, 100 );
				//cvDilate( disp8, dilated);

				//miscellaneous::drawInterestPointsOnImage( pfR, 1 );
				//miscellaneous::drawInterestLinesOnImage( pfR, 1 );

				//miscellaneous::drawInterestPointsOnImage( disp8, 0 );
				//miscellaneous::drawInterestLinesOnImage( disp8, 0 );

				//miscellaneous::drawInterestPointsOnImage( pfL, 1 );
				//miscellaneous::drawInterestLinesOnImage( pfL, 1 );

				//cvShowImage("Left",pfL);
				//cvShowImage("Right",pfR);
				
				cvShowImage("Disp8", disp8 );
				//cvShowImage("edge", edgesOfDepth );
				//cvShowImage( "dilated", dilated );
				//if(cvWaitKey(20)==27) break;
				cvWaitKey( 5 );
			}
		}	
	}

	m_camera.StopCamera();
	cvDestroyWindow("Left");
	cvDestroyWindow("Right");
	return 0;
}

#endif
