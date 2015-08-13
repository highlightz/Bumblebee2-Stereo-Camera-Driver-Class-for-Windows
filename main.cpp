// System Includes
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <iostream>
using std::cout;
using std::endl;

// OpenCV Includes
#include <opencv2/opencv.hpp>

// In this namespace, algorithms being developed and visualizing tools are defined.
namespace miscellaneous
{
	// 1. An OpenCV IplImage struct wrapper, making the accessing to IplImage convenient.
	template < class DataType > 
	class Image
	{
	public:
		Image( IplImage* img = 0 ){ imgp = img; }
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

	// 2. Marks some interest points in 15 fixed position of an image
	void drawInterestPointsOnImage( IplImage* src, int flag )
	{
		// Gray image when flag equals to 1
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

	// 3. Draws a few interest lines on an image
	void drawInterestLinesOnImage( IplImage* src, int flag )
	{
		CvPoint top_left = cvPoint( 10, 10 );     CvPoint top_mid_left = cvPoint( 210, 10 );     CvPoint top_mid_right = cvPoint( 430, 10 );     CvPoint top_right = cvPoint( 630, 10 );
		CvPoint bottom_left = cvPoint( 10, 470 ); CvPoint bottom_mid_left = cvPoint( 210, 470 ); CvPoint bottom_mid_right = cvPoint( 430, 470 ); CvPoint bottom_right = cvPoint( 630, 470 );

		// Gray image when flag equals to 1
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

	// 4. Computes V-disparity based on the input disparity image
	// maxDisp is needed
	// Make a counting line by line, the number of columns of the resulting V-disparity is defined by maxDisp
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
		vector< Vec2f > lines;
		// Hough transform, gathering a series of parameters (rho, theta), 
		// in which each pair parameter corresponds to a line
		HoughLines( contours, lines, 1, pi / 180, 80 );

		vector< Vec2f >::const_iterator it = lines.begin( );
		cout << lines.size( ) << endl;
		for ( it = lines.begin( ); it < lines.end( ); it++ )
		{
			float rho = (*it)[0];
			float theta = (*it)[1];
			if ( theta < pi / 4 || theta > pi * 3 / 4 )
			{
				Point pt1( rho/cos(theta),0);
				Point pt2( ( rho - src.rows*sin(theta))/cos(theta), src.rows );
				line( src, pt1, pt2, Scalar( 255 ), 1 );
			}
			else
			{
				Point pt1( 0, rho/sin(theta));
				Point pt2( src.cols, ( rho - src.cols*cos(theta))/sin(theta) );
				line( src, pt1, pt2, Scalar( 255 ), 1 );
			}
		}
	}
}

#include "bb2_wrapper.h"
#define HEIGHT 480
#define WIDTH 640
bb2_wrapper m_camera( WIDTH, HEIGHT );

IplImage* pfL;
IplImage* pfR;
IplImage* pframeL;
IplImage* pframeR;
IplImage* disp8;

int main( )
{
	pfL = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 3 );
	pfR = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 3 );
	disp8 = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 1 );
	
	// m_camera.setDispRange( 0, 200 );

	if ( !m_camera.StartCamera( ) )
	{
		cout << "StartCamera failed!" << endl;
		return -1;
	}
	else
	{
		// m_camera.showCameraInfo( );
		m_camera.EnableStereoMatch( );
		while ( 1 )
		{
			if ( m_camera.AcquireFrame( ) && m_camera.StereoMatch( ) )
			{
				pframeL = m_camera.GetRetfImgL( );
				pframeR = m_camera.GetRetfImgR( );
				m_camera.Four2Three( pframeL, pfL );
				m_camera.Four2Three( pframeR, pfR );

				disp8 = m_camera.getStereo( );
				
				m_camera.showAvrgDepth( );

				cvShowImage( "Left", pfL );
				cvShowImage( "Right", pfR );
				cvShowImage( "Disp8", disp8 );
				
				if ( cvWaitKey( 20 ) == 27 ) 
				{
				  break;
				}
			}
		}	
	}

	m_camera.StopCamera( );
	cvDestroyWindow( "Left" );
	cvDestroyWindow( "Right" );
	return 0;
}
