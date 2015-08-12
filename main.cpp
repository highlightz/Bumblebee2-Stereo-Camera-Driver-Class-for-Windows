// System Includes
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <iostream>
using std::cout;
using std::endl;

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
