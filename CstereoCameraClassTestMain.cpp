#include "StereoCamera.h"
#include <iostream>
using namespace std;

// Global resources
#define HEIGHT 480
#define WIDTH 640
CstereoCamera m_camera( WIDTH, HEIGHT );
IplImage* pfL;
IplImage* pfR;
IplImage* pframeL;
IplImage* pframeR;

int main( int argc, char* argv[] )
{
	pfL = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 3 );
	pfR = cvCreateImage( cvSize( WIDTH, HEIGHT ), 8, 3 );
	if ( !m_camera.StartCamera( ) )
	{
		cout << "StartCamera failed!" << endl;
		return -1;
	}
	else
	{
		while ( 1 )
		{
          if ( m_camera.AcquireFrame( ) && m_camera.StereoMatch( ) )
	        {
		        pframeL = m_camera.GetRetfImgL( );
		        pframeR = m_camera.GetRetfImgR( );
		        m_camera.Four2Three( pframeL, pfL );
		        m_camera.Four2Three( pframeR, pfR );

		        cvShowImage( "Left", pfL );
				    cvShowImage( "Right", pfR );
		        if ( cvWaitKey( 20 ) == 27 ) break;
	        }
		}	
	}
	
	m_camera.StopCamera( );
	cvDestroyWindow( "Left" );
	cvDestroyWindow( "Right" );
	return 0;
}
