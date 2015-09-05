// bb2_wrapper class
#include "bb2_wrapper.h"

#include <iostream>
using std::cout;
using std::endl;
using std::fixed;
using std::setprecision;

bb2_wrapper::bb2_wrapper( int sw, int sh ) 
	: CstereoCamera( sw, sh ),  // Set definition
	  fifteenPointsDepth( 15 ),  // Choose fifteen fixed points of scene 
	  fifteenPoints3D( 15 ),  // Same as above 
	  eP( 3 ), 
	  ninePoints( 9 )
{
	minDisparity = 0;
	maxDisparity = 64;
	
	groundImg.create( sh, sw, CV_8UC3 );
}

bb2_wrapper::~bb2_wrapper( )
{
}

void bb2_wrapper::setDispRange( int minDisp, int maxDisp )
{
	minDisparity = minDisp;
	maxDisparity = maxDisp;
}

bool bb2_wrapper::StartCamera( )
{
	if(runStatus)
		return true;

	// Open the camera
	fe = flycaptureCreateContext( &flycapture );
	if( ! _HANDLE_FLYCAPTURE_ERROR( "flycaptureCreateContext()", fe ) ) 
		return false;

	// Initialize the Flycapture context
	fe = flycaptureInitialize( flycapture, 0 );
	if( ! _HANDLE_FLYCAPTURE_ERROR( "flycaptureInitialize()", fe ) )
		return false;

	// Save the camera's calibration file, and return the path 
	fe = flycaptureGetCalibrationFileFromCamera( flycapture, &szCalFile );
	if( ! _HANDLE_FLYCAPTURE_ERROR( "flycaptureGetCalibrationFileFromCamera()", fe ) )
		return false;

	// Create a Triclops context from the cameras calibration file
	te = triclopsGetDefaultContextFromFile( &triclops, szCalFile );
	if( ! _HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile()", te ) )
		return false;

	// Get camera information
	fe = flycaptureGetCameraInfo( flycapture, &pInfo );
	if( ! _HANDLE_FLYCAPTURE_ERROR( "flycatpureGetCameraInfo()", fe ) ) 
		return false;

	if (pInfo.CameraType == FLYCAPTURE_COLOR)
		pixelFormat = FLYCAPTURE_RAW16;	 
	else 
		pixelFormat = FLYCAPTURE_MONO16;

	switch (pInfo.CameraModel)
	{
	case FLYCAPTURE_BUMBLEBEE2:
		unsigned long ulValue;
		flycaptureGetCameraRegister( flycapture, 0x1F28, &ulValue );

		if ( ( ulValue & 0x2 ) == 0 )
		{// Hi-res BB2
			iMaxCols = 1024; 
			iMaxRows = 768;   
		}
		else
		{// Low-res BB2
			iMaxCols = 640;
			iMaxRows = 480;
		}
		break;

	case FLYCAPTURE_BUMBLEBEEXB3:
		iMaxCols = 1280;
		iMaxRows = 960;
		break;

	default:
		te = TriclopsErrorInvalidCamera;
		if( ! _HANDLE_TRICLOPS_ERROR( "triclopsCheckCameraModel()", te ) )
			return false;
		break;
	}

	// Start transferring images from the camera to the computer
	fe = flycaptureStartCustomImage( 
		flycapture, 3, 0, 0, iMaxCols, iMaxRows, 100, pixelFormat);
	if( ! _HANDLE_FLYCAPTURE_ERROR( "flycaptureStart()", fe ) )
		return false;

	// Set up some stereo parameters:
	// Set to 320x240 output images
	te = triclopsSetResolution( triclops, stereoHight, stereoWidth );
	if( ! _HANDLE_TRICLOPS_ERROR( "triclopsSetResolution()", te ) )
		return false;

	if(cvRtfImgCR!=NULL && (cvRtfImgCR->width!=stereoWidth || cvRtfImgCR->height!=stereoHight) )
	{
		cvReleaseImageHeader(&cvRtfImgCR);
		cvRtfImgCR = NULL;
	}
	if(cvRtfImgCR==NULL)	cvRtfImgCR = cvCreateImageHeader(cvSize(stereoWidth,stereoHight),8,4);

	if(cvRtfImgCL!=NULL && (cvRtfImgCL->width!=stereoWidth || cvRtfImgCL->height!=stereoHight) )
	{
		cvReleaseImageHeader(&cvRtfImgCL);
		cvRtfImgCL = NULL;
	}
	if(cvRtfImgCL==NULL)	cvRtfImgCL = cvCreateImageHeader(cvSize(stereoWidth,stereoHight),8,4);

	if(cvImgStereo!=NULL && (cvImgStereo->width!=stereoWidth||cvImgStereo->height!=stereoHight))
	{
		cvReleaseImage(&cvImgStereo);
		cvImgStereo = NULL;
	}

	if(cvImgStereo==NULL)	
	{
		cvImgStereo = cvCreateImage(cvSize(stereoWidth,stereoHight),8,1);
		cvSetZero(cvImgStereo);
	}
	//cvNot(cvImgStereo, cvImgStereo);

	// Set disparity range
	te = triclopsSetDisparity( triclops, minDisparity, maxDisparity );
	if( ! _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", te ) )
		return false;  

	// Lets turn off all validation except subpixel and surface
	// This works quite well
	te = triclopsSetTextureValidation( triclops, 0 );
	if( ! _HANDLE_TRICLOPS_ERROR( "triclopsSetTextureValidation()", te ) )
		return false;
	te = triclopsSetUniquenessValidation( triclops, 0 );
	if( ! _HANDLE_TRICLOPS_ERROR( "triclopsSetUniquenessValidation()", te ) )
		return false;

	// Turn on sub-pixel interpolation
	te = triclopsSetSubpixelInterpolation( triclops, 1 );
	if( ! _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", te ) )
		return false;

	// Grab an image from the camera
	fe = flycaptureGrabImage2( flycapture, &flycaptureImage );
	if( ! _HANDLE_FLYCAPTURE_ERROR( "flycaptureGrabImage()", fe ) )
		return false;

	// Extract information from the FlycaptureImage
	imageCols = flycaptureImage.iCols;
	imageRows = flycaptureImage.iRows;
	imageRowInc = flycaptureImage.iRowInc;
	iSideBySideImages = flycaptureImage.iNumImages;
	timeStampSeconds = flycaptureImage.timeStamp.ulSeconds;
	timeStampMicroSeconds = flycaptureImage.timeStamp.ulMicroSeconds;

	if(rowIntColor!=NULL)
	{
		delete[] rowIntColor;
		rowIntColor=NULL;
	}
	if(rowIntMono!=NULL)
	{
		delete[] rowIntMono;
		rowIntMono=NULL;
	}

	rowIntColor = new unsigned char[ imageCols * imageRows * iSideBySideImages * 4 ];
	rowIntMono = new unsigned char[ imageCols * imageRows * iSideBySideImages ];

	tempColorImage.pData = rowIntColor;
	tempMonoImage.pData = rowIntMono;

	//opencv
#ifdef GetRawOpencvImg
	if(cvImgCL!=NULL && (cvImgCL->width!=imageCols || cvImgCL->height!=imageRows) )
	{
		cvReleaseImage(&cvImgCL);
		cvImgCL = NULL;
	}
	if(cvImgCR!=NULL && (cvImgCR->width!=imageCols || cvImgCR->height!=imageRows) )
	{
		cvReleaseImage(&cvImgCR);
		cvImgCR = NULL;
	}
	if(cvImgC!=NULL && (cvImgC->width!=imageCols*iSideBySideImages || cvImgC->height!=imageRows) )
	{
		cvReleaseImageHeader(&cvImgC);
		cvImgC = NULL;
	}

	if(cvImgCL==NULL)	cvImgCL = cvCreateImage(cvSize(imageCols,imageRows),8,4);
	if(cvImgCR==NULL)	cvImgCR = cvCreateImage(cvSize(imageCols,imageRows),8,4);
	if(cvImgC==NULL)	cvImgC = cvCreateImageHeader(cvSize(imageCols*iSideBySideImages,imageRows),8,4);
#endif

	runStatus = true;
	return true;
}

void bb2_wrapper::showCameraInfo( )
{
	// Print the camera info to the screen for the user to validate:

	cout << fixed << setprecision( 2 );

	float baseline;
	te = triclopsGetBaseline( triclops, &baseline );
	_HANDLE_TRICLOPS_ERROR( "triclopsGetBaseline()", te );
	cout << "Baseline: " << baseline << " meters" << endl;

	float focalLength;
	te = triclopsGetFocalLength( triclops, &focalLength );
	_HANDLE_TRICLOPS_ERROR( "triclopsGetFocalLength()", te );
	cout << "Focal length: " << focalLength << " pixels" << endl;

	float centerRow;
	float centerCol;
	te = triclopsGetImageCenter( triclops, &centerRow, &centerCol );
	_HANDLE_TRICLOPS_ERROR( "triclopsGetImageCenter()", te );
	cout << "The center (along y direction): " << centerRow << " pixels" << endl;
	cout << "The center (along x direction): " << centerCol << " pixels" << endl;

	cout << "The CalFile: " << szCalFile << endl;
}

bool bb2_wrapper::StereoMatch( )
{
	if(runStatus)
	{		
		// Pointers to positions in the color buffer that correspond to the beginning
		// of the red, green and blue sections
		unsigned char* redColor = NULL;
		unsigned char* greenColor = NULL;
		unsigned char* blueColor = NULL; 

		redColor = rowIntColor;
		if ( flycaptureImage.iNumImages == 2 )
		{
			greenColor = redColor + ( 4 * imageCols );
			blueColor = redColor + ( 4 * imageCols );
		}

		if ( flycaptureImage.iNumImages == 3 )
		{
			greenColor = redColor + ( 4 * imageCols );
			blueColor = redColor + ( 2 * 4 * imageCols );
		}

		// Use the row interleaved images to build up a packed TriclopsInput.
		// A packed triclops input will contain a single image with 32 bpp.
		te = triclopsBuildPackedTriclopsInput( imageCols, imageRows, imageRowInc * 4,
			timeStampSeconds, timeStampMicroSeconds, greenColor, &colorData );
		_HANDLE_TRICLOPS_ERROR( "triclopsBuildPackedTriclopsInput()", te );

		// Rectify the color image
		TriclopsPackedColorImage  RcImage;
		te = triclopsRectifyPackedColorImage( triclops, TriCam_LEFT, 
			&colorData, &RcImage );
		_HANDLE_TRICLOPS_ERROR( "triclopsRectifyPackedColorImage()", te );

		cvRtfImgCL->imageData = (char*) RcImage.data;

		te = triclopsBuildPackedTriclopsInput( imageCols, imageRows, imageRowInc * 4,
			timeStampSeconds, timeStampMicroSeconds, redColor, &colorData );
		_HANDLE_TRICLOPS_ERROR( "triclopsBuildPackedTriclopsInput()", te );
		te = triclopsRectifyPackedColorImage( triclops, TriCam_RIGHT, 
			&colorData, &RcImage );
		_HANDLE_TRICLOPS_ERROR( "triclopsRectifyPackedColorImage()", te );

		cvRtfImgCR->imageData = (char*) RcImage.data;

		if(stereoStatus)
		{
			float	       x, y, z; 
			int		       r, g, b;			// Corresponding color pixel value of rectified color image
			int		       nPoints = 0;		// Number of successfully matched points                                      
			int		       pixelinc ;
			int		       i, j, k;
			unsigned short*     row;
			unsigned short      disparity;

			pc_seq.clear( );  // Reinitialize frame point clouds

			// Pointers to positions in the mono buffer that correspond to the beginning
			// of the red, green and blue sections
			unsigned char* redMono = NULL;
			unsigned char* greenMono = NULL;
			unsigned char* blueMono = NULL; 

			redMono = rowIntMono;
			if (flycaptureImage.iNumImages == 2)
			{
				greenMono = redMono + imageCols;
				blueMono = redMono + imageCols;
			}

			if (flycaptureImage.iNumImages == 3)
			{
				greenMono = redMono + imageCols;
				blueMono = redMono + ( 2 * imageCols );
			}

			// Use the row interleaved images to build up an RGB TriclopsInput.  
			// An RGB triclops input will contain the 3 raw images (1 from each camera).
			te = triclopsBuildRGBTriclopsInput( imageCols, imageRows, imageRowInc, timeStampSeconds, 
				timeStampMicroSeconds, redMono, greenMono, blueMono, &stereoData);
			_HANDLE_TRICLOPS_ERROR( "triclopsBuildRGBTriclopsInput()", te );

			// Preprocessing the images
			te = triclopsRectify( triclops, &stereoData );
			_HANDLE_TRICLOPS_ERROR( "triclopsRectify()", te );

			// Stereo processing
			te = triclopsStereo( triclops ) ;
			_HANDLE_TRICLOPS_ERROR( "triclopsStereo()", te );

			// Retrieve the interpolated depth image from the context
			te = triclopsGetImage16( triclops, TriImg16_DISPARITY, TriCam_REFERENCE, &depthImage16 );
			_HANDLE_TRICLOPS_ERROR( "triclopsGetImage16()", te );

			// Rectify the color image if applicable
			if ( pixelFormat == FLYCAPTURE_RAW16 )
			{
				te = triclopsRectifyColorImage( triclops, TriCam_REFERENCE, &colorData, &colorImage );
				_HANDLE_TRICLOPS_ERROR( "triclopsRectifyColorImage()", te );
			}
			else
			{
				te = triclopsGetImage( triclops, TriImg_RECTIFIED, TriCam_REFERENCE, &monoImage );
				_HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );
			}

			// Determine the number of pixels spacing per row
			pixelinc = depthImage16.rowinc/2;
			for ( i = 0, k = 0; i < depthImage16.nrows; i++ )
			{
				row     = depthImage16.data + i * pixelinc;
				for ( j = 0; j < depthImage16.ncols; j++, k++ )
				{
					disparity = row[j];

					// do not save invalid points
					if ( disparity < 0xFF00 )
					{
						// convert the 16 bit disparity value to floating point x,y,z
						triclopsRCD16ToXYZ( triclops, i, j, disparity, &x, &y, &z );

						// look at points within a range
						if ( z < 5.0 )
						{
							// rgb things
							if ( pixelFormat == FLYCAPTURE_RAW16 )
							{
								r = (int)colorImage.red[k];
								g = (int)colorImage.green[k];
								b = (int)colorImage.blue[k];		  
							}
							else
							{
								// For mono cameras, we 1just assign the same value to RGB
								r = (int)monoImage.data[k];
								g = (int)monoImage.data[k];
								b = (int)monoImage.data[k];
							}

							// Put the current pixel info into a PointCloud struct
							pc.x = x; pc.y = y; pc.z = z; 
							pc.r = r; pc.g = g; pc.b = b; 
							pc.row = i; pc.col = j;

							bufferFramePointClouds( );  // Whenever there is a point cloud coming, buffer it to the member vector.

							// If the current Point has the right indexes, then fill an interest point with this point info.
							sampleGroundPoints( );
							//bufferFifteenPointsDepth( );
							//bufferFifteenPoints3D( );
							//computeAvrgDepth( );
							//sampleNinePoints( );
							
							useGround( );

							// I guess this is only a qualitative representation of the relationship 
							// between depth and disparity, therefore cannot be directly used for 
							// calculating z.
							cvImgStereo->imageData[i*cvImgStereo->widthStep+j] = (int) (5-z)*255/5;
							nPoints++;
						}
						else
						{
							cvImgStereo->imageData[i*cvImgStereo->widthStep+j] = 0;
						}
					}
					else
					{
						cvImgStereo->imageData[i*cvImgStereo->widthStep+j] = 0;
					}
				}
			}
			
			//printf( "Points count: %d\n", nPoints );
			redMono = NULL;
			greenMono = NULL;
			blueMono = NULL;
		}

		// Delete the image buffer, it is not needed once the TriclopsInput
		// has been built		
		redColor = NULL;
		greenColor = NULL;
		blueColor = NULL;
		
		// A frame processed, a reference ground plane gotten.
		// TODO: this operation should not have been in here.
		genGroundPlane( );

		return true;
	}
	return false;
}

Mat bb2_wrapper::getDepthImage16( ) const
{
	Mat result( depthImage16.nrows, depthImage16.ncols, CV_16UC1 );

	for ( int r = 0; r < depthImage16.nrows; r++ )
	{
		unsigned short* rowPointer = depthImage16.data + r*depthImage16.rowinc/sizeof(unsigned short);
		for ( int c = 0; c < depthImage16.ncols; c++ )
		{
			result.at< ushort >( r, c ) = rowPointer[c];
		}
	}

	return result;
}

IplImage* bb2_wrapper::getStereo( ) const
{
	return cvImgStereo;
}

PointCloud bb2_wrapper::getPointCloud( ) const
{
	return pc;
}

void bb2_wrapper::bufferFifteenPointsDepth( )
{
	// First line
	if ( pc.col == stereoWidth / 4 - stereoWidth / 5 && pc.row == stereoHight / 4 )
	{
		fifteenPointsDepth[0] = pc.z;
	}

	if ( pc.col == stereoWidth / 4 && pc.row == stereoHight / 4 )
	{
		fifteenPointsDepth[1] = pc.z;
	}
	if ( pc.col == stereoWidth / 2 && pc.row == stereoHight / 4 )
	{
		fifteenPointsDepth[2] = pc.z;
	}
	if ( pc.col == ( stereoWidth - stereoWidth / 4 ) && pc.row == stereoHight / 4 )
	{
		fifteenPointsDepth[3] = pc.z;
	}

	if ( pc.col == stereoWidth - stereoWidth / 4 + stereoWidth / 5 && pc.row == stereoHight / 4 )
	{
		fifteenPointsDepth[4] = pc.z;
	}

	// Second line
	if ( pc.col == stereoWidth / 4 - stereoWidth / 5 && pc.row == stereoHight / 2 )
	{
		fifteenPointsDepth[5] = pc.z;
	}
	if ( pc.col == stereoWidth / 4 && pc.row == stereoHight / 2 )
	{
		fifteenPointsDepth[6] = pc.z;
	}
	if ( pc.col == ( stereoWidth / 2 ) && pc.row == stereoHight / 2 )
	{
		fifteenPointsDepth[7] = pc.z;
	}

	if ( pc.col == stereoWidth - stereoWidth / 4 && pc.row == stereoHight / 2 )
	{
		fifteenPointsDepth[8] = pc.z;
	}
	if ( pc.col == stereoWidth - stereoWidth / 4 + stereoWidth / 5 && pc.row == stereoHight / 2 )
	{
		fifteenPointsDepth[9] = pc.z;
	}

	// Third line
	if ( pc.col == stereoWidth / 4 - stereoWidth / 5 && pc.row == stereoHight - stereoHight / 4 )
	{
		fifteenPointsDepth[10] = pc.z;
	}
	if ( pc.col == stereoWidth / 4 && pc.row == stereoHight - stereoHight / 4 )
	{
		fifteenPointsDepth[11] = pc.z;
	}
	if ( pc.col == ( stereoWidth / 2 ) && pc.row == stereoHight - stereoHight / 4 )
	{
		fifteenPointsDepth[12] = pc.z;
	}

	if ( pc.col == stereoWidth - stereoWidth / 4 && pc.row == stereoHight - stereoHight / 4 )
	{
		fifteenPointsDepth[13] = pc.z;
	}
	if ( pc.col == stereoWidth - stereoWidth / 4 + stereoWidth / 5 && pc.row == stereoHight - stereoHight / 4 )
	{
		fifteenPointsDepth[14] = pc.z;
	}
}

void bb2_wrapper::bufferFifteenPoints3D( )
{
	// First line
	if ( pc.col == stereoWidth / 4 - stereoWidth / 5 && pc.row == stereoHight / 4 )
	{
		fifteenPoints3D[0].x = pc.x;
		fifteenPoints3D[0].y = pc.y;
		fifteenPoints3D[0].z = pc.z;
	}

	if ( pc.col == stereoWidth / 4 && pc.row == stereoHight / 4 )
	{
		fifteenPoints3D[1].x = pc.x;
		fifteenPoints3D[1].y = pc.y;
		fifteenPoints3D[1].z = pc.z;
	}
	if ( pc.col == stereoWidth / 2 && pc.row == stereoHight / 4 )
	{
		fifteenPoints3D[2].x = pc.x;
		fifteenPoints3D[2].y = pc.y;
		fifteenPoints3D[2].z = pc.z;
	}
	if ( pc.col == ( stereoWidth - stereoWidth / 4 ) && pc.row == stereoHight / 4 )
	{
		fifteenPoints3D[3].x = pc.x;
		fifteenPoints3D[3].y = pc.y;
		fifteenPoints3D[3].z = pc.z;
	}

	if ( pc.col == stereoWidth - stereoWidth / 4 + stereoWidth / 5 && pc.row == stereoHight / 4 )
	{
		fifteenPoints3D[4].x = pc.x;
		fifteenPoints3D[4].y = pc.y;
		fifteenPoints3D[4].z = pc.z;
	}

	// Second line
	if ( pc.col == stereoWidth / 4 - stereoWidth / 5 && pc.row == stereoHight / 2 )
	{
		fifteenPoints3D[5].x = pc.x;
		fifteenPoints3D[5].y = pc.y;
		fifteenPoints3D[5].z = pc.z;
	}
	if ( pc.col == stereoWidth / 4 && pc.row == stereoHight / 2 )
	{
		fifteenPoints3D[6].x = pc.x;
		fifteenPoints3D[6].y = pc.y;
		fifteenPoints3D[6].z = pc.z;
	}
	if ( pc.col == ( stereoWidth / 2 ) && pc.row == stereoHight / 2 )
	{
		fifteenPoints3D[7].x = pc.x;
		fifteenPoints3D[7].y = pc.y;
		fifteenPoints3D[7].z = pc.z;
	}

	if ( pc.col == stereoWidth - stereoWidth / 4 && pc.row == stereoHight / 2 )
	{
		fifteenPoints3D[8].x = pc.x;
		fifteenPoints3D[8].y = pc.y;
		fifteenPoints3D[8].z = pc.z;
	}
	if ( pc.col == stereoWidth - stereoWidth / 4 + stereoWidth / 5 && pc.row == stereoHight / 2 )
	{
		fifteenPoints3D[9].x = pc.x;
		fifteenPoints3D[9].y = pc.y;
		fifteenPoints3D[9].z = pc.z;
	}

	// Third line
	if ( pc.col == stereoWidth / 4 - stereoWidth / 5 && pc.row == stereoHight - stereoHight / 4 )
	{
		fifteenPoints3D[10].x = pc.x;
		fifteenPoints3D[10].y = pc.y;
		fifteenPoints3D[10].z = pc.z;
	}
	if ( pc.col == stereoWidth / 4 && pc.row == stereoHight - stereoHight / 4 )
	{
		fifteenPoints3D[11].x = pc.x;
		fifteenPoints3D[11].y = pc.y;
		fifteenPoints3D[11].z = pc.z;
	}
	if ( pc.col == ( stereoWidth / 2 ) && pc.row == stereoHight - stereoHight / 4 )
	{
		fifteenPoints3D[12].x = pc.x;
		fifteenPoints3D[12].y = pc.y;
		fifteenPoints3D[12].z = pc.z;
	}

	if ( pc.col == stereoWidth - stereoWidth / 4 && pc.row == stereoHight - stereoHight / 4 )
	{
		fifteenPoints3D[13].x = pc.x;
		fifteenPoints3D[13].y = pc.y;
		fifteenPoints3D[13].z = pc.z;
	}
	if ( pc.col == stereoWidth - stereoWidth / 4 + stereoWidth / 5 && pc.row == stereoHight - stereoHight / 4 )
	{
		fifteenPoints3D[14].x = pc.x;
		fifteenPoints3D[14].y = pc.y;
		fifteenPoints3D[14].z = pc.z;
	}
}

void bb2_wrapper::showFifteenPointsDepth( ) const
{
	cout << "Depth of the marks: " << endl;
	for ( int i = 0; i < 5; i++ )
	{
		cout << fifteenPointsDepth[i] << "m" << '\t';	
	}
	cout << endl;

	for ( int i = 5; i < 10; i++ )
	{
		cout << fifteenPointsDepth[i] << "m" << '\t';	
	}
	cout << endl;

	for ( int i = 10; i < 15; i++ )
	{
		cout << fifteenPointsDepth[i] << "m" << '\t';	
	}
	cout << endl;
	cout << "--------------------------------------------";
	cout << endl;
}


void bb2_wrapper::showFifteenPoints3D( ) const
{
	cout << fixed << setprecision( 2 );
	cout << "3-D coordinates of the marks: " << endl;
	
	for ( int i = 0; i < 5; i++ )
	{
		cout << "(" << fifteenPoints3D[i].x << "m, " << fifteenPoints3D[i].y << "m, " << fifteenPoints3D[i].z << "m)" << '\t';
	}
	cout << endl << endl;

	for ( int i = 5; i < 10; i++ )
	{
		cout << "(" << fifteenPoints3D[i].x << "m, " << fifteenPoints3D[i].y << "m, " << fifteenPoints3D[i].z << "m)" << '\t';
	}
	cout << endl << endl;

	for ( int i = 10; i < 15; i++ )
	{
		cout << "(" << fifteenPoints3D[i].x << "m, " << fifteenPoints3D[i].y << "m, " << fifteenPoints3D[i].z << "m)" << '\t';
	}
	cout << endl << endl;
	cout << "-----------------------------------------------------------------------------------------------------------------------";
	cout << endl;
}

void bb2_wrapper::computeAvrgDepth( )
{
	const int row_target = 240;


	// Depth that is smaller than it is considered to be too near.
	const float fixedDepth = 0.9f;

	if ( pc.row > row_target && pc.row < 470 && pc.col > 10 && pc.col < 210 )
	{
		
		{
			eP[0].avrgDepth += pc.z;
			eP[0].numOfPoints++;

			if ( pc.z < fixedDepth )  // Count the number of pixels with depth less than fixedDepth
			{
				eP[0].numPixelLessThanFixedDepthThreshold++;
			}
		}
	}

	if ( pc.row > row_target && pc.row < 470 && pc.col >= 210 && pc.col < 430 )
	{
		{
			eP[1].avrgDepth += pc.z;
			eP[1].numOfPoints++;

			if ( pc.z < fixedDepth )  // Count the number of pixels with depth less than fixedDepth
			{
				eP[1].numPixelLessThanFixedDepthThreshold++;
			}
		}
	}

	if ( pc.row > row_target && pc.row < 470 && pc.col >= 430 && pc.col < 630 )
	{
		
		{
			eP[2].avrgDepth += pc.z;
			eP[2].numOfPoints++;

			if ( pc.z < fixedDepth )  // Count the number of pixels with depth less than fixedDepth
			{
				eP[2].numPixelLessThanFixedDepthThreshold++;
			}
		}
	}
}

void bb2_wrapper::showAvrgDepth( )
{
	cout << "Counting info of Three windows: " << endl;
	cout << fixed << setprecision( 2 );

	cout << "Effective matches: " << eP[0].numOfPoints << ", " << eP[0].avrgDepth / eP[0].numOfPoints << "m, "<< "Near points: " << eP[0].numPixelLessThanFixedDepthThreshold << endl;	
	cout << "Effective matches: " << eP[1].numOfPoints << ", " << eP[1].avrgDepth / eP[1].numOfPoints << "m, "<< "Near points: " << eP[1].numPixelLessThanFixedDepthThreshold << endl;	
	cout << "Effective matches: " << eP[2].numOfPoints << ", " << eP[2].avrgDepth / eP[2].numOfPoints << "m, "<< "Near points: " << eP[2].numPixelLessThanFixedDepthThreshold << endl;	
	
	cout << endl;
	cout << "---------------------------------------------------------------------------------------";
	cout << endl;

	for ( int i  = 0; i < 3; i++ )
	{
		eP[i].avrgDepth = 0;
		eP[i].numOfPoints = 0;
	}
}

void bb2_wrapper::sampleNinePoints( )
{
	// The following samples are to be tuned according to practical test.
	// Sample 0
	if ( pc.row == stereoHight / 2 && pc.col == stereoWidth / 4 )
	{
		ninePoints[0].x = pc.x;
		ninePoints[0].y = pc.y;
		ninePoints[0].z = pc.z;
	}
	// Sample 1
	if ( pc.row == stereoHight / 2 && pc.col == stereoWidth / 2 )
	{
		ninePoints[1].x = pc.x;
		ninePoints[1].y = pc.y;
		ninePoints[1].z = pc.z;
	}
	// Sample 2
	if ( pc.row == stereoHight / 2 && pc.col == stereoWidth * 3 / 4 )
	{
		ninePoints[2].x = pc.x;
		ninePoints[2].y = pc.y;
		ninePoints[2].z = pc.z;
	}

	// Sample 3
	if ( pc.row == stereoHight * 5 / 8 && pc.col == stereoWidth / 4 )
	{
		ninePoints[3].x = pc.x;
		ninePoints[3].y = pc.y;
		ninePoints[3].z = pc.z;
	}
	// Sample 4
	if ( pc.row == stereoHight * 5 / 8 && pc.col == stereoWidth / 2 )
	{
		ninePoints[4].x = pc.x;
		ninePoints[4].y = pc.y;
		ninePoints[4].z = pc.z;
	}
	// Sample 5
	if ( pc.row == stereoHight * 5 / 8 && pc.col == stereoWidth * 3 / 4 )
	{
		ninePoints[5].x = pc.x;
		ninePoints[5].y = pc.y;
		ninePoints[5].z = pc.z;
	}

	// Sample 6
	if ( pc.row == stereoHight * 7 / 8 && pc.col == stereoWidth / 4 )
	{
		ninePoints[6].x = pc.x;
		ninePoints[6].y = pc.y;
		ninePoints[6].z = pc.z;
	}
	// Sample 7
	if ( pc.row == stereoHight * 7 / 8 && pc.col == stereoWidth / 2 )
	{
		ninePoints[7].x = pc.x;
		ninePoints[7].y = pc.y;
		ninePoints[7].z = pc.z;
	}
	// Sample 8
	if ( pc.row == stereoHight * 7 / 8 && pc.col == stereoWidth * 3 / 4 )
	{
		ninePoints[8].x = pc.x;
		ninePoints[8].y = pc.y;
		ninePoints[8].z = pc.z;
	}
}

vector< CvPoint3D32f > bb2_wrapper::getNinePoints( ) const
{
	return ninePoints;
}

void bb2_wrapper::sampleGroundPoints( )
{
	const int sampleRadius = 85;
	
	// --------------------------------------------------------------------------------------------------------------------------------
	// Point A( 320, 420 ) 
	const int colA = 320;
	const int rowA = 420;
	// If the current point cloud locates in the neighborhood of Point A, then it is considered.
	if ( pc.row > rowA - sampleRadius && pc.row < rowA + sampleRadius && pc.col > colA - sampleRadius && pc.col < colA + sampleRadius )
	{
		groundPlanePoints.planePointA.x += pc.x;
		groundPlanePoints.planePointA.y += pc.y;
		groundPlanePoints.planePointA.z += pc.z;
		groundPlanePoints.numOfEffectivePointsAourndA++;
	}

	// --------------------------------------------------------------------------------------------------------------------------------
	// Point B( 280, 460 ) 
	const int colB = 280;
	const int rowB = 460;
	// If the current point cloud locates in the neighborhood of Point A, then it is considered.
	if ( pc.row > rowB - sampleRadius && pc.row < rowB + sampleRadius && pc.col > colB - sampleRadius && pc.col < colB + sampleRadius )
	{
		groundPlanePoints.planePointB.x += pc.x;
		groundPlanePoints.planePointB.y += pc.y;
		groundPlanePoints.planePointB.z += pc.z;
		groundPlanePoints.numOfEffectivePointsAourndB++;
	}

	// --------------------------------------------------------------------------------------------------------------------------------
	// Point C( 360, 460 ) 
	const int colC = 360;
	const int rowC = 460;
	// If the current point cloud locates in the neighborhood of Point A, then it is considered.
	if ( pc.row > rowC - sampleRadius && pc.row < rowC + sampleRadius && pc.col > colC - sampleRadius && pc.col < colC + sampleRadius )
	{
		groundPlanePoints.planePointC.x += pc.x;
		groundPlanePoints.planePointC.y += pc.y;
		groundPlanePoints.planePointC.z += pc.z;
		groundPlanePoints.numOfEffectivePointsAourndC++;
	}
}

void bb2_wrapper::showSampledGround( )
{
	const float cameraHeight = 0.6f;
	const float frontFixedDistanceA = 1.65f;
	const float frontFixedDistanceBC = 1.5f;
	const float leftFixedDistanceBC = 0.1f; 

	// Show ground plane point A
	if ( groundPlanePoints.numOfEffectivePointsAourndA != 0 )
	{
		cout << "(" << groundPlanePoints.planePointA.x / groundPlanePoints.numOfEffectivePointsAourndA << "," 
			        << groundPlanePoints.planePointA.y / groundPlanePoints.numOfEffectivePointsAourndA << "," 
			        << groundPlanePoints.planePointA.z / groundPlanePoints.numOfEffectivePointsAourndA << ")" << endl;
	}
	else
	{
		groundPlanePoints.planePointA.x = 0; 
		groundPlanePoints.planePointA.y = -cameraHeight;
		groundPlanePoints.planePointA.z = frontFixedDistanceA;
	}
	
	// Show ground plane point B
	if ( groundPlanePoints.numOfEffectivePointsAourndB != 0 )
	{
		cout << "(" << groundPlanePoints.planePointB.x / groundPlanePoints.numOfEffectivePointsAourndB << "," 
			        << groundPlanePoints.planePointB.y / groundPlanePoints.numOfEffectivePointsAourndB << "," 
			        << groundPlanePoints.planePointB.z / groundPlanePoints.numOfEffectivePointsAourndB << ")" << endl;
	}
	else 
	{
		groundPlanePoints.planePointB.x = -leftFixedDistanceBC; 
		groundPlanePoints.planePointB.y = -cameraHeight;
		groundPlanePoints.planePointB.z = frontFixedDistanceBC;
	}
	// Show ground plane point C
	if ( groundPlanePoints.numOfEffectivePointsAourndC != 0 )
	{
		cout << "(" << groundPlanePoints.planePointC.x / groundPlanePoints.numOfEffectivePointsAourndC << "," 
			        << groundPlanePoints.planePointC.y / groundPlanePoints.numOfEffectivePointsAourndC << "," 
			        << groundPlanePoints.planePointC.z / groundPlanePoints.numOfEffectivePointsAourndC << ")" << endl;
	}
	else
	{
		groundPlanePoints.planePointC.x = leftFixedDistanceBC; 
		groundPlanePoints.planePointC.y = -cameraHeight;
		groundPlanePoints.planePointC.z = frontFixedDistanceBC;
	}

	groundPlanePoints.planePointA.x = 0;
	groundPlanePoints.planePointA.y = 0;
	groundPlanePoints.planePointA.z = 0;
	groundPlanePoints.numOfEffectivePointsAourndA = 0;

	groundPlanePoints.planePointB.x = 0;
	groundPlanePoints.planePointB.y = 0;
	groundPlanePoints.planePointB.z = 0;
	groundPlanePoints.numOfEffectivePointsAourndB = 0;

	groundPlanePoints.planePointC.x = 0;
	groundPlanePoints.planePointC.y = 0;
	groundPlanePoints.planePointC.z = 0;
	groundPlanePoints.numOfEffectivePointsAourndC = 0;
}

void bb2_wrapper::genGroundPlane( )
{
	// Represent the final reference plane

	// Predefined ground parameters
	// These params are due be be tuned according to practical road surface conditions.
	const float cameraHeight = 0.6f;
	const float frontFixedDistanceA = 1.65f;
	const float frontFixedDistanceBC = 1.5f;
	const float leftFixedDistanceBC = 0.1f; 
		
	// Update ground params when effective ground plane are detected
	if ( groundPlanePoints.numOfEffectivePointsAourndA != 0 && groundPlanePoints.numOfEffectivePointsAourndB != 0 && groundPlanePoints.numOfEffectivePointsAourndC != 0 )
	{
		float dx1 = groundPlanePoints.planePointB.x / groundPlanePoints.numOfEffectivePointsAourndB - groundPlanePoints.planePointA.x / groundPlanePoints.numOfEffectivePointsAourndA;
		float dy1 = groundPlanePoints.planePointB.y / groundPlanePoints.numOfEffectivePointsAourndB - groundPlanePoints.planePointA.y / groundPlanePoints.numOfEffectivePointsAourndA;
		float dz1 = groundPlanePoints.planePointB.z / groundPlanePoints.numOfEffectivePointsAourndB - groundPlanePoints.planePointA.z / groundPlanePoints.numOfEffectivePointsAourndA;
																																			  
		float dx2 = groundPlanePoints.planePointC.x / groundPlanePoints.numOfEffectivePointsAourndC - groundPlanePoints.planePointA.x / groundPlanePoints.numOfEffectivePointsAourndA;
		float dy2 = groundPlanePoints.planePointC.y / groundPlanePoints.numOfEffectivePointsAourndC - groundPlanePoints.planePointA.y / groundPlanePoints.numOfEffectivePointsAourndA;
		float dz2 = groundPlanePoints.planePointC.z / groundPlanePoints.numOfEffectivePointsAourndC - groundPlanePoints.planePointA.z / groundPlanePoints.numOfEffectivePointsAourndA;
		
		groundPlane.coefs_0 = dy1*dz2-dy2*dz1;
		groundPlane.coefs_1 = dz1*dx2-dz2*dx1;
		groundPlane.coefs_2 = dx1*dy2-dx2*dy1;
		groundPlane.coefs_3 = -groundPlane.coefs_0*groundPlanePoints.planePointA.x / groundPlanePoints.numOfEffectivePointsAourndA-groundPlane.coefs_1*groundPlanePoints.planePointA.y / groundPlanePoints.numOfEffectivePointsAourndA-groundPlane.coefs_2*groundPlanePoints.planePointA.z / groundPlanePoints.numOfEffectivePointsAourndA;
	}
#if 0
	else  // Predefined ground plane
	{
		//ground.planePointA.x = 0; 
		//ground.planePointA.y = -cameraHeight;
		//ground.planePointA.z = frontFixedDistanceA;

		//ground.planePointB.x = -leftFixedDistanceBC; 
		//ground.planePointB.y = -cameraHeight;
		//ground.planePointB.z = frontFixedDistanceBC;

		//ground.planePointC.x = leftFixedDistanceBC; 
		//ground.planePointC.y = -cameraHeight;
		//ground.planePointC.z = frontFixedDistanceBC;

		///*refPlane.A = ( ground.planePointB.y - ground.planePointA.y ) * ( ground.planePointB.z - ground.planePointA.z ) - ( ground.planePointB.z - ground.planePointA.z ) * ( ground.planePointC.y - ground.planePointA.y );
		//refPlane.B = ( ground.planePointC.x - ground.planePointA.x ) * ( ground.planePointB.z - ground.planePointA.z ) - ( ground.planePointB.x - ground.planePointA.x ) * ( ground.planePointC.z - ground.planePointA.z );
		//refPlane.C = ( ground.planePointB.x - ground.planePointA.x ) * ( ground.planePointC.y - ground.planePointA.y ) - ( ground.planePointC.x - ground.planePointA.x ) * ( ground.planePointB.y - ground.planePointA.y );
		//refPlane.planePoint = ground.planePointA;*/
		//
		//finalGroundPlane.dx1 = ground.planePointB.x - ground.planePointA.x;
		//finalGroundPlane.dy1 = ground.planePointB.y - ground.planePointA.y;
		//finalGroundPlane.dz1 = ground.planePointB.z - ground.planePointA.z;
		//								 
		//finalGroundPlane.dx2 = ground.planePointC.x - ground.planePointA.x;
		//finalGroundPlane.dy2 = ground.planePointC.y - ground.planePointA.y;
		//finalGroundPlane.dz2 = ground.planePointC.z - ground.planePointA.z;

		//finalGroundPlane.coefs_0 = finalGroundPlane.dy1*finalGroundPlane.dz2-finalGroundPlane.dy2*finalGroundPlane.dz1;
		//finalGroundPlane.coefs_1 = finalGroundPlane.dz1*finalGroundPlane.dx2-finalGroundPlane.dz2*finalGroundPlane.dx1;
		//finalGroundPlane.coefs_2 = finalGroundPlane.dx1*finalGroundPlane.dy2-finalGroundPlane.dx2*finalGroundPlane.dy1;
		//finalGroundPlane.coefs_3 = -finalGroundPlane.coefs_0*ground.planePointA.x / ground.numOfEffectivePointsAourndA-finalGroundPlane.coefs_1*ground.planePointA.y / ground.numOfEffectivePointsAourndA-finalGroundPlane.coefs_2*ground.planePointA.z / ground.numOfEffectivePointsAourndA;
	}
#endif

	// Reinitialize the ground points when counting work is finished
	groundPlanePoints.planePointA.x = 0;
	groundPlanePoints.planePointA.y = 0;
	groundPlanePoints.planePointA.z = 0;
	groundPlanePoints.numOfEffectivePointsAourndA = 0;

	groundPlanePoints.planePointB.x = 0;
	groundPlanePoints.planePointB.y = 0;
	groundPlanePoints.planePointB.z = 0;
	groundPlanePoints.numOfEffectivePointsAourndB = 0;

	groundPlanePoints.planePointC.x = 0;
	groundPlanePoints.planePointC.y = 0;
	groundPlanePoints.planePointC.z = 0;
	groundPlanePoints.numOfEffectivePointsAourndC = 0;
}

// Use the previously modeled ground plane
void bb2_wrapper::useGround( )
{	
	// Thresholds
	const int row_target = 240;  // Below this row are interest area.

	// This threshold is used to tell whether a point cloud is effective,
	// because when the sum of the colors of a point cloud is too small,
	// we can safely tell that a wrong match has happened.
	const int rgbSum = 3; 

	const float belongToGroundThreshold = 0.03f;  // Unit: meter
	// -------------------------------------------------------------------

	// Normalize the norm vector of ground plane
	float normed_vector = 1 / sqrt( groundPlane.coefs_0 *groundPlane.coefs_0 
	                	          + groundPlane.coefs_1 *groundPlane.coefs_1
								  + groundPlane.coefs_2 *groundPlane.coefs_2 );

	if ( pc.row > row_target && pc.row < stereoHight && pc.col >= 0 && pc.col < stereoWidth )
	{
		if ( pc.b + pc.g + pc.r > rgbSum )  // No holes exist around here
		{
			float distanceToGround = abs( groundPlane.coefs_0 * pc.x
				                        + groundPlane.coefs_1 * pc.y
						                + groundPlane.coefs_2 * pc.z 
						                + groundPlane.coefs_3 ) * normed_vector;
			if ( distanceToGround < belongToGroundThreshold )
			{
				cv::circle( groundImg, cv::Point( pc.col, pc.row ), 1, cv::Scalar( 0, 255, 0 ), 1 );
			}
		}
	}
}

void bb2_wrapper::bufferFramePointClouds( )
{
	// Constraints
	const int row_target = 240;
	if ( pc.row > row_target )
	{
		pc_seq.push_back( pc );
	}
}

vector< PointCloud > bb2_wrapper::getFramePointClouds( ) const
{
	return pc_seq;
}
