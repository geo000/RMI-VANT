/****************************************************************************
 National Institute For Space Research - INPE, Brazil
 ----------------------------------------------------------------------------
 Name        : TePDIMosaic_test.cpp 

 Authors     : Marcos Eduardo Gomes Borges <marcoseborges@gmail.com>
               Marina Laís da Silva Nascimento <marina.lsnascimento@gmail.com>
               Juliano Elias Cardoso Cruz <juliano.ecc@gmail.com>

 Description : Registration and Mosaic of Images acquired by UAV using TerraLib
*****************************************************************************/

#define TEAGN_ENABLE_STDOUT_LOG

/****************************************************************************
 1.  INCLUDE FILES
 1.1 Standard include files
*****************************************************************************/
#include <TePDIExamplesBase.hpp>
#include <TePDIUtils.hpp>
#include <TeAgnostic.h>

/****************************************************************************
 1.2 Application include files
*****************************************************************************/
// Matching
#include <TePDIMMIOMatching.hpp>
#include <TeGTParams.h>
#include <TeDefines.h>
#include <math.h>

// Mosaic
#include <TePDIBlender.hpp>
#include <TePDIParameters.hpp>
#include <TePDIGeoMosaic.hpp>
#include <TePDIBatchGeoMosaic.hpp>
#include <TePDITPMosaic.hpp>
#include <TeInitRasterDecoders.h>

#include <vector>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

#include <string>
#include <iostream>

#include <algorithm>

/****************************************************************************
 2.  DECLARATIONS
 2.1 Internal constants
*****************************************************************************/
#ifndef M_PI
  #define M_PI   3.14159265358979323846
#endif

#ifndef M_PI_2
  #define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
  #define M_PI_4     0.785398163397448309616
#endif

/****************************************************************************
 2.2 Internal macros
*****************************************************************************/

/****************************************************************************
 2.3 Internal type definitions
*****************************************************************************/

/****************************************************************************
 2.4 Global variables (declared as 'extern' in some header file)
*****************************************************************************/

/****************************************************************************
 2.5 Internal function prototypes (defined in Section 4)
*****************************************************************************/
TeGTParams matching(
	 TePDITypes::TePDIRasterPtrType input_image1_ptr,
	 TePDITypes::TePDIRasterPtrType input_image2_ptr,
	 TeSharedPtr< TeCoordPairVect > &out_tie_points_ptr_
);

void mosaic(
	 TePDITypes::TePDIRasterPtrType input_raster1,
	 TePDITypes::TePDIRasterPtrType input_raster2,
	 TePDITypes::TePDIRasterPtrType output_raster,
	 TeGTParams trans_params
);

TePDITypes::TePDIRasterPtrType loadRaster( std::string raster );

void raster2Jpeg(
    const TePDITypes::TePDIRasterPtrType& input_raster_ptr,
    unsigned int raster_channels,
    const std::string& out_file_name,
    TeSharedPtr< TeCoordPairVect > out_tie_points_ptr,
    unsigned int tie_points_space
);


/****************************************************************************
 2.6 Funcao para ler imagens do diretorio
*****************************************************************************/
int getdir (string dir, vector<string> &files, string typeFile)
{
    DIR *dp;
    struct dirent *dirp;
    string imagem = "";
    bool isImagemJPG, tipoEscolhido = false;  

   if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }
 
    
    while ((dirp = readdir(dp)) != NULL) {
 	 imagem = dirp->d_name;
	//somente imagens com extensao JPG
	 isImagemJPG = imagem.find(".JPG") != string::npos;
	//IMG ou MOSAIC
	 tipoEscolhido = imagem.find(typeFile) != string::npos;

	 if(isImagemJPG && tipoEscolhido){
	      files.push_back(string(imagem));
	 }
     
    }

   sort(files.begin(), files.end());
    closedir(dp);
    return 0;
}

/****************************************************************************
 3. MAIN FUNCTION
*****************************************************************************/
int main()
{
  time_t init_time;
  time_t end_time;
  
  TePDITypes::TePDIRasterPtrType input_image1_ptr;
  TePDITypes::TePDIRasterPtrType input_image2_ptr;
  TePDITypes::TePDIRasterPtrType output_image;
  TeSharedPtr< TeCoordPairVect > out_tie_points_ptr;
  TeGTParams gt_params;
  
  TEAGN_TRUE_OR_THROW(
        TePDIUtils::TeAllocRAMRaster( output_image, 1, 1, 1, false, TeUNSIGNEDCHAR, 0 ),
        "ogetdirutput_raster Alloc  error"
  );

  // Diretorio das imagens  
  string dir = TEPDIEXAMPLESRESPATH "mosaic/";
  // Vetores para lista de imagens Mosaic  
  vector<string> filesMosaic = vector<string>();
  // Vetores para lista de imagens IMG  
  vector<string> filesIMG = vector<string>();
  
 //Chamada da funcao para ler imagens do diretorio. 
  getdir(dir,filesMosaic,"Mosaic");
  getdir(dir,filesIMG,"IMG"); 


  
  int numberOfMosaics = 3;

  
  for( int mosaicNumber = 0; mosaicNumber < numberOfMosaics; ++mosaicNumber)
  {
  /******************* Matching and Mosaic Beginning *******************/
          init_time = clock() / CLOCKS_PER_SEC;
          
   
	for (unsigned int j = 0;j < filesIMG.size();j++) {
		//Lista as imagens
		cout << filesMosaic[filesMosaic.size() - 1] << endl;
		cout << filesIMG[j] << endl;
		//carrega as imagens

		input_image1_ptr = loadRaster( filesMosaic[filesMosaic.size() - 1] );
		input_image2_ptr = loadRaster( filesIMG[j] );
		// atualiza a lista de Mosaic
		getdir(dir,filesMosaic,"Mosaic");

           
          TEAGN_LOGMSG( "Matching started" );
          gt_params = matching( input_image1_ptr, input_image2_ptr, out_tie_points_ptr );
          
          TEAGN_LOGMSG( "Mosaic started" );
          mosaic( input_image1_ptr, input_image2_ptr, output_image, gt_params );
          
          end_time = clock() / CLOCKS_PER_SEC;
          TEAGN_LOGMSG(
          	"Total elapsed time: " +
          	TeAgnostic::to_string( (long int)( end_time - init_time ) ) +
          	" seconds"
          );
          /*******************    Matching and Mosaic End    *******************/
          
          TEAGN_LOGMSG( "Saving Images with Tie-points" );
          raster2Jpeg(
                input_image1_ptr, 3,
                TEPDIEXAMPLESRESPATH "mosaic/Tie_Points" + Te2String(mosaicNumber) + "a.JPG", out_tie_points_ptr, 0
          );
          raster2Jpeg(
                input_image2_ptr, 3,
                TEPDIEXAMPLESRESPATH "mosaic/Tie_Points" + Te2String(mosaicNumber) + "b.JPG", out_tie_points_ptr, 1
          );
          
          TEAGN_LOGMSG( "Saving Mosaic Result" );
          TEAGN_TRUE_OR_THROW(
          	TePDIUtils::TeRaster2Jpeg(
          	        output_image,
          	        TEPDIEXAMPLESRESPATH "mosaic/Mosaic" + Te2String(mosaicNumber) + ".JPG",
          	        false, 100
          	),
          	"JPEG generation error"
          );
  } 
}
  
  return EXIT_SUCCESS;
}

/****************************************************************************
 4. Internal function (prototypes defined in section 2.5)
*****************************************************************************/

/*******************************************************************************
* matching() :: The required parameters are:
*-------------------------------------------------------------------------------
* input_image1_ptr      (TePDITypes::TePDIRasterPtrType) - The input image 1
* input_channel1        ( unsigned int ) - Band to process from input_image1
*
* input_image2_ptr      (TePDITypes::TePDIRasterPtrType) - The input image 2
* input_channel2        ( unsigned int ) - Band to process from input_image2
*
* out_tie_points_ptr    ( TeSharedPtr< TeCoordPairVect > ) output tie-points
*******************************************************************************/

TeGTParams matching(
	 TePDITypes::TePDIRasterPtrType input_image1_ptr,
	 TePDITypes::TePDIRasterPtrType input_image2_ptr,
	 TeSharedPtr< TeCoordPairVect > &out_tie_points_ptr_
	 )
{
  // Creating parameters
  TePDIParameters params;
  
  params.SetParameter( "input_image1_ptr" , input_image1_ptr );
  params.SetParameter( "input_channel1" , (unsigned int)0 );
  params.SetParameter( "input_channel1" , (unsigned int)1 );
  params.SetParameter( "input_channel1" , (unsigned int)2 );
    
  params.SetParameter( "input_image2_ptr" , input_image2_ptr );
  params.SetParameter( "input_channel2" , (unsigned int)0 );
  params.SetParameter( "input_channel2" , (unsigned int)1 );
  params.SetParameter( "input_channel2" , (unsigned int)2 );
  
  TeSharedPtr< TeCoordPairVect > out_tie_points_ptr( new TeCoordPairVect );
  params.SetParameter( "out_tie_points_ptr" , out_tie_points_ptr );
  
  /*****************************************************************************
  * Features matching method (default: TePDIMMIOMatching::NormCrossCorrMethod)
  *****************************************************************************/
  params.SetParameter( "matching_method", TePDIMMIOMatching::NormCrossCorrMethod );
  
  /*****************************************************************************
  * The geometric transformation parameters to be used.
  *****************************************************************************/
  TeGTParams gt_params;
  
  /*****************************************************************************
  * Transformations
  *-----------------------------------------------------------------------------
  * affine              2D affine geometric trasformation
  * 2ndDegPolinomial    Second degree polinomial model geometric trasformation
  * projective          Projective geometric trasformation
  *****************************************************************************/
  gt_params.transformation_name_ = "2ndDegPolinomial";
  
  /*****************************************************************************
  * Outliers remotion strategy
  *-----------------------------------------------------------------------------
  * NoOutRemotion        No outliers remotion applied
  * ExaustiveOutRemotion Exaustive outliers remotion
  * LWOutRemotion        Iteractive leave-worse-out) will remotion be performed
  * RANSACRemotion       Random Sample Consensus based outliers remotion
  *****************************************************************************/
  gt_params.out_rem_strat_       = TeGTParams::RANSACRemotion;
  
  gt_params.max_dmap_error_      = 5; // Direct mapping error
  gt_params.max_imap_error_      = 5; // Inverse mapping error
  gt_params.max_dmap_rmse_       = DBL_MAX; // Direct mapping mean square error
  gt_params.max_imap_rmse_       = DBL_MAX; // Inverse mapping mean square error
  
  params.SetParameter( "gt_params" , gt_params );
  
  cout << "Imagem 1: " << endl;
  cout << input_image1_ptr->params().ncols_ - 1 << endl;
  cout << input_image1_ptr->params().nlines_ - 1 << endl;
  cout << "Imagem 2: " << endl;
  cout << input_image2_ptr->params().ncols_ - 1 << endl;
  cout << input_image2_ptr->params().nlines_ - 1 << endl;
  
  
/*  TeBox input_box1( 
    TeCoord2D( 0, 4000 ) , 
    TeCoord2D( 2000, 0 ) );
  params.SetParameter( "input_box1" , input_box1 );

  TeBox input_box2( 
    TeCoord2D( 0, 3998 ) , 
    TeCoord2D( 2998, 0 ) );
  params.SetParameter( "input_box2" , input_box2 );   
*/
//  params.SetParameter( "skip_geom_filter" , (int)1 );
  
  /*****************************************************************************
  * The pixel resolution relation
  *-----------------------------------------------------------------------------
  * pixel_x_relation = img1_pixel_res_x / img2_pixel_res_x (default=1.0)
  * pixel_y_relation = img1_pixel_res_y / img2_pixel_res_y (default=1.0)
  *****************************************************************************/
  params.SetParameter( "pixel_x_relation" , (double)1 );
  params.SetParameter( "pixel_y_relation" , (double)1 );
//  params.SetParameter( "pixel_x_relation" , input_image1_ptr->params().resx_ / input_image2_ptr->params().resx_ );
//  params.SetParameter( "pixel_y_relation" , input_image1_ptr->params().resy_ / input_image2_ptr->params().resy_ );
  
		  params.SetParameter( "enable_multi_thread" , (int)1 );
//  params.SetParameter( "enable_threaded_raster_read", (int)1 );
  
  params.SetParameter( "max_tie_points" , (unsigned int)4000 );
  params.SetParameter( "corr_window_width" , (unsigned int)45 );
  params.SetParameter( "moravec_window_width" , (unsigned int)11 );
  
  TePDIMMIOMatching match_instance;

  TEAGN_TRUE_OR_THROW( match_instance.Reset( params ), "Algorithm reset error" );
  
  TEAGN_TRUE_OR_THROW( match_instance.Apply(), "Algorithm apply error" );
  
  TEAGN_LOGMSG( "Load Tie-points" );
  TeCoordPair auxPair;
  TeCoordPairVect::iterator it     = out_tie_points_ptr->begin();
  TeCoordPairVect::iterator it_end = out_tie_points_ptr->end();
  while( it != it_end ) {
  	auxPair.pt1.setXY( it->pt1.x(), it->pt1.y() ); // point over input image 1
  	auxPair.pt2.setXY( it->pt2.x(), it->pt2.y() ); // the corresponding point over input image 2
  	gt_params.tiepoints_.push_back( auxPair );
  	
        std::cout << "[" + Te2String( it->pt1.x(),1 ) + " , " +
        Te2String( it->pt1.y(),1 ) + "] -> [" +
        Te2String( it->pt2.x(),1 ) + " , " + 
        Te2String( it->pt2.y(),1 ) + "]" << std::endl;
        
  	++it;
  }
  
  out_tie_points_ptr_ = out_tie_points_ptr;
    
  return( gt_params );
}

/*******************************************************************************
* mosaic() :: The required parameters are:
*-------------------------------------------------------------------------------
* input_raster1         Input raster 1 (this will be the reference raster)
* input_raster2         Input raster 2
* channels1             The channels to process from input_raster1
* channels2             The channels to process from input_raster2
* output_raster         Output raster
* trans_params          Geometric transformation parameters
*******************************************************************************/
void mosaic(
	 TePDITypes::TePDIRasterPtrType input_raster1,
	 TePDITypes::TePDIRasterPtrType input_raster2,
	 TePDITypes::TePDIRasterPtrType output_raster,
	 TeGTParams trans_params
	 )
{     
  // Config Channels
  std::vector< unsigned int > channels1;
  channels1.push_back( 0 );
  channels1.push_back( 1 );
  channels1.push_back( 2 );
  
  std::vector< unsigned int > channels2;
  channels2.push_back( 0 );
  channels2.push_back( 1 );
  channels2.push_back( 2 );
    
  // Creating algorithm parameters
  TePDIParameters params;
  params.SetParameter( "input_raster1", input_raster1 );
  params.SetParameter( "input_raster2", input_raster2 );
  params.SetParameter( "channels1", channels1 );
  params.SetParameter( "channels2", channels2 );
  params.SetParameter( "output_raster", output_raster );
  params.SetParameter( "trans_params", trans_params );
  
  /*****************************************************************************
  * Blend Method (default: TePDIBlender::NoBlendMethod)
  *-----------------------------------------------------------------------------
  * TePDIBlender::NoBlendMethod         No blending performed
  * TePDIBlender::MeanBlendMethod       Mean of overlapped pixels method
  * TePDIBlender::EuclideanBlendMethod  Euclidean distance based blending method
  *****************************************************************************/
  params.SetParameter( "blend_method", TePDIBlender::NoBlendMethod );
  
  /*****************************************************************************
  * Interpolation method (default TePDIInterpolator::NNMethod)
  *-----------------------------------------------------------------------------
  * TePDIInterpolator::NNMethod         Near neighborhood interpolation method
  * TePDIInterpolator::BilinearMethod   Bilinear interpolation method
  * TePDIInterpolator::BicubicMethod    Bicubic interpolation method
  *****************************************************************************/
  params.SetParameter( "interp_method", TePDIInterpolator::NNMethod );
  
  /*****************************************************************************
  * A dummy pixel value for use in pixels where no data is available
  *****************************************************************************/
  //params.SetParameter( "dummy_value", (double)0 );
  
  /*****************************************************************************
  * If present auto-equalizing will be made (using overlap area reference)
  *****************************************************************************/  
  params.SetParameter( "auto_equalize", (int)1 );
  
  TePDITPMosaic mos;
  TEAGN_TRUE_OR_THROW( mos.Apply( params ), "Apply error" );
}

/****************************************************************************
* Load raster
*****************************************************************************/
TePDITypes::TePDIRasterPtrType loadRaster( std::string raster )
{
  TEAGN_LOGMSG( "Load " + raster );
  
  TePDITypes::TePDIRasterPtrType input_image_ptr;
  
  TEAGN_TRUE_OR_THROW(
  	TePDIUtils::loadRaster( 
  		std::string( TEPDIEXAMPLESRESPATH "mosaic/" + raster ),
  		input_image_ptr, true
  	),
  	"Error loading raster"
  );
  
  return( input_image_ptr );
}


/****************************************************************************
* Draw tie-points in raster and save to jpeg
*****************************************************************************/
void raster2Jpeg(
    const TePDITypes::TePDIRasterPtrType& input_raster_ptr,
    unsigned int raster_channels,
    const std::string& out_file_name,
    TeSharedPtr< TeCoordPairVect > out_tie_points_ptr,
    unsigned int tie_points_space
    )
{
  TEAGN_TRUE_OR_THROW( ( ! out_file_name.empty() ), "Invalid file name" )
  TEAGN_TRUE_OR_THROW( ( input_raster_ptr->params().nlines_ > 0 ), "Invalid matrix lines" )
  TEAGN_TRUE_OR_THROW( ( input_raster_ptr->params().ncols_ > 0 ), "Invalid matrix cols" )
    
  TeRasterParams params;
  params.setNLinesNColumns( input_raster_ptr->params().nlines_, input_raster_ptr->params().ncols_ );
  params.nBands( raster_channels );
  params.setDataType( TeUNSIGNEDCHAR, -1 );
  params.setPhotometric( TeRasterParams::TeRGB );
  params.decoderIdentifier_ = "JPEG";
  params.mode_ = 'c';
  params.fileName_ = out_file_name;
  
  TeRaster out_raster( params );
  TEAGN_TRUE_OR_THROW( out_raster.init(), "Error init raster" );
  double value = 0;
  
  for( int line = 0 ; line < input_raster_ptr->params().nlines_ ; ++line ) {
    for( int col = 0 ; col < input_raster_ptr->params().ncols_ ; ++col ) {
      for( int channel = 0; channel < raster_channels; ++channel ) {
        input_raster_ptr->getElement( col, line, value, channel );
        TEAGN_TRUE_OR_THROW( out_raster.setElement( col, line, value, channel ), "Error writing raster" )
      }
    }
  }
  
  /* Draw tie-points */
  
  if( out_tie_points_ptr.isActive() ) {
    TeCoordPairVect::iterator it = out_tie_points_ptr->begin();
    TeCoordPairVect::iterator it_end = out_tie_points_ptr->end();
    
    while( it != it_end ) {
      int x = 0;
      int y = 0;
          
      if( tie_points_space == 0 ) {
        x = TeRound( it->pt1.x() );
        y = TeRound( it->pt1.y() );
      } else {
        x = TeRound( it->pt2.x() );
        y = TeRound( it->pt2.y() );
      }
  
      TEAGN_TRUE_OR_THROW( ( x < input_raster_ptr->params().ncols_ ), "Invalid maxima column" )
      TEAGN_TRUE_OR_THROW( ( x >= 0 ), "Invalid maxima column" )
      TEAGN_TRUE_OR_THROW( ( y < input_raster_ptr->params().nlines_ ), "Invalid maxima line" )
      TEAGN_TRUE_OR_THROW( ( y >= 0 ), "Invalid maxima line" )
      
      for( int x1 = x -10; x1 <= x + 10; x1++ )
      {
        for( int y1 = y -10; y1 <= y + 10; y1++ )
        {
          TEAGN_TRUE_OR_THROW( out_raster.setElement( x1, y1, 0, 0 ), "Error writing raster" )
          TEAGN_TRUE_OR_THROW( out_raster.setElement( x1, y1, 255, 1 ), "Error writing raster" )
          TEAGN_TRUE_OR_THROW( out_raster.setElement( x1, y1, 0, 2 ), "Error writing raster" )
        }
      }
            
      ++it;
    }
  }
}

