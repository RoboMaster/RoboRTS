//--------------------------------------------------------------- 
/** 
\file      DxImageProc.h
\brief     Image Processing Library
\version   v1.0.1511.9161 
\date      2015-11-16
\author    Software Department 
<p>Copyright (c) 2012-2015 and all right reserved.</p> 
*/ 
//--------------------------------------------------------------- 

#if !defined (_DXIMAGEPROC_H)         
#define _DXIMAGEPROC_H 			///< pre-compiled macro define

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32                   
#include "stdafx.h"
#define DHDECL __stdcall

#else                          
#include <stdlib.h>
#include <string.h>
#define  DHDECL
#endif

typedef char            VxInt8;     
typedef short           VxInt16;     
typedef int             VxInt32;     
#ifdef _WIN32 
typedef __int64         VxInt64;     
#else
typedef long long       VxInt64;     
#endif
typedef unsigned char   VxUint8;     
typedef unsigned short  VxUint16;    
typedef unsigned int    VxUint32;    

/// status  definition
typedef enum  tagDX_STATUS
{
	DX_OK                         = 0,    
	DX_PARAMETER_INVALID          = -101, 
	DX_PARAMETER_OUT_OF_BOUND     = -102, 
	DX_NOT_ENOUGH_SYSTEM_MEMORY   = -103, 
	DX_NOT_FIND_DEVICE            = -104, 
	DX_STATUS_NOT_SUPPORTED       = -105,
	DX_CPU_NOT_SUPPORT_ACCELERATE = -106  
} DX_STATUS;

/// Bayer layout
typedef enum  tagDX_PIXEL_COLOR_FILTER
{
	NONE    = 0,   
	BAYERRG = 1,   
	BAYERGB = 2,   
	BAYERGR = 3,   
	BAYERBG = 4    
} DX_PIXEL_COLOR_FILTER;

/// image interpolation method
typedef enum tagDX_BAYER_CONVERT_TYPE
{
	RAW2RGB_NEIGHBOUR  = 0,   
	RAW2RGB_ADAPTIVE   = 1,   
	RAW2RGB_NEIGHBOUR3 = 2    
} DX_BAYER_CONVERT_TYPE;

/// image valid bit
typedef enum tagDX_VALID_BIT
{ 
	DX_BIT_0_7	    = 0,    ///< bit 0~7
	DX_BIT_1_8	    = 1,    ///< bit 1~8
	DX_BIT_2_9	    = 2,    ///< bit 2~9
	DX_BIT_3_10	    = 3,    ///< bit 3~10
	DX_BIT_4_11	    = 4     ///< bit 4~11
} DX_VALID_BIT;

/// image actual bits
typedef enum tagDX_ACTUAL_BITS
{
	DX_ACTUAL_BITS_10 = 10,    ///< 10bit
	DX_ACTUAL_BITS_12 = 12,    ///< 12bit
	DX_ACTUAL_BITS_14 = 14,    ///< 14bit
	DX_ACTUAL_BITS_16 = 16     ///< 16bit
} DX_ACTUAL_BITS;

///  image mirror method
typedef enum DX_IMAGE_MIRROR_MODE
{
	HORIZONTAL_MIRROR = 0,     
	VERTICAL_MIRROR   = 1     
}DX_IMAGE_MIRROR_MODE;

/// mono8 image process struct 
typedef  struct  MONO_IMG_PROCESS
{
	bool            bDefectivePixelCorrect;   
	bool            bSharpness;               
	bool            bAccelerate;               
	float           fSharpFactor;              
	VxUint8         *pProLut;                  
	VxUint16        nLutLength;                
	VxUint8         arrReserved[32];           
} MONO_IMG_PROCESS;

/// Raw8 Image process struct 
typedef  struct  COLOR_IMG_PROCESS
{
	bool                   bDefectivePixelCorrect; 
	bool                   bDenoise;              
	bool                   bSharpness;             
	bool                   bAccelerate;            
	VxInt16                *parrCC;               
	VxUint8                nCCBufLength;           
	float                  fSharpFactor;          
	VxUint8                *pProLut;              
	VxUint16               nLutLength;            
    DX_BAYER_CONVERT_TYPE  cvType;                 
	DX_PIXEL_COLOR_FILTER  emLayOut;               
	bool                   bFlip;                  
	VxUint8                arrReserved[32];        
} COLOR_IMG_PROCESS;

//--------------------------------------------------
/**
\brief  Convert Raw8 to Rgb24
\param  pInputBuffer   	[in] input buffer 
\param  pOutputBuffer   [out]output buffer(new buffer)
\param  nWidth  	    [in] image width
\param  nHeight   	    [in] image height
\param  cvtype          [in] Bayer convert type 
\param  nBayerType      [in] pixel color filter 
\param  bFlip           [in] output image flip or not, true:flip false:not flip

\return emStatus  
*/
//--------------------------------------------------
VxInt32 DHDECL DxRaw8toRGB24(void *pInputBuffer, void *pOutputBuffer, VxUint32 nWidth, VxUint32 nHeight, DX_BAYER_CONVERT_TYPE cvtype, DX_PIXEL_COLOR_FILTER nBayerType, bool bFlip);

//--------------------------------------------------
/**
\brief  Convert Raw12Packed to Raw16
\param  pInputBuffer   	[in] input Buffer 
\param  pOutputBuffer   [out]output Buffer(new buffer)
\param  nWidth          [in] image width
\param  nHeight         [in] image height                      

\return emStatus  
*/
//--------------------------------------------------
VxInt32 DHDECL DxRaw12PackedToRaw16(void* pInputBuffer, void* pOutputBuffer, VxUint32 nWidth, VxUint32 nHeight);

//-------------------------------------------------
/**
\brief  Convert Raw10Packed to Raw16
\param  pInputBuffer   	[in] input buffer 
\param  pOutputBuffer   [out]output buffer(new buffer)
\param  nWidth          [in] image width
\param  nHeight         [in] image height 

\return emStatus
*/
//-------------------------------------------------
VxInt32 DHDECL DxRaw10PackedToRaw16(void* pInputBuffer, void* pOutputBuffer, VxUint32 nWidth, VxUint32 nHeight);

//------------------------------------------------
/**
\brief  To rotate the 8-bit image clockwise by 90 degrees
\param  pInputBuffer  	[in] input buffer 
\param  pOutputBuffer	[out]output buffer(new buffer)  
\param  nWidth        	[in] image width
\param  nHeight       	[in] image height 

\return emStatus
*/
//------------------------------------------------
VxInt32 DHDECL DxRotate90CW8B(void* pInputBuffer, void* pOutputBuffer, VxUint32 nWidth, VxUint32 nHeight);

//------------------------------------------------
/**
\brief  To rotate the 8-bit image counter clockwise by 90 degrees
\param  pInputBuffer    [in] input buffer 
\param  pOutputBuffer	[out]output buffer(new buffer) 
\param  nWidth          [in] image width
\param  nHeight         [in] image height 

\return emStatus
*/
//------------------------------------------------
VxInt32 DHDECL DxRotate90CCW8B(void* pInputBuffer, void* pOutputBuffer, VxUint32 nWidth, VxUint32 nHeight);

//-----------------------------------------------
/**
\brief  Brightness adjustment (RGB24 or gray image of 8-bit)
\param  pInputBuffer  	[in] input buffer 
\param  pOutputBuffer	[out]output buffer 
\param  nImagesize    	[in] image size,unit:byte(RGB:width * height * 3)
\param  nFactor        	[in] factor,range(-150~150)

\return emStatus
*/
//-----------------------------------------------
VxInt32 DHDECL DxBrightness(void* pInputBuffer, void* pOutputBuffer, VxUint32 nImagesize, VxInt32 nFactor);

//--------------------------------------------------
/**
\brief  Contrast adjustment(RGB24 or gray image of 8-bit)
\param  pInputBuffer	[in] input buffer      
\param  pOutputBuffer	[out]output buffer
\param  nImagesize      [in] image size,unit:byte(RGB:width * height * 3)
\param  nFactor	        [in] factor,range(-50~100)

\return emStatus
*/
//--------------------------------------------------
VxInt32 DHDECL DxContrast(void* pInputBuffer, void* pOutputBuffer, VxUint32 nImagesize, VxInt32 nFactor);

//--------------------------------------------------
/**
\brief  Sharpen adjustment (RGB24)
\param  pInputBuffer	[in] input buffer      
\param  pOutputBuffer	[out]output buffer
\param  nWidth          [in] image width
\param  nHeight         [in] image height 
\param  fFactor        	[in] factor,range(0.1~5.0)

\return emStatus
*/
//--------------------------------------------------
VxInt32 DHDECL DxSharpen24B(void* pInputBuffer, void* pOutputBuffer, VxUint32 nWidth, VxUint32 nHeight, float fFactor);

//--------------------------------------------------
/**
\brief  Saturation adjustment (RGB24)
\param  pInputBuffer	[in] input buffer      
\param  pOutputBuffer	[out]output buffer
\param  nImageSize     	[in] image size (width * height)
\param  nFactor        	[in] factor,range(0 ~ 128)

\return emStatus   
*/
//--------------------------------------------------
VxInt32 DHDECL DxSaturation(void* pInputBuffer, void* pOutputBuffer, VxUint32 nImagesize, VxInt32 nFactor);

//--------------------------------------------------
/**
\brief  Get white balance ratios(RGB24), In order to calculate accurately, the camera should shoot objective "white" area,or input image
        is white area.
\param  pInputBuffer   	[in] input buffer
\param  nWidth        	[in] image width
\param  nHeight       	[in] image height
\param  dRatioR         [out]R ratio
\param  dRatioG         [out]G ratio
\param  dRatioB         [out]B ratio

\return emStatus  
*/   
//--------------------------------------------------
VxInt32 DHDECL DxGetWhiteBalanceRatio(void *pInputBuffer, VxUint32 nWidth, VxUint32 nHeight, double* dRatioR, double* dRatioG, double* dRatioB);

//-----------------------------------------------------
/**
\brief  Auto raw defective pixel correct,Support image from Raw8 to Raw16, the bit number is actual bit number, when it is more than 8, the actual bit 
        can be every number between 9 to 16. And if image format is packed, you need convert it to Raw16.This function should be used in each frame.

\param  pRawImgBuf      [in,out]Raw image buffer
\param  nWidth        	[in]image width
\param  nHeight       	[in]image height
\param  nBitNum         [in]image bit number (for example:if image 10bit, nBitNum = 10, if image 12bit,nBitNum = 12,range:8 ~ 16)

\return emStatus  
*/
//-----------------------------------------------------
VxInt32 DHDECL DxAutoRawDefectivePixelCorrect(void* pRawImgBuf, VxUint32 nWidth, VxUint32 nHeight, VxInt32 nBitNum);

//--------------------------------------------------
/**
\brief  Convert Raw16 to Raw8
\param  pInputBuffer   	[in] input buffer(size:width * height *2)
\param  pOutputBuffer   [out]output buffer(new buffer,size:width * height)
\param  nWidth          [in] image width
\param  nHeight         [in] image height
\param  nValidBits      [in] valid bits

\return emStatus  
*/
//--------------------------------------------------
VxInt32 DHDECL DxRaw16toRaw8(void *pInputBuffer, void *pOutputBuffer, VxUint32 nWidth, VxUint32 nHeight, DX_VALID_BIT nValidBits);

//--------------------------------------------------
/**
\brief  Convert RGB48 to RGB24
\param  pInputBuffer   	[in] input buffer(size:width * height * 3 *2)
\param  pOutputBuffer   [out]output buffer(new buffer,size:width * height * 3)
\param  nWidth          [in] image width
\param  nHeight         [in] image height
\param  nValidBits      [in] valid bits 

\return emStatus  
*/
//--------------------------------------------------
VxInt32 DHDECL DxRGB48toRGB24(void *pInputBuffer, void *pOutputBuffer, VxUint32 nWidth, VxUint32 nHeight, DX_VALID_BIT nValidBits);

//--------------------------------------------------
/**
\brief  Convert Raw16 to RGB48
\param  pInputBuffer   	[in] input buffer(size:width * height * 2)
\param  pOutputBuffer   [out]output buffer(new buffer,size:width * height * 3 * 2)
\param  nWidth  	    [in] image width
\param  nHeight   	    [in] image height
\param  nActualBits     [in] image actual bits
\param  cvtype          [in] Bayer convert type 
\param  nBayerType      [in] pixel color filter
\param  bFlip           [in] image flip or not, true:flip false:not flip

\return emStatus  
*/
//--------------------------------------------------
VxInt32 DHDECL DxRaw16toRGB48(void *pInputBuffer, void *pOutputBuffer, VxUint32 nWidth, VxUint32 nHeight, DX_ACTUAL_BITS nActualBits, DX_BAYER_CONVERT_TYPE cvtype, DX_PIXEL_COLOR_FILTER nBayerType, bool bFlip);

//--------------------------------------------------
/**
\brief  calculating contrast lookup table (RGB24)
\param  nContrastParam  [in] contrast param,range(-50 ~ 100)
\param  pContrastLut    [out]contrast lookup table
\param  pLutLength      [out]contrast lookup table length(unit:byte)

\return emStatus  
*/
//--------------------------------------------------
VxInt32 DHDECL DxGetContrastLut(int nContrastParam, void *pContrastLut, int *pLutLength);

//--------------------------------------------------
/**
\brief  calculating gamma lookup table (RGB24)
\param  dGammaParam   [in] gamma param,range(0.1 ~ 10) 
\param  pGammaLut     [out]gamma lookup table
\param  pLutLength    [out]gamma lookup table length(unit:byte)

\return emStatus  
*/
//--------------------------------------------------
VxInt32 DHDECL DxGetGammatLut(double dGammaParam, void *pGammaLut, int *pLutLength);

//--------------------------------------------------
/**
\brief  image quality improvement (RGB24)
\param  pInputBuffer   	      [in] input buffer
\param  pOutputBuffer         [out]output buffer
\param  nWidth                [in] image width
\param  nHeight               [in] image height
\param  nColorCorrectionParam [in] color correction param address(get from camera)
\param  pContrastLut          [in] contrast lookup table
\param  pGammaLut             [in] gamma lookup table

\return emStatus  
*/
//--------------------------------------------------
VxInt32 DHDECL DxImageImprovment(void *pInputBuffer, void *pOutputBuffer, VxUint32 nWidth, VxUint32 nHeight, VxInt64 nColorCorrectionParam, void *pContrastLut, void *pGammaLut);

//-------------------------------------------------------------
/**
\brief  image mirror(Raw8 or 8bit image) 
\param  pInputBuff   	[in] input buffer
\param  pOutputBuf      [out]output buffer
\param  nWidth          [in] image width
\param  nHeight         [in] image height
\param  emMirrorMode    [in] mirror mode

\return emStatus
*/
//-------------------------------------------------------------
VxInt32 DHDECL DxImageMirror(void *pInputBuffer, void *pOutputBuffer, VxUint32 nWidth, VxUint32 nHeight, DX_IMAGE_MIRROR_MODE emMirrorMode);

//--------------------------------------------------
/**
\brief  calculating lookup table of 8bit image 
\param  nContrastParam  [in] contrast param,range(-50~100)
\param  dGamma          [in] gamma param,range(0.1~10)
\param  nLightness      [in] lightness param,range(-150~150)
\param  pLut            [out]lookup table
\param  pLutLength      [in] lookup table length(unit:byte)

\return emStatus
*/
//--------------------------------------------------
VxInt32 DHDECL DxGetLut(VxInt32 nContrastParam, double dGamma, VxInt32 nLightness, VxUint8 *pLut, VxUint16 *pLutLength);

//--------------------------------------------------
/**
\brief  calculating array of image processing color adjustment 
\param  nColorCorrectionParam   [in] color correction param address(get from camera)
\param  nSaturation             [in] saturation factor,Range(0~128)
\param  parrCC                  [out]array address
\param  nLength                 [in] length(sizeof(VxInt16)*9)

\return emStatus
*/
//--------------------------------------------------
VxInt32 DHDECL DxCalcCCParam(VxInt64 nColorCorrectionParam, VxInt16 nSaturation, VxInt16 *parrCC, VxUint8 nLength);

//--------------------------------------------------
/**
\brief  Raw8 image process 
\param  pRaw8Buf    	      [in] input buffer
\param  pRgb24Buf             [out]output buffer(new buffer)
\param  nWidth                [in] image width
\param  nHeight               [in] image height
\param  pstClrImgProc         [in] Raw8 image process struct pointer

\return emStatus  
*/
//--------------------------------------------------
VxInt32 DHDECL DxRaw8ImgProcess(void *pRaw8Buf, void *pRgb24Buf, VxUint32 nWidth, VxUint32 nHeight, COLOR_IMG_PROCESS *pstClrImgProc);

//--------------------------------------------------
/**
\brief  Mono8 image process 
\param  pInputBuf    	      [in] input buffer
\param  pOutputBuf            [out]output buffer(new buffer)
\param  nWidth                [in] image width
\param  nHeight               [in] image height
\param  pstGrayImgProc        [in] mono8 image process struct pointer

\return emStatus  
*/
//--------------------------------------------------
VxInt32 DHDECL DxMono8ImgProcess(void *pInputBuf, void *pOutputBuf, VxUint32 nWidth, VxUint32  nHeight, MONO_IMG_PROCESS *pstGrayImgProc);

#ifdef __cplusplus
}
#endif

#endif




