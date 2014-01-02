#ifndef IPP_WRAPPER_H
#define IPP_WRAPPER_H

#ifdef IPP_FOUND
#include <ipp.h>
#else


#include <fwBase.h>
#include <fwImage.h>
#include <fwJPEG.h>
#include <fwSignal.h>
#include <fwVideo.h>

// Basic types
#define Ipp8u Fw8u
#define Ipp32f Fw32f

// Image related types
#define IppiSize FwiSize
#define IppiRect FwiRect

// Defines
#define ippAxsBoth fwAxsBoth
#define ippAxsHorizontal fwAxsHorizontal
#define IPPI_INTER_LINEAR FWI_INTER_LINEAR
#define IPPI_INTER_NN FWI_INTER_NN

// Functions
#define ippiCopy_8u_C3P3RR fwiCopy_8u_C3P3RR
#define ippiCopy_8u_C3P3R fwiCopy_8u_C3P3R
#define ippiCopy_8u_C1R fwiCopy_8u_C1R
#define ippiMirror_8u_C1IR fwiMirror_8u_C1IR
#define ippiMirror_8u_C3IR fwiMirror_8u_C3IR
#define ippiTranspose_8u_C1IR fwiTranspose_8u_C1IR
#define ippiResize_8u_C1R fwiResize_8u_C1R
#define ippiRGBToGray_8u_C3C1R fwiRGBToGray_8u_C3C1R

#define ippiYUV422ToRGB_8u_C2C3R fwiYUV422ToRGB_8u_C2C3R

#define ippsMalloc_8u fwsMalloc_8u
//#define ippsFilterMedian_32f_I fwsFilterMedian_32f_I // Doesn't actually exists
#define ippiResizeSqrPixel_8u_C3R fwiResizeSqrPixel_8u_C3R
#define ippiResizeGetBufSize fwiResizeGetBufSize
#endif


#endif


