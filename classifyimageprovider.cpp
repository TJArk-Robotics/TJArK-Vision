/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  classifyimageprovider.cpp
 * @brief classify image into colored image
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#include "classifyimageprovider.h"
#include "color.h"

#include <tmmintrin.h>
#include <iostream>

using namespace std;

classifyImageProvider::classifyImageProvider()
{
    this->width = 0;
    this->height = 0;
    this->grayImage = nullptr;
    this->cbImage = nullptr;
    this->crImage = nullptr;
    this->coloredImage = nullptr;
}

classifyImageProvider::classifyImageProvider(int w, int h) : width(w), height(h) {
    this->grayImage = (uint8_t*)malloc(width*height*sizeof(uint8_t));
    this->cbImage = (uint8_t*)malloc(width*height*sizeof(uint8_t));
    this->crImage = (uint8_t*)malloc(width*height*sizeof(uint8_t));
    this->coloredImage = (uint8_t*)malloc(width*height*sizeof(uint8_t));
}

void classifyImageProvider::execute(const image &img, const fieldColorProvider &fcp) {
    threshCy = fcp.getThreshCy();
    threshCr = fcp.getThreshCr();
    __m128i tCy = _mm_set1_epi8(threshCy);
    __m128i tCr = _mm_set1_epi8(threshCr);
    __m128i tBCy = _mm_set1_epi8(fcp.getThreshBWY());
    __m128i constZero = _mm_setzero_si128();
    __m128i greenColor = _mm_set1_epi8(color::green);
    __m128i whiteColor = _mm_set1_epi8(color::white);
    __m128i blackColor = _mm_set1_epi8(color::black);
    // using sse
    const uint8_t * img_ptr = img.getData();
    int pointer = 0;
    int singleChannelPointer = 0;
    const int end_pointer = img.getWidth()*img.getHeight()*2-1;

    while (pointer<end_pointer) {
        __m128i regA = _mm_load_si128(reinterpret_cast<const __m128i*>(img_ptr+pointer));
        pointer += 16;
        __m128i regB = _mm_loadu_si128(reinterpret_cast<const __m128i*>(img_ptr+pointer));
        pointer += 16;
        __m128i r1a = _mm_unpacklo_epi8(regA, regB);
        __m128i r1b = _mm_unpackhi_epi8(regA, regB);
        __m128i r2a = _mm_unpacklo_epi8(r1a, r1b);
        __m128i r2b = _mm_unpackhi_epi8(r1a, r1b);
        r1a = _mm_unpacklo_epi8(r2a, r2b);
        r1b = _mm_unpackhi_epi8(r2a, r2b);
        r2a = _mm_unpacklo_epi8(r1a, r1b);
        r2b = _mm_unpackhi_epi8(r1a, r1a);
        r1a = _mm_unpackhi_epi8(r1b, r1b);
        // classify pixels
        __m128i greenCr = _mm_cmpeq_epi8(_mm_subs_epu8(r1a, tCr), constZero);
        __m128i greenCy = _mm_cmpeq_epi8(_mm_subs_epu8(r2a, tCy), constZero);
        __m128i isGreen = _mm_and_si128(greenCy, greenCr);
        __m128i whiteCy = _mm_cmpeq_epi8(_mm_subs_epu8(r2a, tBCy), constZero);
        __m128i res = _mm_or_si128(_mm_and_si128(
                                       isGreen,greenColor),
                                   _mm_andnot_si128(
                                       isGreen,
                                       _mm_or_si128(
                                           _mm_andnot_si128(whiteCy, whiteColor),
                                           _mm_and_si128(whiteCy, blackColor))));
        // store images
        _mm_storeu_si128(reinterpret_cast<__m128i *>(grayImage+singleChannelPointer), r2a);
        _mm_storeu_si128(reinterpret_cast<__m128i *>(cbImage+singleChannelPointer), r2b);
        _mm_storeu_si128(reinterpret_cast<__m128i *>(crImage+singleChannelPointer), r1a);
        _mm_storeu_si128(reinterpret_cast<__m128i *>(coloredImage+singleChannelPointer), res);
        singleChannelPointer += 16;
    }
}

