/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  edgeimageprovider.cpp
 * @brief Detect edge in a single channel.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#include "edgeimageprovider.h"

edgeImageProvider::edgeImageProvider() {
    width = 0;
    height = 0;
    edge = nullptr;
    edgeChannel == channel::y;
}

edgeImageProvider::edgeImageProvider(int w, int h)
    : width(w), height(h), edgeChannel(channel::y) {
    edge = (uint8_t*)malloc(width*height*sizeof(uint8_t));
    memset(edge, 0, width*height*sizeof(uint8_t));
}

edgeImageProvider::edgeImageProvider(int w, int h, channel c)
    : width(w), height(h), edgeChannel(c) {
    edge = (uint8_t*)malloc(width*height*sizeof(uint8_t));
    memset(edge, 0, width*height*sizeof(uint8_t));
}

void edgeImageProvider::execute(const classifyImageProvider &cip, const fieldBoundaryProvider &fbp, channel c) {
    int startY = fbp.getMinBoundary();
    uint8_t * src;
    switch(c) {
        case channel::y:
            src = cip.getGrayImage();
            break;
        case channel::cb:
            src = cip.getCbImage();
            break;
        case channel::cr:
            src = cip.getCrImage();
            break;
        default:
            src = cip.getGrayImage();
            break;
    }

    findEdge(src, edge, width, height, startY);
}

void edgeImageProvider::findEdge(const uint8_t * src, uint8_t * dst, int width ,int height, int startY) {
    if (startY>height)
        return;
    int endX = width-1;
    int endY = height-1;

    const uint8_t* row0 = src + startY * width;
    const uint8_t* row1 = row0 + width;
    const uint8_t* row2 = row1 + width;

    __m128i sumX;
    __m128i sumY;
    __m128i tmp;
    __m128i zeros = _mm_setzero_si128();
//    __m128i thresh = _mm_set1_epi8(20);

    for (int y = startY + 1; y < endY; y++) {
        for (int x = 1; x < endX; x += 14) {
            __m128i row_0 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(row0+x-1));
            __m128i row_1 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(row1+x-1));
            __m128i row_2 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(row2+x-1));
            // a b c
            // d * f
            // g h i
            __m128i valA = row_0;
            __m128i valB = _mm_srli_si128(row_0, 1);
            __m128i valC = _mm_srli_si128(row_0, 2);
            __m128i valD = row_1;
            __m128i valF = _mm_srli_si128(row_1, 2);
            __m128i valG = row_2;
            __m128i valH = _mm_srli_si128(row_2, 1);
            __m128i valI = _mm_srli_si128(row_2, 2);

            sumX = _mm_avg_epu8(valA, valG);
            sumX = _mm_avg_epu8(sumX, valD);
            tmp = _mm_avg_epu8(valC, valI);
            tmp = _mm_avg_epu8(tmp, valF);
            sumX = _mm_subs_epu8(_mm_max_epu8(sumX, tmp), _mm_min_epu8(sumX, tmp));

            sumY = _mm_avg_epu8(valA, valC);
            sumY = _mm_avg_epu8(sumY, valB);
            tmp = _mm_avg_epu8(valG, valI);
            tmp = _mm_avg_epu8(tmp, valH);
            sumY = _mm_subs_epu8(_mm_max_epu8(sumY, tmp), _mm_min_epu8(sumY, tmp));
            __m128i mins = _mm_min_epu8(sumX, sumY);
            __m128i maxs = _mm_max_epu8(sumX, sumY);
            mins = _mm_srli_epi8(mins, 2);
            __m128i result;
            result = _mm_adds_epu8(mins, maxs);
            _mm_storeu_si128(reinterpret_cast<__m128i*>(dst + x + y * width), result);
        }
        row0 += width;
        row1 += width;
        row2 += width;
    }

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            if (j <= 0 || j >= endX - 1 || i <= startY || i >= endY - 1) {
                dst[i * width + j] = 0;
            }
        }
    }
}

void edgeImageProvider::showImage(vector<uint8_t> &rgb) {
    imageTool::grayToRgba(rgb, edge, width, height);
}

void edgeImageProvider::saveImage(const string &fileName) {
    vector<uint8_t> rgb(width * height * 4, 255);
    showImage(rgb);
    imageTool::saveImage(fileName, rgb, width, height);
}
