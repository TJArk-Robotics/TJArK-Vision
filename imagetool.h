/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  imagetool.h
 * @brief Some image functions.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#ifndef IMAGETOOL_H
#define IMAGETOOL_H

#include <stdint.h>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>

#include <lodepng/lodepng.h>
#include <image.h>
#include <color.h>
#include <point.h>

#define posix_memalign(p, a, s) (((*(p)) = _aligned_malloc((s), (a))), *(p) ?0 :errno)

using namespace std;

namespace imageTool
{
inline uint8_t clip(float d) {
    if(d<0)d=0u;
    if(d>255)d=255u;
    return (uint8_t)d;
}

template <typename T>
inline T clip (T min, T x, T max) {
    return std::min(std::max(min, x), max);
}

inline void rgbaToYuv422(uint8_t *out, const std::vector<uint8_t> &in, int width, int height) {
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int r = in[x * 4 + y * width * 4];
            int g = in[1 + x * 4 + y * width * 4];
            int b = in[2 + x * 4 + y * width * 4];
            out[x * 2 + y * width * 2] = (uint8_t) (0.299 * r + 0.587 * g + 0.114 * b);
            if (x % 2 == 0) {
                out[1 + x * 2 + y * width * 2] =
                        (uint8_t) (-0.169 * r - 0.331 * g + 0.499 * b + 128);
            } else {
                out[1 + x * 2 + y * width * 2] =
                        (uint8_t) (0.498 * r - 0.419 * g - 0.0813 * b + 128);
            }
        }
    }
}

inline void grayToRgba(std::vector<uint8_t> &out, const uint8_t *const in, int width, int height) {
    for(int py=0;py<height;py++){
        for(int px=0;px<width;px++){
            int y=in[(px+py*width)]&255;
            out[px * 4 + py * width * 4] = clip(y);
            out[1 + px * 4 + py * width * 4] = clip(y);
            out[2 + px * 4 + py * width * 4] = clip(y);
        }
    }
}

inline void yuv422ToRgba(std::vector<uint8_t> &out, const uint8_t *const in, int width, int height) {
    for(int py=0;py<height;py++){
        int cbLast=in[(0+py*width)*2+1]&255;
        int crLast=in[(0+py*width)*2+3]&255;
        for(int px=0;px<width;px++){
            int y=in[(px+py*width)*2]&255;
            if((px&1)==0){
                cbLast=in[(px+py*width)*2+1]&255;
            }else{
                crLast=in[(px+py*width)*2+1]&255;
            }
            int cb=cbLast;
            int cr=crLast;
            out[px * 4 + py * width * 4] = clip(y+1.402*(cr-128)+2);
            out[1 + px * 4 + py * width * 4] = clip(y-0.344*(cb-128)-0.714*(cr-128));
            out[2 + px * 4 + py * width * 4] = clip(y+1.772*(cb-128)+2);
        }
    }
}

inline void coloredToRgba(std::vector<uint8_t> &out, const uint8_t *const in, int width, int height) {
    for(int py=0;py<height;py++){
        for(int px=0;px<width;px++){
            int c=in[(px+py*width)]&255;
            if (c==color::green) {
                out[px * 4 + py * width * 4] = 0;
                out[1 + px * 4 + py * width * 4] = 205;
                out[2 + px * 4 + py * width * 4] = 0;
            }
            else if (c==color::white) {
                out[px * 4 + py * width * 4] = 255;
                out[1 + px * 4 + py * width * 4] = 255;
                out[2 + px * 4 + py * width * 4] = 255;
            }
            else if (c==color::black){
                out[px * 4 + py * width * 4] = 0;
                out[1 + px * 4 + py * width * 4] = 0;
                out[2 + px * 4 + py * width * 4] = 0;
            }
            else {
                out[px * 4 + py * width * 4] = 128;
                out[1 + px * 4 + py * width * 4] = 128;
                out[2 + px * 4 + py * width * 4] = 128;
            }
        }
    }
}

inline void setRgba(vector<uint8_t> & in, int px, int py, int r, int g, int b, int width) {
    in[px * 4 + py * width * 4] = clip(r);
    in[1 + px * 4 + py * width * 4] = clip(g);
    in[2 + px * 4 + py * width * 4] = clip(b);
}

inline void setRgba(uint8_t * in, int px, int py, int r, int g, int b, int width=640, int height=480) {
    if (px<0 || px>width-1) return;
    if (py<0 || py>height-1) return;
    *(in + px * 4 + py * width * 4) = clip(r);
    *(in + 1 + px * 4 + py * width * 4) = clip(g);
    *(in + 2 + px * 4 + py * width * 4) = clip(b);
}

inline uint8_t * loadImage(const string & fileName, uint32_t & width, uint32_t & height) {
    vector<uint8_t> imageRGBA;
    size_t bufferSize = sizeof(uint8_t) * 2 * width * height;
    uint32_t error = lodepng::decode(imageRGBA, width, height, fileName);

    if (error) {
        cout << "[ERROR] While decoding image: " << error << ": " << lodepng_error_text(error) << std::endl;
        exit(1);
    }

    uint8_t * imageYUV422 = nullptr;
    if (posix_memalign((void **)&imageYUV422, 16, bufferSize) != 0) {
        cout << "[ERROR] While allocating memory: " << strerror(errno) << std::endl;
        exit(1);
    }

    if(imageYUV422 == nullptr) {
        cout << "[ERROR] Couldn't allocate yuv image memory." << std::endl;
        exit(1);
    }

    rgbaToYuv422(imageYUV422, imageRGBA, width, height);
    return imageYUV422;
}

inline void saveImage(const string & fileName, vector<uint8_t>& in, int width, int height) {
    auto error = lodepng::encode(fileName, in, width, height);
    if (!error)
        cout << "Success saving " << fileName << endl;
    else
        cout << "Error saving " << fileName << endl;
}

inline void saveImage(const string & fileName, image & img, int width, int height) {
    uint8_t * data = img.getData();
    vector<uint8_t> result(width * height * 4, 255);
    yuv422ToRgba(result, data, width, height);
    auto error = lodepng::encode(fileName, result, width, height);
    if (!error)
        cout << "Success saving " << fileName << endl;
    else
        cout << "Error saving " << fileName << " with error code: " << error << endl;
}

inline void saveGrayscaleImage(const string & fileName, uint8_t* in, int width, int height) {
    vector<uint8_t> result(width * height * 4, 255);
    grayToRgba(result, in, width, height);
    auto error = lodepng::encode(fileName, result, width, height);
    if (!error)
        cout << "Success saving " << fileName << endl;
    else
        cout << "Error saving " << fileName << " with error code: " << error << endl;
}

inline void saveColoredImage(const string & fileName, uint8_t* in, int width, int height) {
    vector<uint8_t> result(width * height * 4, 255);
    coloredToRgba(result, in, width, height);
    auto error = lodepng::encode(fileName, result, width, height);
    if (!error)
        cout << "Success saving " << fileName << endl;
    else
        cout << "Error saving " << fileName << " with error code: " << error << endl;
}

inline void drawCross(uint8_t * in, int x, int y, int r, int g, int b,
                      int len=5, int width=640, int height=480) {
    for (int i=0; i<len; ++i) {
        setRgba(in, x-i, y-i, r, g, b, width, height);
        setRgba(in, x+i, y+i, r, g, b, width, height);
        setRgba(in, x-i, y+i, r, g, b, width, height);
        setRgba(in, x+i, y-i, r, g, b, width, height);
    }
}

inline void drawLine(uint8_t * in, int startX, int startY, int endX, int endY,
                     int r, int g, int b, int width=640, int height=480) {
    for (float i=0; i<=1; i+=0.001) {
        int x = (int)round(startX*(1-i)+endX*i);
        int y = (int)round(startY*(1-i)+endY*i);
                setRgba(in, x, y, r, g, b, width, height);
    }
}

inline void drawLine(uint8_t * in, point start, point end,
                     int r, int g, int b, int thickness=1, int width=640, int height=480) {
    for (float i=0; i<=1; i+=0.001) {
        int x = (int)round(start.x*(1-i)+end.x*i);
        int y = (int)round(start.y*(1-i)+end.y*i);
            for (int q=1-thickness; q<thickness; q++)
                setRgba(in, x, y+q, r, g, b, width, height);
    }
}

inline void drawArray(uint8_t * in, int startX, int startY, float angle,
                      int r, int g, int b, int len=15, int width=640, int height=480) {
    int endX = startX+len*cos(angle);
    int endY = startY+len*sin(angle);
    drawLine(in, startX, startY, endX, endY, r, g, b, width, height);
    int a1x = endX-len/3*sin(3*pi/8-angle);
    int a1y = endY-len/3*cos(3*pi/8-angle);
    int a2x = endX-len/3*cos(angle-pi/8);
    int a2y = endY-len/3*sin(angle-pi/8);
    drawLine(in, endX, endY, a1x, a1y, r, g, b, width, height);
    drawLine(in, endX, endY, a2x, a2y, r, g, b, width, height);
}

inline void drawRect(uint8_t * in, point leftTop, point rightBot,
                     int r, int g, int b, bool fill=false, int thickness=1, int width=640, int height=480) {
    for (int x=leftTop.x; x<=rightBot.x; ++x)
        for (int y=leftTop.y; y<=rightBot.y; ++y) {
            if (fill)
                setRgba(in, x, y, r, g, b, width, height);
            else
                if (fabs(x-leftTop.x)<thickness || fabs(x-rightBot.x)<thickness || fabs(y-leftTop.y)<thickness || fabs(y-rightBot.y)<thickness)
                    setRgba(in, x, y, r, g, b, width, height);
        }
}
}

#endif // IMAGETOOL_H
