/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  fieldcolorprovider.cpp
 * @brief Detect field color range.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#include "fieldcolorprovider.h"

fieldColorProvider::fieldColorProvider()
{
    this->width = 0;
    this->height = 0;
    this->isVValid = false;
    this->isYValid = false;
    this->tV = 0;
    this->tY = 0;
    this->lineSpace = 8;
}

fieldColorProvider::fieldColorProvider(uint32_t w, uint32_t h)
    : width(w), height(h)
{
    this->isVValid = false;
    this->isYValid = false;
    this->tV = 0;
    this->tY = 0;
    this->lineSpace = 8;
}

bool fieldColorProvider::execute(const image &img) {
    const uint8_t v_penn = 200;
    const uint8_t v_diam = 10;
    vector<float> histV(256, 0);
    int cnt = 0;
    for (int y=0; y<height; y+=lineSpace) {
        for (int x=0; x<width; x+=lineSpace) {
            uint8_t v = img.cr(x,y);
            if (v>v_penn)
                continue;
            histV[v] ++;
            cnt++;
        }
    }

    float otsuV, deltaV, dV;
    otsu(&histV[0], otsuV, deltaV, dV);
    vector<float> histY(256, 0);
    cnt = 0;
    for (int y=0; y<height; y+=lineSpace) {
        for (int x=0; x<width; x+=lineSpace) {
            uint8_t cy = img.y(x,y);
            histY[cy] ++;
            cnt ++;
        }
    }
    float massY = 0;
    for (int i=0; i<256; ++i)
        massY += i*histY[i]/cnt;
    int y_penn = massY-35;
    y_penn = (y_penn<100?100:y_penn);
    for (int i=0; i<256; ++i)
        if (i<y_penn)
            histY[i] = 0;
    float otsuY, deltaY, dY;
    otsu(&histY[0], otsuY, deltaY, dY);
    this->isVValid = dV>v_diam && otsuV>80;
    this->isYValid = dY>40 ;
    this->tV = otsuV;
    this->tY = otsuY;
    vector<float> histBWY(256, 0);
    for (int y=0; y<height; y+=2*lineSpace) {
        for (int x=0; x<width; x+=2*lineSpace) {
            uint8_t cy = img.y(x,y);
            if (isGreen(img, x, y))
                histBWY[cy] ++;
        }
    }
    float bwy, BWdeltaY, BWdY;
    otsu(&histBWY[0], bwy, BWdeltaY, BWdY);
    this->BWY = (bwy>10)?bwy:10;
    return true;
}

void fieldColorProvider::otsu(const float *hist, float &thre, float &delta, float &d) {
    float w0, w1, u0, u1, u0_tmp, u1_tmp, delta_tmp, d_tmp;
    float delta_max = 0;
    float d_max = 0;
    float threshold = 0;
    for (int i=0; i<256; ++i) {
        w0=0; w1=0; u0=0; u1=0; u0_tmp=0; u1_tmp=0; delta_tmp=0; d_tmp=0;
        for (int j=0; j<256; ++j) {
            if (j<i) {
                w0 += hist[j];
                u0_tmp += j*hist[j];
            }
            else {
                w1 += hist[j];
                u1_tmp += j*hist[j];
            }
            if (w0==0 || w1==0)
                continue;
            u0 = (u0_tmp)/(w0);
            u1 = (u1_tmp)/(w1);
            d_tmp = u0-u1;
            delta_tmp = w0*w1*d_tmp*d_tmp;
            if (delta_tmp > delta_max) {
                delta_max = delta_tmp;
                threshold = i;
                d_max = fabs(d_tmp);
            }
        }
    }
    thre = threshold;
    delta = delta_max;
    d = d_max;
}

uint8_t fieldColorProvider::findMinStable(const float *hist, int thresh) {
    int sum = 0;
    for (int i=0; i<256; ++i) {
        sum += hist[i];
        if (sum>thresh)
            return i;
    }
    return 255;
}

bool fieldColorProvider::isGreen(const image & img, int x, int y) {
    uint8_t cy = img.y(x, y);
    uint8_t cr = img.cr(x, y);
    if (isVValid && isYValid) {
        if (!(cr>tV || cy>tY))
            return true;
    }
    else if (isVValid) {
        if (cr<tV)
            return true;
    }
    else {
        if (cr<100 && cy<200)
            return true;
    }
    return false;
}

uint8_t fieldColorProvider::getThreshV() {
    return this->tV;
}

uint8_t fieldColorProvider::getThreshY() {
    return this->tY;
}

uint8_t fieldColorProvider::getThreshCy() const {
    return isYValid?tY:(isVValid?255:200);
}

uint8_t fieldColorProvider::getThreshCr() const {
    return isVValid?tV:100;
}

uint8_t fieldColorProvider::getThreshBWY() const {
    return BWY;
}

bool fieldColorProvider::getStatusV() {
    return this->isVValid;
}

bool fieldColorProvider::getStatusY() {
    return this->isYValid;
}

void fieldColorProvider::getGreenIndex(const image &in, vector<int> &xs, vector<int> &ys) {
    xs.clear();
    ys.clear();
    for (int y=0; y<height; y++) {
        for (int x=0; x<width; x++) {
            if (isGreen(in, x, y)) {
                xs.push_back(x);
                ys.push_back(y);
            }
        }
    }
}

void fieldColorProvider::showImage(const image &in, image &out) {
    out = in;
    for (int y=0; y<height; y++) {
        for (int x=0; x<width; x++) {
            if (isGreen(out, x, y)) {
                out.setY(x, y, 0);
            }
        }
    }
}

void fieldColorProvider::showImage(const image &in, vector<uint8_t> &rgb) {
    vector<int> xs, ys;
    getGreenIndex(in, xs, ys);
    imageTool::yuv422ToRgba(rgb, in.getData(), width, height);
    for (size_t i=0; i<xs.size(); ++i) {
        imageTool::setRgba(rgb, xs[i], ys[i], 0, 205, 0, width);
    }
}
