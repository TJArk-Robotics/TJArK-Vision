/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  fieldcolorprovider.h
 * @brief Detect field color range.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#ifndef FIELDCOLORPROVIDER_H
#define FIELDCOLORPROVIDER_H

#include <stdint.h>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>

#include <image.h>
#include <imagetool.h>

using namespace std;

class fieldColorProvider
{
private:
    int width;
    int height;

    uint8_t tV;
    uint8_t tY;
    uint8_t BWY;
    bool isVValid;
    bool isYValid;

    int lineSpace;

    bool compensate = false;

public:
    fieldColorProvider();
    fieldColorProvider(uint32_t w, uint32_t h);
    bool execute(const image & img);
    uint8_t findMinStable(const float *hist, int thresh);
    void otsu(const float * hist, float &thre, float &delta, float &d);
    bool isGreen(const image &img, int x, int y);
    uint8_t getThreshY();
    uint8_t getThreshV();
    uint8_t getThreshCy() const;
    uint8_t getThreshCr() const;
    uint8_t getThreshBWY() const;
    bool getStatusY();
    bool getStatusV();

    void getGreenIndex(const image & in, vector<int> & x, vector<int> &y);
    void showImage(const image & in, image & out);
    void showImage(const image & in, vector<uint8_t> & rgb);
};

#endif // FIELDCOLORPROVIDER_H
