/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  classifyimageprovider.h
 * @brief classify image into colored image
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#ifndef CLASSIFYIMAGEPROVIDER_H
#define CLASSIFYIMAGEPROVIDER_H

#include <stdint.h>

#include <image.h>
#include <color.h>
#include <fieldcolorprovider.h>

class classifyImageProvider
{
private:
    int width;
    int height;
    uint8_t threshCy;
    uint8_t threshCr;
    uint8_t * coloredImage;
    uint8_t * grayImage;
    uint8_t * cbImage;
    uint8_t * crImage;

public:

    classifyImageProvider();
    classifyImageProvider(int w, int h);
    void execute(const image &img, const fieldColorProvider &fcp);

    uint8_t * getGrayImage() const {
        return grayImage;
    }

    uint8_t * getCbImage() const {
        return cbImage;
    }

    uint8_t * getCrImage() const {
        return crImage;
    }

    uint8_t * getColoredImage() const {
        return coloredImage;
    }

    color getColor(int x, int y) const {
        if (x<0 || x>width || y<0 || y>height)
            return color::none;
        return color(*(coloredImage+x+y*width));
    }

    uint8_t getCy(int x, int y) const {
        if (x<0 || x>width || y<0 || y>height)
            return 0;
        return *(grayImage+x+y*width);
    }

    uint8_t getCr(int x, int y) const {
        if (x<0 || x>width || y<0 || y>height)
            return 0;
        return *(crImage+x+y*width);
    }

};

#endif // CLASSIFYIMAGEPROVIDER_H
