/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  image.h
 * @brief Basic class for image.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#ifndef IMAGE_H
#define IMAGE_H

#include <stdint.h>
#include <string>

using namespace std;

class image
{
private:
    uint8_t * data;
    uint32_t width;
    uint32_t height;
    uint8_t channels;
public:
    image();
    image(uint32_t w, uint32_t h);
    image(uint32_t w, uint32_t h, uint8_t * i);
    void loadImage(const string & fileName);

    uint8_t * getData() const {
        uint8_t * a = new uint8_t[width*height*2];
        memcpy(a, data, width*height*2*sizeof(uint8_t));
        return a;
    }

    uint32_t getWidth() const {
        return width;
    }

    uint32_t getHeight() const {
        return height;
    }

    uint8_t getChannels() const {
        return channels;
    }

    uint8_t y(int x, int y) const {
        return channels==3?data[(x + y * width) << 1]:data[(x + y * width)];
    }

    uint8_t cb(int x, int y) const {
        return channels==3?data[(((x + y * width) >> 1) << 2) + 1]:0;
    }

    uint8_t cr(int x, int y) const {
        return channels==3?data[((x + y * width) << 1) | 3]:0;
    }

    void setY(int x, int y, int val) {
        if (channels==3)
            data[(x+y*width)<<1]=val;
        else
            data[(x+y*width)]=val;
    }

    void setWidth(uint32_t w) {
        this->width = w;
    }

    void setHeight(uint32_t h) {
        this->height = h;
    }

    void setData(uint8_t * d) {
        this->data = d;
    }

    void setChannels(uint8_t c) {
        this->channels = c;
    }

    image & operator = (const image & other) {
        this->width = other.getWidth();
        this->height = other.getHeight();
        this->channels = other.getChannels();
        this->data = other.getData();
        return *this;
    }
};

#endif // IMAGE_H
