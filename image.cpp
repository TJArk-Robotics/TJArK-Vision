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
#include "image.h"
#include <imagetool.h>

image::image() {
    width = 0;
    height = 0;
    channels = 0;
    data = nullptr;
}

image::image(uint32_t w, uint32_t h)
    : width(w), height(h) {
    data = nullptr;
    channels = 3;
}

image::image(uint32_t w, uint32_t h, uint8_t *i)
    : width(w), height(h), data(i) {
    channels = 3;
}

void image::loadImage(const string &fileName) {
    data = imageTool::loadImage(fileName, width, height);
}
