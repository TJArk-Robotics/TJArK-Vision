/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  edgeimageprovider.h
 * @brief Detect edge in a single channel.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#ifndef EDGEIMAGEPROVIDER_H
#define EDGEIMAGEPROVIDER_H

#include <stdint.h>
#include <tmmintrin.h>
#include <SIMD.h>

#include <classifyimageprovider.h>
#include <fieldboundaryprovider.h>

enum channel {
    y,
    cb,
    cr
};

class edgeImageProvider
{
private:
    int width;
    int height;

    channel edgeChannel;
    uint8_t * edge;

public:
    edgeImageProvider();
    edgeImageProvider(int w, int h);
    edgeImageProvider(int w, int h, channel c);

    void execute(const classifyImageProvider & cip, const fieldBoundaryProvider & fbp, channel c=channel::y);
    static void findEdge(const uint8_t *src, uint8_t *dst, int width, int height, int boundY);
    uint8_t * getEdge() const {
        return edge;
    }

    void showImage(vector<uint8_t> &rgb);
    void saveImage(const string &fileName);
};

#endif // EDGEIMAGEPROVIDER_H
