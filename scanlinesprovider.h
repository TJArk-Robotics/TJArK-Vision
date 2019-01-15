/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  scanlinesprovider.h
 * @brief Fill scanlines.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#ifndef SCANLINESPROVIDER_H
#define SCANLINESPROVIDER_H

#include <cmath>
#include <image.h>
#include <color.h>
#include <point.h>
#include <imagetool.h>
#include <classifyimageprovider.h>

#define MAX_EDGE_CNT 20

struct scanline {
    point edge[MAX_EDGE_CNT];
    color edgeColor[MAX_EDGE_CNT];
    float length[MAX_EDGE_CNT];
    int cnt;
};

class scanlinesProvider
{
private:
    int width;
    int height;
    int numberOfScanlines;
    int linespace;
    int maxSkip;
    scanline * verScanlines;
    void reset();

public:
    scanlinesProvider();
    scanlinesProvider(int w, int h);

    void execute(const classifyImageProvider &cip);
    float edgeGradient(const classifyImageProvider &cip, int x, int y);
    void postProcess();
    void showImage(const image & img, vector<uint8_t> &rgb);
    void saveImage(const image & img, const string & fileName);

    int getNumberOfScanlines() const {
        return numberOfScanlines;
    }

    scanline * getScanlines() const {
        return verScanlines;
    }
};

#endif // SCANLINESPROVIDER_H
