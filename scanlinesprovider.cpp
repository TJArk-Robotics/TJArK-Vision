/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  scanlinesprovider.cpp
 * @brief Fill scanlines.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#include "scanlinesprovider.h"

scanlinesProvider::scanlinesProvider()
{
    this->height = 0;
    this->width = 0;
    this->linespace = 16;
    this->maxSkip = 1;
    this->numberOfScanlines = 0;
    this->verScanlines = nullptr;
}

scanlinesProvider::scanlinesProvider(int w, int h) : width(w), height(h) {
    this->linespace = 16;
    this->maxSkip = 1;
    this->numberOfScanlines = (width-linespace)/linespace+1;
    this->verScanlines = (scanline*)malloc(numberOfScanlines*sizeof(scanline));
    for (int i=0; i<numberOfScanlines; ++i) {
        for (int j=0; j<MAX_EDGE_CNT; ++j) {
            (verScanlines+i)->edge[j] = point(-1, -1);
            (verScanlines+i)->edgeColor[j] = color::none;
            (verScanlines+i)->length[j] = 0.f;
        }
        (verScanlines+i)->cnt = 0;
    }
}

void scanlinesProvider::reset() {
    for (int i=0; i<numberOfScanlines; ++i) {
        for (int j=0; j<MAX_EDGE_CNT; ++j) {
            (verScanlines+i)->edge[j] = point(-1, -1);
            (verScanlines+i)->edgeColor[j] = color::none;
            (verScanlines+i)->length[j] = 0.f;
        }
        (verScanlines+i)->cnt = 0;
    }
}

void scanlinesProvider::execute(const classifyImageProvider & cip) {
    reset();
    for (int i=linespace/2-1; i<width-linespace/2; i+=linespace) {
        scanline * l = verScanlines+i/linespace;
        // add bottom edge
        color lastColor = cip.getColor(i, height-1);
        color curColor = lastColor;
        l->edge[l->cnt] = point(i,  height-1);
//        l->gradient[l->cnt] = edgeGradient(cip, i, height-1);
        l->edgeColor[l->cnt++] = curColor;
        int skip = 0;
        for (int j=height-2; j>0; --j) {
            curColor = cip.getColor(i, j);
            if (curColor != lastColor)
                skip ++;
            else
                skip = (--skip<0)?0:skip;
            if (skip>maxSkip) {
                j = j+skip-1;
                l->edge[l->cnt] = point(i,  j);
//                l->gradient[l->cnt] = edgeGradient(cip, i, j);
                l->edgeColor[l->cnt++] = curColor;
                skip = 0;
                lastColor = curColor;
            }

            if (l->cnt>MAX_EDGE_CNT-2)
                break;
        }
        // add top edge
        l->edge[l->cnt] = point(i,  0);
//        l->gradient[l->cnt] = edgeGradient(cip, i, 0);
        l->edgeColor[l->cnt++] = cip.getColor(i, 0);
    }
    postProcess();
}

float scanlinesProvider::edgeGradient(const classifyImageProvider &cip, int x, int y) {
    float gx = 0;
    float gy = 0;
    uint8_t lt = cip.getCy(x-1, y-1);
    uint8_t mt = cip.getCy(x  , y-1);
    uint8_t rt = cip.getCy(x+1, y-1);
    uint8_t lm = cip.getCy(x-1, y  );
    uint8_t rm = cip.getCy(x+1, y  );
    uint8_t lb = cip.getCy(x-1, y+1);
    uint8_t mb = cip.getCy(x  , y+1);
    uint8_t rb = cip.getCy(x+1, y+1);
    gx = -(3*lt+10*lm+3*lb)+(3*rt+10*rm+3*rb);
    gy = -(3*lt+10*mt+3*rt)+(3*lb+10*mb+3*rb);
    return atan2(gy, gx);
}

void scanlinesProvider::postProcess() {
    for (int i=0; i<numberOfScanlines; ++i) {
        for (int j=0; j<MAX_EDGE_CNT-1; ++j) {
            scanline * l = verScanlines+i;
            l->length[j] = std::max(l->edge[j].x-l->edge[j+1].x, l->edge[j].y-l->edge[j+1].y);
        }
    }
}

void scanlinesProvider::showImage(const image &img, vector<uint8_t> &rgb) {
    imageTool::yuv422ToRgba(rgb, img.getData(), width, height);
    for(int i=0; i<numberOfScanlines; ++i) {
        scanline * l = verScanlines+i;
        for (int j=0; j<l->cnt-1; ++j) {
            int r,g,b;
            if (l->edgeColor[j]==color::green) {
                r = 255;
                g = 255;
                b = 0;
            }
            else if (l->edgeColor[j]==color::white) {
                r = 255;
                g = 0;
                b = 0;
            }
            else if (l->edgeColor[j]==color::black) {
                r = 0;
                g = 0;
                b = 255;
            }
            else {
                r = 0;
                g = 0;
                b = 0;
            }
            imageTool::drawLine(&rgb[0], l->edge[j].x, l->edge[j].y,
                    l->edge[j+1].x, l->edge[j+1].y,
                    r, g, b, width, height);
        }
    }
}

void scanlinesProvider::saveImage(const image &img, const string &fileName) {
    vector<uint8_t> rgb(width * height * 4, 255);
    showImage(img, rgb);
    imageTool::saveImage(fileName, rgb, width, height);
}
