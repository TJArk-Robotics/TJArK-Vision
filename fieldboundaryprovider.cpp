/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  fieldboundaryprovider.cpp
 * @brief Detect field boundary.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#include "fieldboundaryprovider.h"

fieldBoundaryProvider::fieldBoundaryProvider()
{
    this->width = 0;
    this->height = 0;
    this->candidates.clear();
    this->fieldBoundary.clear();
}

fieldBoundaryProvider::fieldBoundaryProvider(int w, int h) : width(w), height(h) {
    this->candidates.clear();
    this->fieldBoundary.clear();
}

void fieldBoundaryProvider::execute(const scanlinesProvider &slp) {
    candidates.clear();
    fieldBoundary.clear();
    for (int i=0; i<slp.getNumberOfScanlines(); ++i) {
        scanline * l = slp.getScanlines()+i;
        int j=0;
        while (j<l->cnt-1) {
            if (l->edgeColor[j]!=color::green) {
                int k = j;
                while(k<l->cnt-1) {
                    if (l->edgeColor[++k]==color::green)
                        break;
                }
                if ((l->edge[j].y-l->edge[k].y) > maxSkip)
                    break;
                j = k;
            }
            if (j==l->cnt-1) {
                j--;
                break;
            }
            j++;
        }
        candidates.push_back(l->edge[j]);
    }

    bool twoBorder = false;
    vector<point> res1, res2;
    vector<float> line1, line2;
    bool boundaryFound = ransacLine(candidates, line1, res1);
    if (res1.size()>6) {
        if (ransacLine(res1, line2, res2))
            twoBorder = true;
    }
    for (int i=0; i<width; ++i) {
        int tmp = line1[0]*i+line1[2];
        int y;
        if (!boundaryFound) {
            fieldBoundary.push_back(0);
            continue;
        }
        if (twoBorder) {
            int tmp2 = line2[0]*i+line2[2];
            y = std::max(tmp, tmp2);
        }
        else
            y = std::max(0, tmp);
        fieldBoundary.push_back(y);
    }
    upperBoundary = *min_element(fieldBoundary.begin(), fieldBoundary.end());
}

bool fieldBoundaryProvider::ransacLine(const vector<point> &p, vector<float> &bestLine, vector<point> &res,
                                       float sigma, int iter) {
    bestLine.clear();
    res.clear();
    bestLine.push_back(0);
    bestLine.push_back(0);
    bestLine.push_back(0);
    vector<float> line = bestLine;
    int n = static_cast<int>(p.size());
    int bestOnLine = 0;
    for (int i=0; i<iter; ++i) {
        point p1 = p[distrib(seed)*n];
        point p2 = p[distrib(seed)*n];
        float k=0;
        if ((p1.x-p2.x)==0) {
            line[0] = 1;
            line[1] = 0;
            line[2] = -p1.x;
            k = 0;
        }
        else {
            k = float(p1.y-p2.y)/float(p1.x-p2.x);
            line[0] = k;
            line[1] = -1;
            line[2] = p1.y - k*p1.x;
        }
        int onLine = 0;
        for (const auto & a : p) {
            float d = line[0]*a.x - a.y + line[2];
            d /= sqrt(k*k+1);
            if (fabs(d)<sigma)
                onLine ++;

        }
        if (onLine > bestOnLine) {
            bestOnLine = onLine;
            bestLine = line;
        }
    }
    if (bestOnLine<10 && bestOnLine*4 < 3*n) {
        res = p;
        return false;
    }
    for (const auto & a : p) {
        float d = bestLine[0]*a.x - a.y + bestLine[2];
        d /= sqrt(bestLine[0]*bestLine[0]+bestLine[1]*bestLine[1]);
        if (fabs(d)>=sigma)
            res.push_back(a);
    }
    return true;
}

void fieldBoundaryProvider::showImage(const image &img, vector<uint8_t> &rgb) {
    imageTool::yuv422ToRgba(rgb, img.getData(), width, height);
    for (const auto & p : candidates) {
        imageTool::drawCross(&rgb[0], p.x, p.y, 255, 255, 0);
    }
    for (int i=0; i<width; ++i) {
        imageTool::drawCross(&rgb[0], i, fieldBoundary[i], 255, 20, 150, 2);
    }
}

void fieldBoundaryProvider::saveImage(const image &img, const string &fileName) {
    vector<uint8_t> rgb(width * height * 4, 255);
    showImage(img, rgb);
    imageTool::saveImage(fileName, rgb, width, height);
}
