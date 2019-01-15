/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  lineprovider.cpp
 * @brief Detect lines and return positions.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#include "lineprovider.h"

lineprovider::lineprovider()
{
    this->width = 0;
    this->height = 0;
    this->lines.clear();
    this->segment.clear();
    this->spots.clear();
}

lineprovider::lineprovider(int w, int h) : width(w), height(h) {
    this->lines.clear();
    this->segment.clear();
    this->spots.clear();
}

void lineprovider::execute(const fieldSpotProvider & fsp) {
    this->lines.clear();
    this->segment.clear();
    this->spots.clear();

    segment = fsp.getLineSpots();
    spots = segment;

    vector<lineSegment> segHypo;
    while (!segment.empty()) {
        segHypo.clear();
        float  center = segment.front().angle;
        float maxDir = center;
        float minDir = center;
        segHypo.push_back(segment.front());
        segment.erase(segment.begin());
        for (size_t i=0; i<segment.size();) {
            point delta = segment[i].self-segHypo.back().self;
            if (fabs(segment[i].angle-center)<maxAngError
                    && delta.norm()<maxDisError
                    && fabs(delta.invdir()-center)<1.2f*maxAngError
                    && fabs(delta.invdir()-maxDir)<1.5f*maxAngError
                    && fabs(delta.invdir()-minDir)<1.5f*maxAngError) {
                segHypo.push_back(segment[i]);
                center = 1.f/(segHypo.size()+1.f) * segment[i].angle + segHypo.size()/(segHypo.size()+1.f) * center;
                maxDir=std::max(delta.invdir(), maxDir);
                minDir=std::min(delta.invdir(), minDir);
                segment.erase(segment.begin()+i);
                continue;
            }
            i++;
        }
        if (segHypo.size()>4) {
            point start, end;
            vector<lineSegment> res, res2;
            bool lineFound = ransacLine(segHypo, start, end, res);
            if (lineFound) {
                lines.push_back(line(start, end));
            }
            while(res.size()>3 && lineFound) {
               lineFound = ransacLine(res, start, end, res2);
               if (lineFound) {
                   lines.push_back(line(start, end));
                   res = res2;
               }
            }
        }
    }

}

bool lineprovider::ransacLine(const vector<lineSegment> &p, point & start, point & end, vector<lineSegment> &res,
                                       float sigma, int iter) {
    res.clear();
    float best[3] = {0};
    float line[3] = {0};
    int n = static_cast<int>(p.size());
    int bestOnLine = 0;

    for (int i=0; i<iter; ++i) {
        point p1 = p[distrib(seed)*n].self;
        point p2 = p[distrib(seed)*n].self;
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
            float d = line[0]*a.self.x - a.self.y + line[2];
            d /= sqrt(k*k+1);
            if (fabs(d)<sigma)
                onLine ++;

        }
        if (onLine > bestOnLine) {
            bestOnLine = onLine;
            for (int i=0; i<3; ++i)
                best[i] = line[i];
        }
    }

    if (bestOnLine<6 && bestOnLine*4 < 3*n) {
        res = p;
        return false;
    }

    vector<lineSegment> on;
    for (const auto & a : p) {
        float d = best[0]*a.self.x - a.self.y + best[2];
        d /= sqrt(best[0]*best[0]+best[1]*best[1]);
        if (fabs(d)>=sigma)
            res.push_back(a);
        else
            on.push_back(a);
    }
    sort(on.begin(), on.end(), [&](lineSegment & p1, lineSegment & p2){return p1.self.x<p2.self.x;});

    start = on.front().self;
    end = on.back().self;
    return true;
}

void lineprovider::showImage(const image &img, vector<uint8_t> &rgb) {
    imageTool::yuv422ToRgba(rgb, img.getData(), width, height);
    for ( auto & l : lines) {
        imageTool::drawLine(&rgb[0], l.start, l.end, 255, 0, 0, 2);
    }
}

void lineprovider::saveImage(const image &img, const string &fileName) {
    vector<uint8_t> rgb(width * height * 4, 255);
    showImage(img, rgb);
    imageTool::saveImage(fileName, rgb, width, height);
}
