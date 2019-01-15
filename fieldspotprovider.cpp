/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  fieldspotprovider.cpp
 * @brief Provide possible spots for further detection.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#include "fieldspotprovider.h"

fieldSpotProvider::fieldSpotProvider()
{
    this->width = 0;
    this->height = 0;
    this->unused.clear();
    this->lineSpots.clear();
    this->obstacleSpots.clear();
    this->obstacleHypo.clear();
    this->ballSpots.clear();
}

fieldSpotProvider::fieldSpotProvider(int w, int h) : width(w), height(h) {
    this->unused.clear();
    this->lineSpots.clear();
    this->obstacleSpots.clear();
    this->obstacleHypo.clear();
    this->ballSpots.clear();
}

void fieldSpotProvider::execute(const classifyImageProvider & cip, const scanlinesProvider &slp, const fieldBoundaryProvider & fbp) {
    this->unused.clear();
    this->lineSpots.clear();
    this->obstacleSpots.clear();
    this->obstacleHypo.clear();
    this->ballSpots.clear();

    for (int i=0; i<slp.getNumberOfScanlines(); ++i) {
        scanline * l = slp.getScanlines()+i;
        for (int j=0; j<l->cnt-1; ++j) {
            if (l->edgeColor[j]==color::white && l->edgeColor[j+1]==color::green) {
                if (l->edge[j+1].y < fbp.getBoundaryByIndex(l->edge[j+1].x)-20)
                    continue;
                int step = l->length[j]>3?2:1;
                float g1 = edgeGradient(cip, l->edge[j].x, l->edge[j].y, step);
                float g2 = edgeGradient(cip, l->edge[j+1].x, l->edge[j+1].y, step);
                point c = point((l->edge[j].x+l->edge[j+1].x)/2, (l->edge[j].y+l->edge[j+1].y)/2);
                float a = (g2 + g1 + pi)/2.f;
                a = (a<0)?-a:a;
                if (fabs(fabs(g1)+fabs(g2)-pi)<1.5f*maxAngError)
                    lineSpots.push_back(lineSegment(c, a, g1, g2));
                else
                    unused.push_back(lineSegment(c, a, g1, g2));
            }
        }
    }

    for (int i=0; i<slp.getNumberOfScanlines(); ++i) {
        scanline * l = slp.getScanlines()+i;
        bool found = false;
        for (int j=0; j<l->cnt-1; ++j) {
            if ((l->edge[j].y-fbp.getBoundaryByIndex(l->edge[j].x))<20)
                continue;
            if (l->edgeColor[j]==color::black) {
                point c = point((l->edge[j].x+l->edge[j+1].x)/2, (l->edge[j].y+l->edge[j+1].y)/2);
                if (checkGreenRatio(cip, c, 10)<0.3f)
                    ballSpots.push_back(c);
            }
            if (l->edgeColor[j]!=color::green) {
                for (int k=j+1; k<l->cnt;++k) {
                    if (l->edgeColor[k]==color::green && l->length[k]>10) {
                        int width = std::max(l->edge[j].x-l->edge[k].x, l->edge[j].y-l->edge[k].y);
                        point c = point((l->edge[j].x+l->edge[k].x)/2, (l->edge[j].y+l->edge[k].y)/2);
                        if (width<80 && width>10) {
                            ballSpots.push_back(c);
                        }
                    }
                    if ((l->edgeColor[k]==color::green && l->length[k]>20) || k==l->cnt-1) {
                        int width = std::max(l->edge[j].x-l->edge[k].x, l->edge[j].y-l->edge[k].y);
                        if (width>minRobotWidth) {
                            found = true;
                            obstacleSpots.push_back(point(l->edge[j].x, l->edge[j].y));
                            j = k;
                        }
                        break;
                    }
                }
            }
//            if (found)
//                break;
        }
    }
    if (!lineSpots.empty())
        sort(lineSpots.begin(), lineSpots.end(),
             [&](lineSegment& s1, lineSegment& s2)
             {if (s1.self.x<s2.self.x) return true;
              if (s1.self.x==s2.self.x && s1.self.y<s2.self.y) return true;
              return false;});
    if (!obstacleSpots.empty()) {
        sort(obstacleSpots.begin(), obstacleSpots.end(),
             [&](obstacleSegment& o1, obstacleSegment& o2)
             {if (o1.self.x<o2.self.x) return true;
              if (o1.self.x==o2.self.x && o1.self.y<o2.self.y) return true;
              return false;});
        int skip = 0;
        vector<vector<point>> group;
        vector<point> tmp;
        for (size_t i=0; i<obstacleSpots.size()-1; i++) {
            int dis = abs(obstacleSpots[i].self.x-obstacleSpots[i+1].self.x);
            int hei = abs(obstacleSpots[i].self.y-obstacleSpots[i+1].self.y);

            if (dis>32 || hei > 20)
                skip++;
            else {
                skip = (--skip<0)?0:skip;
            }
            tmp.push_back(obstacleSpots[i].self);
            if  (skip>1 || dis > 48 || hei > 50) {
                skip = 0;
                group.push_back(tmp);
                tmp.clear();
            }

            if (i == obstacleSpots.size()-2) {
                tmp.push_back(obstacleSpots[i+1].self);
                group.push_back(tmp);
            }
        }
        for (auto & g : group) {
            if (g.size()<2)
                continue;
            int yMax = 0;
            for (auto p : g)
                if (p.y>yMax)
                    yMax = p.y;
            obstacleHypo.push_back(
                        obstacleHypothesis(
                            point(g.front().x, yMax), point(g.back().x, yMax)));
        }
    }
}

float fieldSpotProvider::edgeGradient(const classifyImageProvider &cip, int x, int y, int step) {
    float gx = 0;
    float gy = 0;
    uint8_t lt = cip.getCr(x-step, y-step);
    uint8_t mt = cip.getCr(x        , y-1);
    uint8_t rt = cip.getCr(x+step, y-step);
    uint8_t lm = cip.getCr(x-step, y     );
    uint8_t rm = cip.getCr(x+step, y     );
    uint8_t lb = cip.getCr(x-step, y+step);
    uint8_t mb = cip.getCr(x     , y+step);
    uint8_t rb = cip.getCr(x+step, y+step);
    gx = -(lt+2*lm+lb)+(rt+2*rm+rb);
    gy = -(lt+2*mt+rt)+(lb+2*mb+rb);
    return atan2(gy, gx);
}

float fieldSpotProvider::checkGreenRatio(const classifyImageProvider &cip, const point &p, int r) {
    int cnt=0;
    for (int x=-r; x<=r; ++x)
        for (int y=-r; y<=r; ++y) {
            if (cip.getColor(p.x+x, p.y+y)==color::green)
                cnt++;
        }
    return float(cnt)/float(r*r*4);
}

void fieldSpotProvider::showImage(const image &img, vector<uint8_t> &rgb) {
    imageTool::yuv422ToRgba(rgb, img.getData(), width, height);
    for (const auto & s : lineSpots) {
        imageTool::drawCross(&rgb[0], s.self.x, s.self.y, 255, 0, 0);
        imageTool::drawArray(&rgb[0], s.self.x, s.self.y, s.gradUp, 255, 48, 48);
        imageTool::drawArray(&rgb[0], s.self.x, s.self.y, s.gradDown, 0, 0, 238);
    }

    for (const auto & s : ballSpots) {
        imageTool::drawCross(&rgb[0], s.self.x, s.self.y, 255, 140, 0, 10);
    }
    for (const auto & s : obstacleSpots) {
        imageTool::drawCross(&rgb[0], s.self.x, s.self.y, 255, 255, 255, 8);
    }

    for (auto & o : obstacleHypo) {
        imageTool::drawRect(&rgb[0], o.leftTop, o.rightBot, 0, 191, 255);
    }

}

void fieldSpotProvider::saveImage(const image &img, const string &fileName) {
    vector<uint8_t> rgb(width * height * 4, 255);
    showImage(img, rgb);
    imageTool::saveImage(fileName, rgb, width, height);
}
