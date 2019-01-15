/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  ballprovider.cpp
 * @brief return ball position if there is a ball
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#include "ballprovider.h"

ballProvider::ballProvider()
{
    br.clear();
    points.clear();
    disMap.fill(nullptr);
}

ballProvider::ballProvider(int w, int h) : width(w), height(h) {
    br.clear();
    points.clear();
    candidate.clear();
    points.reserve(h);
    for (int i=0; i<h; ++i) {
        vector<int> tmp;
        points.push_back(tmp);
    }
    disMap.fill(nullptr);
}

void ballProvider::execute(const classifyImageProvider& cip, const fieldSpotProvider &fsp, const edgeImageProvider & eip, const fieldBoundaryProvider &fbp) {
    br.clear();
    points.clear();
    candidate.clear();
    balls.clear();
    disMap.fill(nullptr);
    points.resize(height);

    uint8_t *edge = eip.getEdge();

    auto bs = fsp.getBallSpots();
    for (auto & a: bs)
        br.push_back(ballRegion(a.self-point(50,50), a.self+point(50,50)));
    for (size_t i=0; i<br.size()-1;) {
        ballRegion merged;
        if (iou(br[i], br[i+1], merged)>0.5f) {
            br[i] = merged;
            br.erase(br.begin()+i+1);
        }
        else {
            i++;
        }
    }
    for (const auto b : br) {
        searchRegion(edge, b.leftTop, b.rightBot);
    }
    for (auto & c : candidate) {
        float imgtemp[16*16];
        int cnnResult = -1;
        float scores[2];
        sampleImage(cip, c.center.x, c.center.y, c.radius, 16,imgtemp);
        cnn_classifier(imgtemp, &cnnResult, scores);
        if (cnnResult==1) {
            c.score = scores[1];
            balls.push_back(c);
        }
    }
}

void ballProvider::searchRegion(const uint8_t *edge, point lt, point rb) {
    points.clear();
    points.resize(height);
    for (int i=0; i<height; ++i)
        points.reserve(width);
    for (int y=lt.y; y<rb.y; ++y) {
        for (int x=lt.x; x<rb.x; ++x) {
            if (x>width || x<0) continue;
            if (y>height || y<0) continue;
            if (*(edge+x+y*width)>30) {
                points[y].push_back(x);
            }
        }
    }

    int prob = RAND_MAX / 50;
    for(int y=0; y < height; y++) {
        for(const int & x : points[y]) {
            if(rand() < prob) {
                FRHT(x, y);
            }
        }
    }
}

float ballProvider::iou(const ballRegion &r1, const ballRegion &r2, ballRegion & merged) {
    if (r1.rightBot.x < r2.leftTop.x || r2.rightBot.x < r1.leftTop.x) return 0.f;
    if (r1.rightBot.y < r2.leftTop.y || r2.rightBot.y < r1.leftTop.y) return 0.f;
    if (r1.rightBot < r2.rightBot && r2.leftTop < r1.leftTop) return 1.f;
    if (r2.rightBot < r1.rightBot && r1.leftTop < r2.leftTop) return 1.f;
    float ulX = std::min(r1.leftTop.x, r2.leftTop.x);
    float utY = std::min(r1.leftTop.y, r2.leftTop.y);
    float urX = std::max(r1.rightBot.x, r2.rightBot.x);
    float ubY = std::max(r1.rightBot.y, r2.rightBot.y);
    float ilX = std::max(r1.leftTop.x, r2.leftTop.x);
    float itY = std::max(r1.leftTop.y, r2.leftTop.y);
    float irX = std::min(r1.rightBot.x, r2.rightBot.x);
    float ibY = std::min(r1.rightBot.y, r2.rightBot.y);
    float interSect = (irX-ilX)*(ibY-itY);
    float outerSect = std::min((r1.height*r1.width), (r2.height*r2.width));
    merged = ballRegion(point(ulX, utY), point(urX, ubY));
    return (interSect)/outerSect;
}

void ballProvider::FRHT(int x, int y) {
    vector<circle> tmp;
    int maxAccum = 0;
    for(int winY = static_cast<int>(std::max(0, y - windowRadius)); winY < static_cast<int>(std::min(height, y + windowRadius)); winY++) {
        for(const int& winX : points[winY]) {
            if(winX < std::max(0, x - windowRadius) || (winY == y && winX == x))
                continue;
            if(winX >= std::min(width, x + windowRadius))
                break;
            int distance = std::round(std::hypot(static_cast<float>(winX)-x, static_cast<float>(winY)-y));
            if (distance>=acceptedDis) {
                if (disMap[distance]==nullptr) {
                    disMap[distance] = new point(winX, winY);
                }
                else {
                    circle can(x, y, winX, winY, disMap[distance]->x, disMap[distance]->y);
                    if (!isnan(can.radius) && !isnan(can.center.x) && !isnan(can.center.y)) {
                        if (can.radius>=minRadius && can.radius<=maxRadius) {
                            bool prevFound = false;
                            for (circle & c : tmp) {
                                if (fabs(c.radius-can.radius) <= acceptRadiusError
                                        && (c.center-can.center).norm() <= acceptDistanceError) {
                                    c.radius = (c.radius+can.radius)/2.f;
                                    c.center = (c.center+can.center)/2.f;
                                    maxAccum = std::max(maxAccum, ++c.accum);
                                    prevFound = true;
                                }
                            }
                            if (!prevFound)
                                tmp.push_back(can);
                        }
                    }
                }
            }
        }
    }
    if (maxAccum >= acceptThresh) {
        for (const circle & can : tmp) {
            if (can.accum == maxAccum) {
                bool prevFound = false;
                for (circle & c : candidate) {
                    if (fabs(c.radius-can.radius) <= acceptRadiusError
                            && (c.center-can.center).norm() <= acceptDistanceError) {
                        c.radius = (c.radius+can.radius)/2.f;
                        c.center = (c.center+can.center)/2.f;
                        c.accum += can.accum;
                        prevFound = true;
                    }
                }
                if (!prevFound)
                    candidate.push_back(can);
            }
        }
    }
    for (point * & d : disMap)
        if (d != nullptr) {
            delete d;
            d = nullptr;
        }
}

void ballProvider::sampleImage(const classifyImageProvider& cip, int px, int py, int radius, int target, float* sampleImage) const {
    float imageScale = float(radius) / float(target) * 2.f;
    int x = static_cast<int>(px - radius);
    int y = static_cast<int>(py - radius);

    int tempI, tempJ;

    for (int i = 0; i < target; ++i) {
        for (int j = 0; j < target; ++j) {
            tempI = y + static_cast<int>(i * imageScale);
            tempJ = x + static_cast<int>(j * imageScale);
            if (tempI >= height)
            tempI = height - 1;
            if (tempJ >= width)
            tempJ = width - 1;
            if (tempI < 0)
            tempI = 0;
            if (tempJ < 0)
            tempJ = 0;
            sampleImage[i + j*target] = cip.getCy(tempJ,tempI);
        }
    }
}

void ballProvider::showImage(const image &img, vector<uint8_t> &rgb) {
    imageTool::yuv422ToRgba(rgb, img.getData(), width, height);
//    for (const auto & s : br) {
//        imageTool::drawRect(&rgb[0], s.leftTop, s.rightBot, 0, 191, 255, false);
//    }
    for (auto & c : candidate) {
//        imageTool::drawCross(&rgb[0], c.center.x, c.center.y, 255, 69, 0, 10);
//        imageTool::drawRect(&rgb[0], point(c.center-point(c.radius, c.radius)), point(c.center+point(c.radius, c.radius)), 0, 0, 255, false);
    }
    for (auto & c : balls) {
//        imageTool::drawCross(&rgb[0], c.center.x, c.center.y, 255, 69, 0, 10);
        imageTool::drawRect(&rgb[0], point(c.center-point(c.radius, c.radius)), point(c.center+point(c.radius, c.radius)), 255, 0, 0, false);
    }
}

void ballProvider::saveImage(const image &img, const string &fileName) {
    vector<uint8_t> rgb(width * height * 4, 255);
    showImage(img, rgb);
    imageTool::saveImage(fileName, rgb, width, height);
}
