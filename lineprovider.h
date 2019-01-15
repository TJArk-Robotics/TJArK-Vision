/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  lineprovider.h
 * @brief Detect lines and return positions.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#ifndef LINEPROVIDER_H
#define LINEPROVIDER_H

#include <vector>
#include <algorithm>
#include <random>
#include <point.h>
#include <tjarkmath.h>
#include <classifyimageprovider.h>
#include <scanlinesprovider.h>
#include <fieldboundaryprovider.h>
#include <fieldspotprovider.h>

using namespace std;

class line {
public:
    point start;
    point end;
    line() = default;
    line(point s, point e) : start(s), end(e) {}

    float length() {
        return (start-end).norm();
    }

    bool isPointOnLine(point & p, int sigma=1) {
        return tjark_vision::isPointOnLine(start, end, p, sigma);
    }
};

//class lineSegment {
//public:
//    point self;
//    float angle;
//    lineSegment() {
//        self = point(-1, -1);
//        angle = 0;
//    }
//    lineSegment(point p, float a) : self(p), angle(a) {
//    }
//};

class lineprovider
{
private:
    int width;
    int height;

    vector<line> lines;
    vector<lineSegment> spots;
    vector<lineSegment> segment;

    float maxDisError = 50.f;
    float maxAngError = 15*pi/180.f;

public:
    lineprovider();
    lineprovider(int w, int h);

    std::mt19937 seed;
    std::uniform_real_distribution<> distrib{0,1};

    void execute(const fieldSpotProvider &fsp);
    bool ransacLine(const vector<lineSegment> &p, point & start, point & end, vector<lineSegment> &res, float sigma=1, int iter=20) ;

    void showImage(const image & img, vector<uint8_t> &rgb);
    void saveImage(const image &img, const string &fileName);
};

#endif // LINEPROVIDER_H
