/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  fieldspotprovider.h
 * @brief Provide possible spots for further detection.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#ifndef FIELDSPOTPROVIDER_H
#define FIELDSPOTPROVIDER_H

#include <vector>
#include <algorithm>
#include <point.h>
#include <classifyimageprovider.h>
#include <scanlinesprovider.h>
#include <fieldboundaryprovider.h>

class lineSegment {
public:
    point self;
    float angle;
    float gradUp;
    float gradDown;
    lineSegment() {
        self = point(-1, -1);
        angle = 0;
        gradUp = 0;
        gradDown = 0;
    }
    lineSegment(point p, float a) : self(p), angle(a) {
        gradUp = 0;
        gradDown = 0;
    }
    lineSegment(point p, float a, float g1, float g2) : self(p), angle(a), gradUp(g1), gradDown(g2) {}
};

class obstacleSegment {
public:
    point self;
    obstacleSegment() = default;
    obstacleSegment(point p) : self(p) {}
};

class obstacleHypothesis {
public:
    point leftTop;
    point rightTop;
    point leftBot;
    point rightBot;
    int width;
    int height;
    obstacleHypothesis() = default;
    obstacleHypothesis(point l, point r){
        width = r.x - l.x;
        height = width;
        leftTop = point(l.x, (l.y-height)<0?0:(l.y-height));
        leftBot = point(l.x, l.y);
        rightTop = point(r.x, (r.y-height)<0?0:(r.y-height));
        rightBot = r;
    }
};

class ballSegment {
public:
    point self;
    ballSegment() = default;
    ballSegment(point p) : self(p) {}
};

class fieldSpotProvider
{
private:
    int width;
    int height;

    vector<lineSegment> unused;
    vector<lineSegment> lineSpots;
    vector<obstacleSegment> obstacleSpots;
    vector<obstacleHypothesis> obstacleHypo;
    vector<ballSegment> ballSpots;

    float maxAngError = 15*pi/180.f;
    float minRobotWidth = 50;

public:
    fieldSpotProvider();
    fieldSpotProvider(int w, int h);

    void execute(const classifyImageProvider & cip, const scanlinesProvider &slp, const fieldBoundaryProvider & fbp);
    float edgeGradient(const classifyImageProvider &cip, int x, int y, int step);
    float checkGreenRatio(const classifyImageProvider & cip, const point & p, int r);

    vector<lineSegment> getLineSpots() const {
        return lineSpots;
    }

    vector<ballSegment> getBallSpots() const {
        return ballSpots;
    }

    void showImage(const image & img, vector<uint8_t> &rgb);
    void saveImage(const image & img, const string &fileName);
};

#endif // FIELDSPOTPROVIDER_H
