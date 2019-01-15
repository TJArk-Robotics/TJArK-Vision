/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  ballprovider.h
 * @brief return ball position if there is a ball
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#ifndef BALLPROVIDER_H
#define BALLPROVIDER_H

#include <vector>
#include <array>
#include <algorithm>
#include <random>
#include <point.h>
#include <tjarkmath.h>
#include <imagetool.h>
#include <classifyimageprovider.h>
#include <scanlinesprovider.h>
#include <fieldboundaryprovider.h>
#include <fieldspotprovider.h>
#include <edgeimageprovider.h>
#include <cnn_classifier.h>

struct ballRegion {
    point leftTop;
    point rightBot;
    int width;
    int height;
    ballRegion() = default;
    ballRegion(point l, point r) : leftTop(l), rightBot(r) {
        width = rightBot.x-leftTop.x;
        height = rightBot.y-leftTop.y;
    }
};

struct circle
{
  public:
    point center;
    float radius;
    float score;
    int accum;

    circle(int x1, int y1, int x2, int y2, int x3, int y3) : center(point(0,0)), radius(nanf("")), accum(1), score(0.f) {
        if((x1 != x3 || y1 != y3) && (x2 != x3 || y2 != y3) && (x1 != x2 || y1 != y2) && (x1 != x3 || x2 != x3)) {
            float p1x, p1y, p2x, p2y, p3x, p3y;
            if(x1 == x3 || y1 == y3) {
                if(x1 == x2) {
                    p1x = (float)x2;
                    p1y = (float)y2;
                    p2x = (float)x3;
                    p2y = (float)y3;
                    p3x = (float)x1;
                    p3y = (float)y1;
                }
                else {
                    p1x = (float)x3;
                    p1y = (float)y3;
                    p2x = (float)x2;
                    p2y = (float)y2;
                    p3x = (float)x1;
                    p3y = (float)y1;
                }
            }
            else {
                if(x1 == x2) {
                    p1x = (float)x1;
                    p1y = (float)y1;
                    p2x = (float)x3;
                    p2y = (float)y3;
                    p3x = (float)x2;
                    p3y = (float)y2;
                }
                else {
                    p1x = (float)x3;
                    p1y = (float)y3;
                    p2x = (float)x1;
                    p2y = (float)y1;
                    p3x = (float)x2;
                    p3y = (float)y2;
                }
            }
            float ma, mb;
            if(p1x != p2x)
                ma = (p2y - p1y) / (p2x - p1x);
            else
                ma = 1000000;
            if(p2x != p3x)
                mb = (p3y - p2y) / (p3x - p2x);
            else
                mb = 1000000;
            if(ma != 0 || mb != 0) {
                center.x = (ma * mb * (p1y - p3y) + mb * (p1x + p2x) - ma * (p2x + p3x)) / (2 * (mb - ma));
                if(ma != 0)
                    center.y = -(center.x - (p1x + p2x) / 2) / ma + (p1y + p2y) / 2;
                else
                    center.y = -(center.x - (p2x + p3x) / 2) / mb + (p2y + p3y) / 2;
                radius = (center - point((float)x1, (float)y1)).norm();
            }
        }
    }
};

class ballProvider
{
private:
    int width;
    int height;

    int windowRadius = 12;
    int acceptedDis = 8;
    int minRadius = 5;
    int maxRadius = 40;
    int acceptRadiusError = 10;
    int acceptDistanceError = 10;
    unsigned int acceptThresh = 50;

    vector<ballRegion> br;
    vector<vector<int>> points;
    vector<circle> candidate;
    vector<circle> balls;
    array<point*, 16*3> disMap;

public:
    ballProvider();
    ballProvider(int w, int h);

    void execute(const classifyImageProvider& cip, const fieldSpotProvider & fsp, const edgeImageProvider & eip, const fieldBoundaryProvider & fbp);
    float iou(const ballRegion & r1, const ballRegion & r2, ballRegion &merged);
    void searchRegion(const uint8_t * edge, point lt, point rb);
    void FRHT(int x, int y);
    void sampleImage(const classifyImageProvider& cip, int px, int py, int radius, int target, float* sampleImage) const;

    void showImage(const image & img, vector<uint8_t> &rgb);
    void saveImage(const image & img, const string &fileName);
};

#endif // BALLPROVIDER_H
