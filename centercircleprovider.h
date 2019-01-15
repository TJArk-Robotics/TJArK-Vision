/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  cevtercircleprovider.h
 * @brief return center circle position if exists
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#ifndef CENTERCIRCLEPROVIDER_H
#define CENTERCIRCLEPROVIDER_H

#include <vector>
#include <algorithm>
#include <point.h>
#include <tjarkmath.h>
#include <fieldspotprovider.h>

using namespace std;

class Ellipse{
public:
    float a,b,c,d,e,f;
    float a1,b1,c1,d1,e1,f1;
    float ta, tb;
    float focus;
    float trans[2][2];
    float translation[2];

    Ellipse() : a(0),b(0),c(0),d(0),e(0),f(0),
        a1(0),b1(0),c1(0),d1(0),e1(0),f1(0),
        ta(0),tb(0), focus(0) {
        trans[0][0] = 0;
        trans[0][1] = 0;
        trans[1][0] = 0;
        trans[1][1] = 0;

        translation[0] = 0;
        translation[1] = 0;
    }

    Ellipse(const std::vector<float> &p) : a(p[0]), b(p[1]), c(p[2]), d(p[3]), e(p[4]), f(p[5]),
            a1(0),b1(0),c1(0),d1(0),e1(0),f1(0), ta(0),tb(0), focus(0) {
        trans[0][0] = 0;
        trans[0][1] = 0;
        trans[1][0] = 0;
        trans[1][1] = 0;

        translation[0] = 0;
        translation[1] = 0;
    }

    void operator = (const Ellipse &other) {
        ta = other.ta;
        tb = other.tb;
        for (int i=0; i<2; i++)
            for (int j=0; j<2; j++)
                trans[i][j] = other.trans[i][j];
        focus = other.focus;
        translation[0] = other.translation[0];
        translation[1] = other.translation[1];
    }

};

class centerCircleProvider
{
private:
    int width;
    int height;
    float S[7][7];
    float L[7][7];
    float C[7];
    float invL[7][7];

    Ellipse centerCircle;
    bool centerCircleFound;

    vector<point> bestCandidates;
    vector<lineSegment> bestSegments;

public:

    std::mt19937 seed;
    std::uniform_real_distribution<> distrib{0,1};

    float maxAngError = 15*pi/180.f;
    float maxDisError = 60.f;

    centerCircleProvider();
    centerCircleProvider(int w, int h):width(w), height(h){}

    vector<point> crosses;

    void execute(const fieldSpotProvider & fsp);
    bool computeModleCoefficient(const vector<point> & sample, vector<float> coef);
    int choldc(float a[][7], int n, float l[][7]);
    int inverse(float TB[][7], float InvB[][7], int N);
    bool ellipseFit(const vector<point> & points, vector<float> & result);
    void Doolittle(vector<float> & a);

    float generateEllipsePoint(float x, bool min) {
        float a = sqrtf((25*16-16*x*x)/25);
        return min?-a:a;
    }

    int transEllipse(Ellipse &el);
    float getRating(const vector<lineSegment> &seg, const Ellipse& e);
    float distanceToEllipse(float x, float y, Ellipse el);
    void projectPointToEllipse(point &p, const float trans[][2], const float translation[2]);
    void projectPointFromEllipse(point &p, const float trans[][2], const float translation[2]);

    void reset();

    void showImage(const image & img, vector<uint8_t> &rgb);
    void saveImage(const image &img, const string &fileName);
};

#endif // CENTERCIRCLEPROVIDER_H
