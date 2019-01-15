#ifndef FIELDBOUNDARYPROVIDER_H
#define FIELDBOUNDARYPROVIDER_H

#include <vector>
#include <random>
#include <image.h>
#include <point.h>
#include <imagetool.h>
//#include <ransacline.h>
/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  fieldboundaryprovider.h
 * @brief Detect field boundary.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#include <scanlinesprovider.h>

class fieldBoundaryProvider
{
private:
    int width;
    int height;
    int maxSkip = 30;
    vector<point> candidates;
    vector<int> fieldBoundary;
    int upperBoundary;

public:
    fieldBoundaryProvider();
    fieldBoundaryProvider(int w, int h);

    std::mt19937 seed;
    std::uniform_real_distribution<> distrib{0,1};

    void execute(const scanlinesProvider & slp);
    void showImage(const image & img, vector<uint8_t> &rgb);
    void saveImage(const image &img, const string &fileName);
    bool ransacLine(const vector<point> & p, vector<float> & bestLine, vector<point> & res,
                   float sigma=2, int iter=20);
    vector<int> getFieldBoundary() const {
        vector<int> tmp = fieldBoundary;
        return  tmp;
    }

    int getBoundaryByIndex(int x) const {
        if (x<0 || x>width)
            return 0;
        return fieldBoundary[x];
    }

    int getMinBoundary() const {
        return upperBoundary;
    }
};

#endif // FIELDBOUNDARYPROVIDER_H
