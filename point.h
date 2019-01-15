/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  point.h
 * @brief Basic class for point.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#ifndef POINT_H
#define POINT_H

#include <cmath>
#define pi 3.141592653

class point {
public:
    float x;
    float y;
    point()=default;
    point(float x_, float y_) : x(x_), y(y_) {}

    point operator + (const point& other) {
        return point(this->x+other.x, this->y+other.y);
    }

    point operator - (const point& other) {
        return point(this->x-other.x, this->y-other.y);
    }

    point operator * (const float a) {
        return point(this->x*a, this->y*a);
    }

    point operator / (const float a) {
        return point(this->x/a, this->y/a);
    }

    bool operator < (const point & other) const {
        return this->x<other.x && this->y<other.y;
    }

    bool operator > (const point & other) const {
        return this->x>other.x && this->y>other.y;
    }

    float norm() {
        return sqrtf(x*x+y*y);
    }

    float direction() {
        return atan2(y, x);
    }

    float invdir() {
        float a = atan2(x, y);
        return a>3.141592653/2?3.141592653-a:a;
    }
};

#endif // POINT_H
