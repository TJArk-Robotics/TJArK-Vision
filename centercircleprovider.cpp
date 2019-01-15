/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  cevtercircleprovider.cpp
 * @brief return center circle position if exists
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 * ANNOUNCE： This file is partly from HTWKVision
 *
 ***************************************************/
#include "centercircleprovider.h"

centerCircleProvider::centerCircleProvider()
{
    centerCircleFound = false;
    reset();
}

void centerCircleProvider::execute(const fieldSpotProvider &fsp) {

    size_t n = fsp.getLineSpots().size();
    crosses.clear();
    vector<lineSegment> segment = fsp.getLineSpots();
    centerCircleFound = false;
    vector<float> result;
    vector<point> sample;

    bool foundBest=false;
    float maxRating=0.35f;
    Ellipse bestEllipse;
    for (int iter=0; iter<20; iter++) {
        reset();
        sample.clear();
        float S11=0,S12=0,S13=0,S14=0,S15=0,S16=0,S23=0,S25=0,S26=0,S33=0,S35=0,S36=0,S46=0,S56=0;
        vector<vector<float>> a;
        for (int i=0; i<6; ++i) {
            lineSegment seg = segment[(int)(distrib(seed)*n)];
            sample.push_back(seg.self);
            }
            if (!ellipseFit(sample, result))
                continue;
            Ellipse e(result);
            if (transEllipse(e)==0) {
                float rating = getRating(segment, e);
                if (rating > maxRating) {
                    bestEllipse = e;
                    maxRating = rating;
                    foundBest = true;
                }
            }
        }

    if (!foundBest)
        return;

    bestCandidates.clear();
    bestSegments.clear();

    for (const lineSegment & s : segment) {
        float dis = distanceToEllipse(s.self.x, s.self.y, bestEllipse);
        if (fabs(dis) < 3) {
            bestCandidates.push_back(s.self);
            bestSegments.push_back(s);
        }
    }
    if (!ellipseFit(bestCandidates, result))
        return;
    Ellipse betterE(result);
    if(transEllipse(betterE)!=0)
        return;
    float betterRating=getRating(bestSegments,betterE);
    if (betterRating<0.35f)
        return;
    centerCircleFound = foundBest;
    centerCircle = betterE;
}

void centerCircleProvider::showImage(const image &img, vector<uint8_t> &rgb) {
    imageTool::yuv422ToRgba(rgb, img.getData(), width, height);
    for (auto & p : bestCandidates)
        imageTool::drawCross(&rgb[0], p.x, p.y, 0,0,0,10);
    if (!centerCircleFound) return;
    float stepWidth=M_PI*2/64;
    float lastX=0;
    float lastY=0;
    for(double a=0;a<=M_PI*2+0.00001;a+=stepWidth){
        float x=sinf(a);
        float y=cosf(a);
        point p;
        p.x=x*centerCircle.ta;
        p.y=y*centerCircle.tb;
        projectPointFromEllipse(p, centerCircle.trans, centerCircle.translation);
        float px=p.x;
        float py=p.y;
        if (a>0)
            for(float d=0;d<=1;d+=0.01){
                int nx=(int)round(px*(1-d)+lastX*d);
                int ny=(int)round(py*(1-d)+lastY*d);
                if(nx<0||ny<0||nx>=width||ny>=height)continue;
                imageTool::drawCross(&rgb[0], nx, ny, 255, 20, 150, 2);
            }

        lastX=px;
        lastY=py;
    }
//    point cWei = point(0, 0);
//    projectPointFromEllipse(cWei, centerCircle.trans, centerCircle.translation);
//    imageTool::drawCross(&rgb[0], cWei.x, cWei.y, 255, 20, 150, 10);
}

void centerCircleProvider::saveImage(const image &img, const string &fileName) {
    vector<uint8_t> rgb(width * height * 4, 255);
    showImage(img, rgb);
    imageTool::saveImage(fileName, rgb, width, height);
}

bool centerCircleProvider::computeModleCoefficient(const vector<point> &sample, vector<float> coef) {

}

bool centerCircleProvider::ellipseFit(const vector<point> &points, vector<float> &result) {
    result.clear();
    result.resize(6);
    if (points.size() < 6)
        return false;

    float S11=0,S12=0,S13=0,S14=0,S15=0,S16=0,S23=0,S25=0,S26=0,S33=0,S35=0,S36=0,S46=0,S56=0;
        for (auto & p : points) {
            float tx = p.x;
            float ty = p.y;
            float txtx=tx*tx;
            float txty=tx*ty;
            float tyty=ty*ty;
            S11+=txtx*txtx;
            S12+=txtx*txty;
            S13+=txtx*tyty;
            S23+=txty*tyty;
            S33+=tyty*tyty;
            S14+=txtx*tx;
            S15+=txtx*ty;
            S25+=txty*ty;
            S35+=tyty*ty;
            S16+=txtx;
            S26+=txty;
            S36+=tyty;
            S46+=tx;
            S56+=ty;
        }
        S[1][1]=S11;
        S[2][1]=S[1][2]=S12;
        S[2][2]=S[3][1]=S[1][3]=S13;
        S[3][2]=S[2][3]=S23;
        S[3][3]=S33;
        S[4][1]=S[1][4]=S14;
        S[4][2]=S[2][4]=S[5][1]=S[1][5]=S15;
        S[4][3]=S[3][4]=S[5][2]=S[2][5]=S25;
        S[5][3]=S[3][5]=S35;
        S[4][4]=S[6][1]=S[1][6]=S16;
        S[5][4]=S[4][5]=S[6][2]=S[2][6]=S26;
        S[5][5]=S[6][3]=S[3][6]=S36;
        S[6][4]=S[4][6]=S46;
        S[6][5]=S[5][6]=S56;
        S[6][6]=points.size();

        if(choldc(S, 6, L)!=0)
            return false;


        if(inverse(L, invL, 6) != 0)
            return false;

        C[1] = invL[1][1] * -4 * invL[1][3] + invL[1][2] * invL[1][2];
        C[2] = invL[2][1] * -4 * invL[2][3] + invL[2][2] * invL[2][2];
        C[3] = invL[3][1] * -4 * invL[3][3] + invL[3][2] * invL[3][2];
        C[4] = invL[4][1] * -4 * invL[4][3] + invL[4][2] * invL[4][2];
        C[5] = invL[5][1] * -4 * invL[5][3] + invL[5][2] * invL[5][2];
        C[6] = invL[6][1] * -4 * invL[6][3] + invL[6][2] * invL[6][2];

        for (int j=1;j<=6;j++)
          {
            float mod = 0.0;
            for (int i=1;i<=6;i++)
              mod += invL[j][i]*invL[j][i];

            if(mod == 0)
                return false;

            for (int i=1;i<=6;i++)
                invL[j][i] /=  sqrtf(mod);
          }

        float zero = 10e-20;
        int solind = 0;

        for (int j = 1; j <= 6; j++)
            if (C[j] < -zero)
                solind = j;
        bool allZero=true;
        for (int i = 1; i <= 6; i++) {
            result[i-1] = invL[solind][i];
            if(result[i-1]!=0)
                allZero=false;
        }
        return !allZero;
}

int centerCircleProvider::choldc(float a[][7], int n, float l[][7]) {
    int i, j, k;
        float sum;
        float *p = (float*)malloc(sizeof(float)*(n + 1));
        if (p == nullptr) {
            fprintf(stderr, "Error: malloc() returned NULL in file %s line %d. Exiting.\n", __FILE__, __LINE__);
            exit(EXIT_FAILURE);
        }

        for (i = 1; i <= n; i++) {
            for (j = i; j <= n; j++) {
                for (sum = a[i][j], k = i - 1; k >= 1; k--)
                    sum -= a[i][k] * a[j][k];
                if (i == j) {
                    if (sum <= 0.0f)
                    {
                        free(p);
                        return -1;
                    } else
                        p[i] = sqrtf(sum);
                } else {
                    if(p[i] == 0) {
                        free(p);
                        return -1;
                    }
                    a[j][i] = sum / p[i];
                }
            }
        }
        for (i = 1; i <= n; i++)
            for (j = i; j <= n; j++)
                if (i == j)
                    l[i][i] = p[i];
                else {
                    l[j][i] = a[j][i];
                    l[i][j] = 0.0f;
                }
        free(p);
        return 0;
}

int centerCircleProvider::inverse(float TB[][7], float InvB[][7], int N) {
    int k, i, j, p, q;
       float mult;
       float D, temp;
       float maxpivot;
       int npivot;
       float A[N+1][2*N+2];
       float eps = 10e-20;

       for (k = 1; k <= N; k++) {
           for (j = 1; j <= N; j++)
               A[k][j] = TB[k][j];
           for (j = N + 1; j <= 2 * N + 1; j++)
               A[k][j] = (float) 0;
           A[k][k - 1 + N + 2] = (float) 1;
       }
       for (k = 1; k <= N; k++) {
           maxpivot = fabs((float) A[k][k]);
           npivot = k;
           for (i = k; i <= N; i++)
               if (maxpivot < fabs((float) A[i][k])) {
                   maxpivot = fabs((float) A[i][k]);
                   npivot = i;
               }
           if (maxpivot >= eps) {
               if (npivot != k)
                   for (j = k; j <= 2 * N + 1; j++) {
                       temp = A[npivot][j];
                       A[npivot][j] = A[k][j];
                       A[k][j] = temp;
                   };
               D = A[k][k];
               if(D==0 || D == numeric_limits<float>::infinity() || D == -numeric_limits<float>::infinity())
                   return -1;
               for (j = 2 * N + 1; j >= k; j--)
                   A[k][j] = A[k][j] / D;
               for (i = 1; i <= N; i++) {
                   if (i != k) {
                       mult = A[i][k];
                       for (j = 2 * N + 1; j >= k; j--)
                           A[i][j] = A[i][j] - mult * A[k][j];
                   }
               }
           } else { // The matrix may be singular
               return (-1);
           };
       }
       for (k = 1, p = 1; k <= N; k++, p++)
           for (j = N + 2, q = 1; j <= 2 * N + 1; j++, q++)
               InvB[p][q] = A[k][j];
       return (0);
}

void centerCircleProvider::reset() {
    for(int j=0;j<7;j++){
        for(int i=0;i<7;i++){
            S[j][i]=0;
            L[j][i]=0;
            invL[j][i]=0;
        }
    }
}
int centerCircleProvider::transEllipse(Ellipse &el){
    float eva[2];
    float temp1, temp2;

    tjark_vision::eigenvalues(el.a,el.b,el.c,eva);
    if(eva[0]==el.a||eva[1]==el.a)
        return -1;
    tjark_vision::eigenvectors(el.a,el.b,eva,el.trans);
    if(tjark_vision::det(el.trans)==-1){
        el.trans[0][1]*=-1;
        el.trans[1][1]*=-1;
    }
    el.a1=eva[0];
    el.b1=0;
    el.c1=eva[1];
    temp1=el.trans[0][0]* el.d + el.trans[1][0]*el.e;
    temp2=el.trans[0][1]* el.d + el.trans[1][1]*el.e;
    el.d1=temp1;
    el.e1=temp2;
    el.translation[0]= el.d1 / (2 * el.a1);
    el.translation[1]= el.e1 / (2 * el.c1);;
    el.f1=el.f-(  ((el.d1*el.d1)/(4*(el.a1)))  + (el.e1*el.e1)/(4*(el.c1))  );
    if(-el.f1/el.a1<0||-el.f1/el.c1<0)
        return -1;
    el.ta=sqrtf(-el.f1/el.a1);
    el.tb=sqrtf(-el.f1/el.c1);

    if(numeric_limits<float>::infinity() == el.ta || el.tb == numeric_limits<float>::infinity())
        return -1;

    el.focus=sqrtf(fabsf(- el.ta*el.ta + el.tb*el.tb));
    return 0;
}

float centerCircleProvider::getRating(const vector<lineSegment> &seg, const Ellipse& e){
    int histo[36];
    memset(histo,0,sizeof(int)*36);
    for(const lineSegment & s : seg){
        point p=s.self;
        projectPointToEllipse(p, e.trans, e.translation);

        if(e.ta == 0 || e.tb == 0)
            continue;

        float px=p.x/e.ta;
        float py=p.y/e.tb;
        float dist=px*px+py*py;
        if(dist>1.1f*1.1f||dist<0.9f*0.9f){
            continue;
        }
        float angle=atan2(py,px)+M_PI;
        int index=((int)(angle/(M_PI*2)*36))%36;
        histo[index]++;
    }
    float rating=0;
    for(int angle=0;angle<18;angle++){
        if(histo[angle]>0&&histo[angle+36/2]>0){
            rating+=100+histo[angle];
        }
    }
    rating/=100.f*36/2;
    return rating;
}

float centerCircleProvider::distanceToEllipse(float x, float y, Ellipse el){
    point p=point(x,y);
    projectPointToEllipse(p, el.trans, el.translation);
    x=p.x;
    y=p.y;
    float dis=-INFINITY;
    if(el.tb<el.ta){
        float e = powf(y,2);
        float aspect=1;
        dis = aspect*(sqrt(powf(el.focus-x,2)+e)+sqrt(powf(-el.focus-x,2)+e)-2*el.ta);
    }else{
        float e = powf(x,2);
        float aspect=1;
        dis = aspect*(sqrt(powf(el.focus-y,2)+e)+sqrt(powf(-el.focus-y,2)+e)-2*el.tb);
    }
    return dis;
}

void centerCircleProvider::projectPointToEllipse(point &p, const float trans[][2], const float translation[2]) {
    float temp1=trans[0][0]* p.x + trans[1][0]*p.y;
    float temp2=trans[0][1]* p.x + trans[1][1]*p.y;
    p.x=temp1+translation[0];
    p.y=temp2+translation[1];
}

void centerCircleProvider::projectPointFromEllipse(point &p, const float trans[][2], const float translation[2]) {
    p.x-=translation[0];
    p.y-=translation[1];
    float temp1=trans[1][1]* p.x - trans[1][0]*p.y;
    float temp2=-trans[0][1]* p.x + trans[0][0]*p.y;
    p.x=temp1;
    p.y=temp2;
}
