/***************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  tjarksvd.h
 * @brief some functions for matrix operation.
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 ***************************************************/
#ifndef TJARKMATH_H
#define TJARKMATH_H

#include <cmath>
#include <ctime>
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <point.h>

using namespace std;

namespace tjark_vision {
    const int MAX_ITER = 50;
    const float eps=0.001;

    static inline float get_norm(float *x, int n){
        float r=0;
        for(int i=0;i<n;i++)
            r+=x[i]*x[i];
        return sqrt(r);
    }
    static inline float normalize(float *x, int n){
        float r=get_norm(x,n);
        if(r<eps)
            return 0;
        for(int i=0;i<n;i++)
            x[i]/=r;
        return r;
    }

    static inline float product(float*a, float *b,int n){
        float r=0;
        for(int i=0;i<n;i++)
            r+=a[i]*b[i];
        return r;
    }

    static inline void orth(float *a, float *b, int n){//|a|=1
        float r=product(a,b,n);
        for(int i=0;i<n;i++)
            b[i]-=r*a[i];

    }

    static inline bool svd(vector<vector<float>> A, int K, vector<vector<float>> & U, vector<float> & S, vector<vector<float>> & V){
        int M=A.size();
        int N=A[0].size();
        U.clear();
        V.clear();
        S.clear();
        S.resize(K,0);
        U.resize(K);
        for(int i=0;i<K;i++)
            U[i].resize(M,0);
        V.resize(N);
        for(int i=0;i<N;i++)
            V[i].resize(N,0);


        srand(time(0));
        float *left_vector=new float[M];
        float *next_left_vector=new float[M];
        float *right_vector=new float[N];
        float *next_right_vector=new float[N];
//        int col=0;
        for(int col=0;col<K;col++){
            float diff=1;
            float r=-1;
            while(1){
                for(int i=0;i<M;i++)
                    left_vector[i]= (float)rand() / RAND_MAX;
                if(normalize(left_vector, M)>eps)
                    break;
            }

            for(int iter=0;diff>=eps && iter<MAX_ITER;iter++){
                memset(next_left_vector,0,sizeof(float)*M);
                memset(next_right_vector,0,sizeof(float)*N);
                for(int i=0;i<M;i++)
                    for(int j=0;j<N;j++)
                        next_right_vector[j]+=left_vector[i]*A[i][j];

                r=normalize(next_right_vector,N);
                if(r<eps) break;
                for(int i=0;i<col;i++)
                    orth(&V[i][0],next_right_vector,N);
                normalize(next_right_vector,N);

                for(int i=0;i<M;i++)
                    for(int j=0;j<N;j++)
                        next_left_vector[i]+=next_right_vector[j]*A[i][j];
                r=normalize(next_left_vector,M);
                if(r<eps) break;
                for(int i=0;i<col;i++)
                    orth(&U[i][0],next_left_vector,M);
                normalize(next_left_vector,M);
                diff=0;
                for(int i=0;i<M;i++){
                    float d=next_left_vector[i]-left_vector[i];
                    diff+=d*d;
                }

                memcpy(left_vector,next_left_vector,sizeof(float)*M);
                memcpy(right_vector,next_right_vector,sizeof(float)*N);
            }
            if(r>=eps){
                S[col]=r;
                memcpy((char *)&U[col][0],left_vector,sizeof(float)*M);
                memcpy((char *)&V[col][0],right_vector,sizeof(float)*N);
            }else{
//                cout<<r<<endl;
                break;
            }
        }
        delete [] next_left_vector;
        delete [] next_right_vector;
        delete [] left_vector;
        delete [] right_vector;

        return true;
    }

    static inline void Doolittle(int n,double *A,double *b) {
        double *L = new double[n*n];
        double *U = new double[n*n];
        double *y = new double[n];
        double *x = new double[n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                *(U + i*n + j) = 0;
                if (i==j)
                    *(L + i*n + j) = 1;
                else
                    *(L + i*n + j) = 0;
            }
        }
        for (int k = 0; k < n; k++) {
            for (int j = k; j < n; j++) {
                *(U + k*n + j) = *(A + k*n + j);
                for (int r = 0; r < k; r++)
                    *(U + k*n + j) = *(U + k*n + j) - (*(L + k*n + r)*(*(U + r*n + j)));
            }
            for (int i = k+1; i < n; i++) {
                *(L + i*n + k) = *(A + i*n + k);
                for (int r = 0; r < k; r++) {
                    *(L + i*n + k) = *(L + i*n + k) - (*(L + i*n + r)*(*(U + r*n + k)));
                }
                *(L + i*n + k) = *(L + i*n + k) / (*(U + k*n + k));
            }
        }
        for (int i = 0; i < n; i++) {
            *(y + i) = *(b + i);
            for (int j = 0; j < i; j++) {
                *(y + i) = *(y + i) - *(L + i*n + j)*(*(y + j));
            }
        }
        for (int i = n-1; i >= 0; i--) {
            *(x + i) = *(y + i);
            for (int j = i+1; j < n; j++) {
                *(y + i) = *(y + i) - *(U + i*n + j)*(*(x + j));
            }
            *(x + i) = *(y + i) / (*(U + i*n + i));
        }

        delete[]L;
        delete[]U;
        delete[]y;
    }


    static inline int cholesky(const vector<vector<float>>& a, vector<vector<float>>& l) {
        int m = a.size();
        int n = a[0].size();
        bool spd = (m==n);
        if (!spd)
            return -1;
        l.resize(m);
        for (int i=0; i<m; ++i)
            l[i].resize(m);

        for( int j=0; j<n; ++j ) {
            float d = 0;
            for( int k=0; k<j; ++k ) {
                float s = 0;
                for( int i=0; i<k; ++i )
                    s += l[k][i]*l[j][i];

                l[j][k] = s = (a[j][k]-s) / l[k][k];
                d = d + s*s;
                spd = spd && (a[k][j] == a[j][k]);
            }

            d = a[j][j] - d;
            spd = spd && ( d > 0 );

            l[j][j] = sqrt( d > 0 ? d : 0 );
            for( int k=j+1; k<n; ++k )
                l[j][k] = 0;
        }
    }

    static inline void getLine(point p, float dir, vector<float> & res) {
        res.clear();
        float a, b, c;
        if (fabs(dir-pi/2)<1e-5) {
            a = 1;
            b = 0;
            c = -p.x;
        }
        else {
            a = tan(dir);
            b = -1;
            c = p.y - a*p.x;
        }

        res.push_back(a);
        res.push_back(b);
        res.push_back(c);
    }

    static inline void getLine(point p1, point p2, vector<float> & res) {
        res.clear();
        float a, b, c;
        if (fabs(p1.x-p2.x)<1e-5) {
            a = 1;
            b = 0;
            c = -p1.x;
        }
        else {
            a = float(p1.y-p2.y)/float(p1.x-p2.x);
            b = -1;
            c = -a*p1.x+p1.y;
        }
        res.push_back(a);
        res.push_back(b);
        res.push_back(c);
    }

    static inline bool isPointOnLine(point p1, point p2, point p3, int sigma=1) {
        float a, b, c;
        if (fabs(p1.x-p2.x)<1e-5) {
            a = 1;
            b = 0;
            c = -p1.x;
        }
        else {
            a = float(p1.y-p2.y)/float(p1.x-p2.x);
            b = -1;
            c = -a*p1.x+p1.y;
        }
        float d = a*p3.x - p3.y + c;
        d /= sqrt(a*a+b*b);
        if (fabs(d)<sigma)
            return true;
        return false;
    }

   static inline  void eigenvectors(float a, float b, float eva[2],float eve[][2]){
        float l;

        //ev1
        eve[0][0]=b/(2*(eva[0]-a));
        eve[1][0]=1;
        l = sqrtf(eve[0][0]*eve[0][0]+1);
        eve[0][0]/=l;
        eve[1][0]/=l;
        //ev2
        eve[0][1]=b/(2*(eva[1]-a));
        eve[1][1]=1;
        l = sqrtf(eve[0][1]*eve[0][1]+1);
        eve[0][1]/=l;
        eve[1][1]/=l;
    }

    static inline void eigenvalues(float a, float b, float c,float erg[2]){

        float w = sqrtf(a*a + b*b + c*c - 2*a*c);
        float p = a+c;
        erg[0]=(p + w) / 2;
        erg[1]=(p - w) / 2;
    }

    static inline int det(float a[][2]){
        return (int)roundf(a[0][0]*a[1][1]-a[1][0]*a[0][1]);
    }

    static inline bool getIntersection(vector<float> & l1, vector<float> & l2, point & inter) {
        float d = l1[0]*l2[1] - l2[0]*l1[1];
        if (fabs(d)<1e-5)
            return false;
        inter.x = (l1[1]*l2[2]-l2[1]*l1[2])/d;
        inter.y = (l1[2]*l2[0]-l2[2]*l1[0])/d;
        return true;
    }

    static inline void getMiddlePoint(const point & p1, const point & p2, point & mid) {
        mid.x = (p1.x+p2.x)/2.f;
        mid.y = (p1.y+p2.y)/2.f;
    }

    static inline bool inMiddle(const point & p, const point & p1, const point & p2) {
        int xMax = std::max(p1.x, p2.x);
        int yMax = std::max(p1.y, p2.y);
        int xMin = std::min(p1.x, p2.x);
        int yMin = std::min(p1.y, p2.y);
        return (p.x>xMin && p.x<xMax && p.y>yMin && p.y<yMax);
    }
}

#endif // TJARKMATH_H
