/******************************************************
 *
 * This is the official code release of Team TJArk
 * in RoboCup SPL.
 *
 * Licensed under the GNU General Public License v3.0
 * @copyright TJArk, Tongji University, China
 *
 * @file  main.cpp
 * @brife demo for vision system in SPL games
 * @date  2019-01-14
 *
 * @auther Mr. Tree <tongjilishu@163.com>
 * <a href="mailto:tongjilishu@163.com">Mr. Tree</a>
 *
 * CAUTION: Due to the lack of Camera Pose (i.e. Camera
 * Matrix), the perception performance has been reduced.
 *
 ******************************************************/
#include <iostream>
#include <string>
#include <chrono>

#include <imagetool.h>
#include <image.h>
#include <fieldcolorprovider.h>
#include <classifyimageprovider.h>
#include <edgeimageprovider.h>
#include <scanlinesprovider.h>
#include <fieldboundaryprovider.h>
#include <fieldspotprovider.h>
#include <centercircleprovider.h>
#include <lineprovider.h>
#include <ballprovider.h>

#include <fstream>


using namespace std;
using namespace std::chrono;

int main(int argc, char* argv[])
{

    if (argc < 2) {
        cout << "Please Enter Image Path..." << endl;
        return -1;
    }
    const string fileName = argv[1];//"I:\\Projects\\tjark\\TJArkVision\\1\\pictures\\alt\\6.png";

    uint32_t w = 640;
    uint32_t h = 480;
//    c.generate();
    image imageYUV(w, h);
    fieldColorProvider fcp(w, h);
    classifyImageProvider cip(w, h);
    scanlinesProvider slp(w, h);
    fieldBoundaryProvider fbp(w, h);
    edgeImageProvider eip(w, h);
    fieldSpotProvider fsp(w, h);
    centerCircleProvider ccp(w, h);
    lineprovider lip(w, h);
    ballProvider bp(w, h);


    imageYUV.loadImage(fileName);
    auto t_loadingImage = system_clock::now();
        fcp.execute(imageYUV);
    auto t_fcp = system_clock::now();
        cip.execute(imageYUV, fcp);
    auto t_cip = system_clock::now();
        slp.execute(cip);
    auto t_slp = system_clock::now();
        fbp.execute(slp);
    auto t_fbp = system_clock::now();
        eip.execute(cip, fbp, channel::y);
    auto t_eip = system_clock::now();
        fsp.execute(cip, slp, fbp);
    auto t_fsp = system_clock::now();
        ccp.execute(fsp);
    auto t_ccp = system_clock::now();
        lip.execute(fsp);
    auto t_lip = system_clock::now();
        bp.execute(cip, fsp, eip, fbp);
    auto t_bp = system_clock::now();
    auto t_end = system_clock::now();

    auto d_fcp = duration_cast<microseconds>(t_fcp - t_loadingImage);
    auto d_cip = duration_cast<microseconds>(t_cip - t_fcp);
    auto d_slp = duration_cast<microseconds>(t_slp - t_cip);
    auto d_fbp = duration_cast<microseconds>(t_fbp - t_slp);
    auto d_eip = duration_cast<microseconds>(t_eip - t_fbp);
    auto d_fsp = duration_cast<microseconds>(t_fsp - t_eip);
    auto d_ccp = duration_cast<microseconds>(t_ccp - t_fsp);
    auto d_lip = duration_cast<microseconds>(t_lip - t_ccp);
    auto d_bp = duration_cast<microseconds>(t_bp - t_lip);
    auto d_end = duration_cast<microseconds>(t_end - t_loadingImage);

    cout << "d_FieldColorProvider = " << d_fcp.count()/1000.f << "ms" <<endl;
    cout << "d_ColoredImageProvider = " << d_cip.count()/1000.f << "ms" <<endl;
    cout << "d_EdgeImageProvider = " << d_eip.count()/1000.f << "ms" <<endl;
    cout << "d_ScanlineProvider = " << d_slp.count()/1000.f << "ms" <<endl;
    cout << "d_FieldBoundaryProvider = " << d_fbp.count()/1000.f << "ms" <<endl;
    cout << "d_FieldSpotsProvider = " << d_fsp.count()/1000.f << "ms" <<endl;
    cout << "d_CenterCircleProvider = " << d_ccp.count()/1000.f << "ms" <<endl;
    cout << "d_LineProvider = " << d_lip.count()/1000.f << "ms" <<endl;
    cout << "d_BallProvider = " << d_bp.count()/1000.f << "ms" <<endl;
    cout << "d_TimeTotal = " << d_end.count()/1000.f << "ms" <<endl;

    vector<uint8_t> fcpResult(w * h * 4, 255);
    fcp.showImage(imageYUV, fcpResult);
    const string fcpFile = "color.png";
    imageTool::saveImage(fcpFile, fcpResult, w, h);

    uint8_t * cyImage = cip.getGrayImage();
    uint8_t * cbImage = cip.getCbImage();
    uint8_t * crImage = cip.getCrImage();
    uint8_t * coloredImage = cip.getColoredImage();
    imageTool::saveGrayscaleImage("gray.png", cyImage, w, h);
    imageTool::saveGrayscaleImage("Cb.png", cbImage, w, h);
    imageTool::saveGrayscaleImage("Cr.png", crImage, w, h);
    imageTool::saveColoredImage("coloredImage.png", coloredImage, w, h);
    slp.saveImage(imageYUV, "scanline.png");
    eip.saveImage("edgeImage.png");
    fbp.saveImage(imageYUV, "fieldBoundary.png");
    fsp.saveImage(imageYUV, "spotsProvider.png");
    lip.saveImage(imageYUV, "linesProvider.png");
    bp.saveImage(imageYUV, "ballProvider.png");
    ccp.saveImage(imageYUV, "centerCircle.png");

    cout <<"Finish Processing!" << endl;
    return 0;
}
