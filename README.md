# TJArk-Vision

###This is the official code release of Team TJArk in RoboCup SPL.

Licensed under the GNU General Public License v3.0

@copyright TJArk, Tongji University, China

This is a Vision Library for RoboCup SPL games.

It depends on no 3rd party libs.

It realizes a real-time detection for NAO robots. Typically, it will cost 13-16ms per frame.

We provide a demo program, which will generate a couple of images with debug vision data.

For more information (vision algorithm, other parts of TJArk project like motion, behavior, actions, etc.), please see Team Research Paper.

**CAUTION: Due to the lack of Camera Pose (i.e. Camera Matrix), the perception performance has been greatly reduced.**

* It is currently in QT, you can simply open "TJArkVison.pro" in QT and build it.*
* To run the program, type this in command line: ./TJarkVison path/to/image.png*

**There will soon be a CMake version in this repository.**

**对于中国RoboCup SPL参赛队，如使用TJArk代码，请联系我们进行声明：tjark.official@gmail.com**
**CNN网络及原始权重文件并不是本代码的一部分，代码中仅给出一种可用的CNN分类器。**
**TJArk不对代码的使用及原理进行解释、维护以及技术帮助。**

Team TJArk

Tongji University, China

Jan. 15, 2019