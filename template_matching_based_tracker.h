/*
  Copyright 2007 Computer Vision Lab,
  Ecole Polytechnique Federale de Lausanne (EPFL), Switzerland.
  All rights reserved.

  Author: Vincent Lepetit (http://cvlab.epfl.ch/~lepetit)

  This file is part of the ferns_demo software.

  ferns_demo is free software; you can redistribute it and/or modify it under the
  terms of the GNU General Public License as published by the Free Software
  Foundation; either version 2 of the License, or (at your option) any later
  version.

  ferns_demo is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
  PARTICULAR PURPOSE. See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along with
  ferns_demo; if not, write to the Free Software Foundation, Inc., 51 Franklin
  Street, Fifth Floor, Boston, MA 02110-1301, USA
*/
#ifndef template_matching_based_tracker_h
#define template_matching_based_tracker_h
/*头件的中的#ifndef，这是一个很关键的东西。比如你有两个C文件，这两个C文件都include了同一个头文件。而编译时，这两个C文件要一同编译成一个可运行文件，于是问题来了，大量的声明冲突。 

还是把头文件的内容都放在#ifndef和#endif中吧。不管你的头文件会不会被多个文件引用，你都要加上这个。一般格式是这样的： 

#ifndef <标识> 
#define <标识> 

...... 
...... 

#endif 

<标识>在理论上来说可以是自由命名的，但每个头文件的这个“标识”都应该是唯一的。标识的命名规则一般是头文件名全大写，前后加下划线，并把文件名中的“.”也变成下划线，如：stdio.h 

#ifndef _STDIO_H_ 
#define _STDIO_H_ 

...... 

#endif*/
#include <cv.h>
#include "homography06.h"//单应性
#include "homography_estimator.h"

class template_matching_based_tracker
{
 public:
  template_matching_based_tracker(void);//此处void表示此函数没有参数

  bool load(const char * filename);
  //加载字符串指针filename
  //LOAD事件是加载事件，当你窗体加载成功后就会触发的事件，比如说你想初始化某些属性就可以在LOAD事件内完成。
  void save(const char * filename);

  void learn(IplImage * image,
	     int number_of_levels, int max_motion, int nx, int ny,
	     int xUL, int yUL,
	     int xBR, int yBR,
	     int bx, int by,
	     int Ns);

  void initialize(void);//此句表示该函数没有返回值且没有参数，那么为什么int u0...?????????
  void initialize(int u0, int v0,
		  int u1, int v1,
		  int u2, int v2,
		  int u3, int v3);

  bool track(IplImage * input_frame);

  homography06 f;

  //private:
  void find_2d_points(IplImage * image, int bx, int by);//寻找2d点
  void compute_As_matrices(IplImage * image, int max_motion, int Ns);//计算矩阵
  void move(int x, int y, float & x2, float & y2, int amp);//移动
  bool normalize(CvMat * V);//标准化
  void add_noise(CvMat * V);//加噪声
  IplImage * compute_gradient(IplImage * image);
  void get_local_maximum(IplImage * G,
			 int xc, int yc, int w, int h,
			 int & xm, int & ym);

  homography_estimator he;

  int * m;
  CvMat ** As;
  CvMat * U0, * U, * I0, * DU, * DI, * I1;
  float * u0, * u, * i0, * du, * i1;
  int number_of_levels, nx, ny;
};


#endif
