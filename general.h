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
#ifndef GENERAL_H
#define GENERAL_H

#include <stdlib.h>

float rand_01(void);
float rand_m1p1(void);
float rand(float min, float max);

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279
#endif

int gf_sqr(const int x);
//float gf_sqr(const float x);

//double dist2(double u1, double v1, double u2, double v2);

// inline functions:

inline float rand_01(void)
{
  return (rand() % RAND_MAX) / (float)RAND_MAX;//先取余再做除法
}

inline float rand_m1p1(void)
{
  return 2.f * rand_01() - 1.f;
}

inline float rand(float min, float max)
{
  return min + rand_01() * (max - min);
}

inline int gf_sqr(const int x)
{
  return x * x;
}

inline float gf_sqr(const float x)
{
  return x * x;
}

//inline double dist2(double u1, double v1, double u2, double v2)
//{
//  return (u1 - u2) * (u1 - u2) + (v1 - v2) * (v1 - v2);
//}

#endif // GENERAL_H
