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
#include <algorithm>
using namespace std;

#include "mcv.h"
#include "planar_pattern_detector_builder.h"

planar_pattern_detector * planar_pattern_detector_builder::build_with_cache(const char * image_name,
									    affine_transformation_range * range,
									    int maximum_number_of_points_on_model,
									    int number_of_generated_images_to_find_stable_points,
									    double minimum_number_of_views_rate,
									    int patch_size, int yape_radius, int number_of_octaves,
									    int number_of_ferns, int number_of_tests_per_fern,
									    int number_of_samples_for_refinement, int number_of_samples_for_test,
									    char * given_detector_data_filename,//？？？？？？？？？？？
									    int roi_up_left_u, int roi_up_left_v,//？？？？？？？？？？？
									    int roi_bottom_right_u, int roi_bottom_right_v)
{
  planar_pattern_detector * detector = new planar_pattern_detector();

  detector->image_generator->set_transformation_range(range);

  char detector_data_filename[1000];
  if (given_detector_data_filename == 0)//没有给定检测文件时，将image_name格式化，并写入 "%s.detector_data"中，存放在detector_data_filename缓冲区
    sprintf(detector_data_filename, "%s.detector_data", image_name);
  /*
  字符串格式化命令，主要功能是把格式化的数据写入某个字符串中
  函数原型
  int sprintf( char *buffer, const char *format, [argument] … );
  参数列表
  buffer：char型指针，指向将要写入的字符串的缓冲区。
  format：格式化字符串。
  [argument]...：可选参数，可以是任何类型的数据。
 
  %c格式对应的是单个字符，%s格式对应的是字符串。
  //例：char  a;
  //char b[20];
  //scanf("%c",&a);只能输入一个字符。
  //scanf("%s",b);可以输入一串不超过20字符的字符串。
  */
  else
    strcpy(detector_data_filename, given_detector_data_filename);
/*
  把从src地址开始且含有'\0'结束符的字符串复制到以dest开始的地址空间
  原型声明：char *strcpy(char* dest, const char *src);
  src和dest所指内存区域不可以重叠且dest必须有足够的空间来容纳src的字符串。返回指向dest的指针。
  */
  if (detector->load(detector_data_filename))
    cout << "> [planar_pattern_detector_builder] " << detector_data_filename << " file read." << endl;
  //cout<<x<<endl<<x;首先输出x变量的值，然后输出一个换行符，最后在输出一次x变量的值（只有基本数据类型以及字符串数组可以直接用cout输出）
  else 
  {
    delete detector;

    cout << "> [planar_pattern_detector_builder] Can't find file " << detector_data_filename << "." << endl;
    cout << "> [planar_pattern_detector_builder] Creating one..." << endl;

    detector = learn(image_name,
		     range,
		     maximum_number_of_points_on_model,
		     number_of_generated_images_to_find_stable_points,
		     minimum_number_of_views_rate,
		     patch_size, yape_radius, number_of_octaves,
		     number_of_ferns, number_of_tests_per_fern,
		     number_of_samples_for_refinement, number_of_samples_for_test,
		     roi_up_left_u, roi_up_left_v,
		     roi_bottom_right_u, roi_bottom_right_v);

    if (detector != 0) detector->save(detector_data_filename);
  }

  return detector;
}

planar_pattern_detector * planar_pattern_detector_builder::force_build(const char * image_name,
								       affine_transformation_range * range,
								       int maximum_number_of_points_on_model,
								       int number_of_generated_images_to_find_stable_points,
								       double minimum_number_of_views_rate,
								       int patch_size, int yape_radius, int number_of_octaves,
								       int number_of_ferns, int number_of_tests_per_fern,
								       int number_of_samples_for_refinement, int number_of_samples_for_test,
								       char * given_detector_data_filename,
								       int roi_up_left_u, int roi_up_left_v,
								       int roi_bottom_right_u, int roi_bottom_right_v)
{
  char detector_data_filename[1000];
  if (given_detector_data_filename == 0)
    sprintf(detector_data_filename, "%s.detector_data", image_name);
  else
    strcpy(detector_data_filename, given_detector_data_filename);
  /*
   函数原型 char *strcpy(char *dest,const char *src)

  头文件：#include<string.h>

  功能是：从src地址开始且含有null结束符的字符串复制到以dest地址开始的字符串中，并返回指向dest的指针。
  通俗的讲就是将 src字符数组复制到dest数组中，如果dest数组本身有数据，会把src里的数据全部复制到dest中，如果dest中有数据小于src地址长度的将会被覆盖，而大于src长度的将保留

  说明：dest的地址长度要足够大，不然会产生溢出。Dest的内存长度要大于等于src的内存长度。
*/

  cout << "> [planar_pattern_detector_builder] Creating file " << detector_data_filename << ":" << endl;

  planar_pattern_detector * detector = learn(image_name,
					     range,
					     maximum_number_of_points_on_model,
					     number_of_generated_images_to_find_stable_points,
					     minimum_number_of_views_rate,
					     patch_size, yape_radius, number_of_octaves,
					     number_of_ferns, number_of_tests_per_fern,
					     number_of_samples_for_refinement, number_of_samples_for_test,
					     roi_up_left_u, roi_up_left_v,
					     roi_bottom_right_u, roi_bottom_right_v);

  if (detector != 0) detector->save(detector_data_filename);

  return detector;
}

planar_pattern_detector * planar_pattern_detector_builder::just_load(const char * given_detector_data_filename)//给定的检测数据文件名
{
  planar_pattern_detector * detector = new planar_pattern_detector();

  const char * detector_data_filename = given_detector_data_filename;

  if (detector->load(detector_data_filename)) 
  {
    cout << "> [planar_pattern_detector_builder] " << detector_data_filename << " file read." << endl;
    return detector;
  } else 
  {
    cerr << ">! [planar_pattern_detector_builder] Couldn't find file " << detector_data_filename << "." << endl;
    return 0;
  }
}

planar_pattern_detector * planar_pattern_detector_builder::learn(const char * image_name,
								 affine_transformation_range * range,
								 int maximum_number_of_points_on_model,
								 int number_of_generated_images_to_find_stable_points,
								 double minimum_number_of_views_rate,
								 int patch_size, int yape_radius, int number_of_octaves,
								 int number_of_ferns, int number_of_tests_per_fern,
								 int number_of_samples_for_refinement, int number_of_samples_for_test,
								 int roi_up_left_u, int roi_up_left_v,
								 int roi_bottom_right_u, int roi_bottom_right_v)
{
  planar_pattern_detector * detector = new planar_pattern_detector();

  strcpy(detector->image_name, image_name);//->指向结构体或者类的指针用来引用对象里面的变量

  detector->model_image = mcvLoadImage(image_name, 0);
  if (detector->model_image == 0) return 0;
  if (detector->model_image->nChannels != 1) 
  {
    cerr << ">! [planar_pattern_detector_builder::learn] Wrong image format" << endl;
    return 0;
  }
  
  //寻找感兴趣区域,如果没有感兴趣区域，则将整个图像作为目标//本程序中roi_up_left_u == -1，并且作者给出了roifilename
  if (roi_up_left_u == -1) //左上角位置,-1表示判断，非零，与数字没有关系//==:是比较,是与==后的数进行比较,相等为真,如果不相等为假。
  {
    char roi_filename[1000];
    sprintf(roi_filename, "%s.roi", image_name);
    ifstream roif(roi_filename);
    if (roif.good()) //判断是否文件读完,这是文件流的一个函数
	{
      cout << "> [planar_pattern_detector_builder::learn] Reading ROI from file " << roi_filename << ".\n";
      for(int i = 0; i < 4; i++)
	roif >> detector->u_corner[i] >> detector->v_corner[i];
      roif.close();
    } 
	else 
	{
      cout << "> [planar_pattern_detector_builder::learn] No ROI file found. Taking the whole image as object." << endl;

      detector->u_corner[0] = 0;                                detector->v_corner[0] = 0;
      detector->u_corner[1] = detector->model_image->width - 1; detector->v_corner[1] = 0;
      detector->u_corner[2] = detector->model_image->width - 1; detector->v_corner[2] = detector->model_image->height - 1;
      detector->u_corner[3] = 0;                                detector->v_corner[3] = detector->model_image->height - 1;
    }
  } 
  else //指的是程序中给定感兴趣位置的四个点的坐标//此工程没有在程序中直接给出
  {
    detector->u_corner[0] = roi_up_left_u;      detector->v_corner[0] = roi_up_left_v;
    detector->u_corner[1] = roi_bottom_right_u; detector->v_corner[1] = roi_up_left_v;
    detector->u_corner[2] = roi_bottom_right_u; detector->v_corner[2] = roi_bottom_right_v;
    detector->u_corner[3] = roi_up_left_u;      detector->v_corner[3] = roi_bottom_right_v;//右下角位置
  }

  detector->patch_size = patch_size;
  detector->yape_radius = yape_radius;
  detector->number_of_octaves = number_of_octaves;

  detector->image_generator->set_original_image(detector->model_image);//set_original_image（）这个函数不懂
  detector->image_generator->set_mask(detector->u_corner[0], detector->v_corner[0],//？？
				      detector->u_corner[2], detector->v_corner[2]);
  detector->image_generator->set_transformation_range(range);


  detector->pyramid = new fine_gaussian_pyramid(yape_radius, patch_size, number_of_octaves);//高斯金字塔分为组octaves，组里面分为层level。

  detect_most_stable_model_points(detector,
				  maximum_number_of_points_on_model,
                                  yape_radius, number_of_octaves,
                                  number_of_generated_images_to_find_stable_points,
                                  minimum_number_of_views_rate);


  detector->save_image_of_model_points("model_points.bmp", patch_size);

  cout << "> [planar_pattern_detector_builder] Creating classifier: " << flush;
  detector->classifier = new fern_based_point_classifier(maximum_number_of_points_on_model,
							 number_of_ferns, number_of_tests_per_fern,
							 -patch_size / 2, patch_size / 2, -patch_size / 2, patch_size / 2, 0, 0);
  cout << endl;
  cout << "> [planar_pattern_detector_builder] Ok." << endl;

  cout << "> [planar_pattern_detector_builder] Training: " << endl;
  detector->classifier->reset_leaves_distributions();//重置叶子分布
  cout << "   - leaves distributions reset ok. " << flush;
  detector->classifier->train(detector->model_points, detector->number_of_model_points,
			      number_of_octaves, yape_radius, number_of_samples_for_refinement,
			      detector->image_generator);

  cout << "   - training... " << number_of_samples_for_refinement << " images generated." << endl;
  detector->classifier->finalize_training();
  cout << "   - posterior probabilities computed." << endl;//后验概率计算

  cout << "   - testing:" << endl;
  //平均识别率如何计算？
  detector->mean_recognition_rate = detector->classifier->test(detector->model_points, detector->number_of_model_points,
							       number_of_octaves, yape_radius, number_of_samples_for_test,
							       detector->image_generator);

  return detector;
}

//函数cmp_tmp_model_points需要返回一个bool类型的值，其中的p1，p2代表该函数的参数
//static：静态成员函数
static bool cmp_tmp_model_points(pair<keypoint, int> p1, pair<keypoint, int> p2)//？？？
	/*pair是一种模板类型，其中包含两个数据值，两个数据的类型可以不同
	pair<int, string> a;
    表示a中有两个类型，第一个元素是int型的，第二个元素是string类型的，如果创建pair的时候没有对其进行初始化，则调用默认构造函数对其初始化。*/
{
  return p1.second > p2.second;
}

void planar_pattern_detector_builder::detect_most_stable_model_points(planar_pattern_detector * detector,
								      int maximum_number_of_points_on_model,
								      int /*yape_radius*/, int /*number_of_octaves*/,
								      int number_of_generated_images,
								      double minimum_number_of_views_rate)
{
  cout << "> [planar_pattern_detector_builder] Determining most stable points:" << endl;// 判决,,,

  const int K = 2;
  vector< pair<keypoint, int> > tmp_model_point_vector;//???

  fine_gaussian_pyramid    * pyramid         = detector->pyramid;
  affine_image_generator06 * image_generator = detector->image_generator;
  pyr_yape06               * point_detector  = detector->point_detector;

  image_generator->disable_random_background();
  image_generator->generate_Id_image();
  pyramid->set_image(image_generator->generated_image);
  if (point_detector == 0) detector->point_detector = new pyr_yape06(); point_detector = detector->point_detector;
  keypoint * tmp_model_point_array = new keypoint[K * maximum_number_of_points_on_model];

  for(int i = 0; i < number_of_generated_images; i++) 
  {
    cout << "   (Generating views: " << number_of_generated_images - i << ")     \r" << flush;
    
    if (i == 0) image_generator->generate_Id_image();
    else        image_generator->generate_random_affine_image();
    pyramid->set_image(image_generator->generated_image);

    int current_detected_point_number = detector->point_detector->detect(pyramid, tmp_model_point_array,//得到当前检测点的数量
									 K * maximum_number_of_points_on_model);

    for(int j = 0; j < current_detected_point_number; j++) 
	{
      keypoint * k = tmp_model_point_array + j;
      float nu, nv;

      image_generator->inverse_affine_transformation(k->fr_u(), k->fr_v(), nu, nv);//对当前检测到的点进行逆仿射变换得到nu，nv？？？fr_u()???
      nu = fine_gaussian_pyramid::convCoordf(nu, 0, int(k->scale));
      nv = fine_gaussian_pyramid::convCoordf(nv, 0, int(k->scale));

      keypoint kd(nu, nv, k->scale);//将nu，nv与检测点比较？？
      if (kd.fr_u() >= detector->u_corner[0] && kd.fr_u() <= detector->u_corner[1] &&
	  kd.fr_v() >= detector->v_corner[0] && kd.fr_v() <= detector->v_corner[3])	
	  {
	pair<keypoint, int> * mp = search_for_existing_model_point(&tmp_model_point_vector, nu, nv, int(k->scale));//满足上述条件是寻找模型上对应的点

	if (mp != 0) {
	  // Move the keypoint coordinates in the center of gravity of all agglomerated keypoints:移动（在所有团簇点的重心位置）的关键点坐标
	  float n = float(mp->second);
	  mp->first.u = (mp->first.u * (n - 1) + nu) / n;
	  mp->first.v = (mp->first.v * (n - 1) + nv) / n;
	  mp->second++;
	} 
	else 
	{
	  keypoint op(nu, nv, k->scale);
	  tmp_model_point_vector.push_back(pair<keypoint, int>(op, 1));
	}
      }
    }
  }
  cout << "> [planar_pattern_detector_builder] " << number_of_generated_images << " images generated; " << int( tmp_model_point_vector.size() ) << " points detected." << endl;

  sort(tmp_model_point_vector.begin(), tmp_model_point_vector.end(), cmp_tmp_model_points);

  int min_views = int(minimum_number_of_views_rate * double(number_of_generated_images));
  int it_index;
  detector->model_points = new keypoint[maximum_number_of_points_on_model];
  vector< pair<keypoint, int> >::iterator it;
  for(it = tmp_model_point_vector.begin(), it_index = 0;
      (it < tmp_model_point_vector.end()) && (it_index < maximum_number_of_points_on_model) && (it->second >= min_views);
      it++, it_index++) {

    detector->model_points[it_index].u = it->first.u;
    detector->model_points[it_index].v = it->first.v;
    detector->model_points[it_index].scale = it->first.scale;
    detector->model_points[it_index].class_index = it_index;

    if (it_index == 0) cout << "> [planar_pattern_detector_builder] Point " << it_index << " detected " << it->second << " times.." << flush;
  }
  if (it_index > 0)
    cout << "..point " << it_index - 1 << " detected " << (it-1)->second << " times ("
         << number_of_generated_images
         << " generated views, "
         << minimum_number_of_views_rate * 100 << "% minimum rate)." << endl;
  else
    cout << ">! [planar_pattern_detector_builder] No model points detected ?!?! gloups..." << endl;

  detector->number_of_model_points = it_index;

  delete [] tmp_model_point_array;
}

pair<keypoint, int> * planar_pattern_detector_builder::
search_for_existing_model_point(vector< pair<keypoint, int> > * tmp_model_points,
                                float cu, float cv, int scale)
{
  const float keypoint_distance_threshold = 2.0;

  for(vector< pair<keypoint, int> >::iterator it = tmp_model_points->begin();
      it < tmp_model_points->end();
      it++)
    if (int(it->first.scale) == scale) {
      float dist2 = (it->first.u - cu) * (it->first.u - cu) + (it->first.v - cv) * (it->first.v - cv);

      if (dist2 < keypoint_distance_threshold * keypoint_distance_threshold)
        return &(*it);
    }

  return 0;
}
