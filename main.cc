/*
  Copyright 2007, 2008 Computer Vision Lab,
  Ecole Polytechnique Federale de Lausanne (EPFL), Switzerland.
  All rights reserved.

  Authors: Vincent Lepetit (http://cvlab.epfl.ch/~lepetit)
           Mustafa Ozuysal (http://cvlab.epfl.ch/~oezuysal)
           Julien  Pilet   (http://cvlab.epfl.ch/~jpilet)

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
#include <cv.h>		//������һ��������opencvͷ�ļ� 
#include <highgui.h> //�߼�ͼ���û�����ӿ� ���磺���������ڣ���ʾ��ͼ�� ��Ҫ�õ����ģ��

#include <iostream>
#include <string>
using namespace std; //ͨ�����������ռ������ֲ�ͬ���������

#include "mcv.h"
#include "planar_pattern_detector_builder.h" //����ͷ�ļ�
#include "template_matching_based_tracker.h" //���ٵ�ͷ�ļ�

const int max_filename = 1000; //const���ε�����������ָ�����ͣ������͵ı���������ֵ�ǲ��ܱ����µ�

enum source_type {webcam_source, sequence_source, video_source};
//�ڡ�ö�١����͵Ķ������оٳ����п��ܵ�ȡֵ����˵��Ϊ�á�ö�١����͵ı���ȡֵ���ܳ�������ķ�Χ
planar_pattern_detector * detector; //*һ�㱻����ָ����������ֽз���ȡַ�����
//����detector��һ��ָ�������ָ��planar_pattern_detector������
template_matching_based_tracker * tracker;

int mode = 2;
bool show_tracked_locations = true;
//C++�� bool���ֵ�����ΪTrue,Ϊ�����False
bool show_keypoints = true;

CvFont font;

void draw_quadrangle(IplImage * frame,
		     int u0, int v0,
		     int u1, int v1,
		     int u2, int v2,
		     int u3, int v3,
		     CvScalar color, int thickness = 1)
{
  cvLine(frame, cvPoint(u0, v0), cvPoint(u1, v1), color, thickness);
  cvLine(frame, cvPoint(u1, v1), cvPoint(u2, v2), color, thickness);
  cvLine(frame, cvPoint(u2, v2), cvPoint(u3, v3), color, thickness);
  cvLine(frame, cvPoint(u3, v3), cvPoint(u0, v0), color, thickness);
}

void draw_detected_position(IplImage * frame, planar_pattern_detector * detector) //Ȧ�����λ��
{
  draw_quadrangle(frame,
		  detector->detected_u_corner[0], detector->detected_v_corner[0],
		  detector->detected_u_corner[1], detector->detected_v_corner[1],
		  detector->detected_u_corner[2], detector->detected_v_corner[2],
		  detector->detected_u_corner[3], detector->detected_v_corner[3],
		  cvScalar(255), 3);
}

void draw_initial_rectangle(IplImage * frame, template_matching_based_tracker * tracker) //��ʼ���߿򣺸��ٵõ�
{
  draw_quadrangle(frame,
		  tracker->u0[0], tracker->u0[1],
		  tracker->u0[2], tracker->u0[3],
		  tracker->u0[4], tracker->u0[5],
		  tracker->u0[6], tracker->u0[7],
		  cvScalar(128), 3);
}

void draw_tracked_position(IplImage * frame, template_matching_based_tracker * tracker)
{
  draw_quadrangle(frame,
		  tracker->u[0], tracker->u[1],
		  tracker->u[2], tracker->u[3],
		  tracker->u[4], tracker->u[5],
		  tracker->u[6], tracker->u[7],
		  cvScalar(255), 3);
}

void draw_tracked_locations(IplImage * frame, template_matching_based_tracker * tracker)                                                           //Ϊʲô����λ��ҲҪcvcircle��
{
  for(int i = 0; i < tracker->nx * tracker->ny; i++) { //������������������������nx��ʲô
//����trackerָ��template_matching_based_tracker�ĵ�ַ����tracker->ny��ʹ��template_matching_based_tracker�еĳ�Աny��������������������������
//*�������
    int x1, y1; //�������ͱ������綨��һ�����ͱ���i��:int i;�������Ϳ���Ϊi��ֵ�ˣ������������͵ģ���ΧҲ�����ơ���Ҫ��һ�������ֵ��������intǰ��long
    tracker->f.transform_point(tracker->m[2 * i], tracker->m[2 * i + 1], x1, y1);//ģ��ƥ��.CC
    cvCircle(frame, cvPoint(x1, y1), 3, cvScalar(255, 255, 255), 1);
  }
}

void draw_detected_keypoints(IplImage * frame, planar_pattern_detector * detector)//��⵽�ĵ�
{
  for(int i = 0; i < detector->number_of_detected_points; i++)
    cvCircle(frame,
	     cvPoint(detector->detected_points[i].fr_u(),
		     detector->detected_points[i].fr_v()),
	     16 * (1 << int(detector->detected_points[i].scale)),
		 //�������������������������һλ�൱�ڳ���2����Ч�ʱȳ˷��� &��cout�ȵ������������
	     cvScalar(100), 1);
}

void draw_recognized_keypoints(IplImage * frame, planar_pattern_detector * detector)                                                           //recognized�ؼ��㣿����׼�㣨���ȶ�����Щ������
{
  for(int i = 0; i < detector->number_of_model_points; i++)
    if (detector->model_points[i].class_score > 0)
      cvCircle(frame,
	       cvPoint(detector->model_points[i].potential_correspondent->fr_u(),
		       detector->model_points[i].potential_correspondent->fr_v()),
	       16 * (1 << int(detector->detected_points[i].scale)),
	       cvScalar(255, 255, 255), 1);
}

			
void detect_and_draw(IplImage * frame)
{
	static bool last_frame_ok=false;

	if (mode == 1 || ((mode==0) && last_frame_ok)) 
	{
		// ||�߼������������ʽ��������ֵ��||������ֵ��or
		bool ok = tracker->track(frame);
		last_frame_ok=ok;
//���mode=1(only tracking),����last_frame_ok=ok


		if (!ok) 
		{ //����ģʽһʱ
			if (mode==0) return detect_and_draw(frame);//mode=0(Detect when tracking fails or for initialization then track)������detect_and_draw
			else 
			{
				draw_initial_rectangle(frame, tracker);
				tracker->initialize();
			}//�Ȳ���ģʽһҲ����ģʽ�㣨�ȷǸ���Ҳ���ǳ�ʼ����,����ʼ�����ٷ���
		} 
		else 
		{//mode=1
			draw_tracked_position(frame, tracker);//��������λ��
			if (show_tracked_locations) draw_tracked_locations(frame, tracker);//�������λ���ܹ�չ�֣�Ϊ�棩���������λ��
		}
		cvPutText(frame, "template-based 3D tracking", cvPoint(10, 30), &font, cvScalar(255, 255, 255));//��mode=1ʱ����ע��"template-based 3D tracking"
	} else {//�������������������ʱ��mode=1��mode=0��mode��Ϊһ���㣩
		detector->detect(frame);//��������м��
	if (detector->pattern_is_detected) {
			last_frame_ok=true;

			tracker->initialize(detector->detected_u_corner[0], detector->detected_v_corner[0],
					detector->detected_u_corner[1], detector->detected_v_corner[1],
					detector->detected_u_corner[2], detector->detected_v_corner[2],
					detector->detected_u_corner[3], detector->detected_v_corner[3]);
					

			if (mode == 3 && tracker->track(frame)) //��mode=3���ܹ����ٵ�
			{

				if (show_keypoints) 
				{
					draw_detected_keypoints(frame, detector);
					draw_recognized_keypoints(frame, detector);
				}
				draw_tracked_position(frame, tracker);
				if (show_tracked_locations) draw_tracked_locations(frame, tracker);

				cvPutText(frame, "detection+template-based 3D tracking", cvPoint(10, 30), &font, cvScalar(255, 255, 255));
			} 
			else 
			{
				if (show_keypoints) 
				{
					draw_detected_keypoints(frame, detector);
					draw_recognized_keypoints(frame, detector);
				}
				draw_detected_position(frame, detector);
				cvPutText(frame, "detection", cvPoint(10, 30), &font, cvScalar(255, 255, 255));
			}
		} 
           else 
	     {
			last_frame_ok=false;
			if (show_keypoints) draw_detected_keypoints(frame, detector);

			if (mode == 3)
				cvPutText(frame, "detection + template-based 3D tracking", cvPoint(10, 30), &font, cvScalar(255, 255, 255));//��255��255��255�������ɫ����
	/*void cvPutText( CvArr* img, const char* text, CvPoint org, const CvFont* font,CvScalar color );
����img:����ͼ��
����text:Ҫ��ʾ���ַ���
����org:��һ����ĸ���½ǵ�����
����font:ָ������ṹ��ָ��
����color:�ı�����ɫ*/
			else
				cvPutText(frame, "detection", cvPoint(10, 30), &font, cvScalar(255, 255, 255));
		 }
	}

	cvShowImage("ferns-demo", frame);
}

void help(const string& exec_name) {
  cout << exec_name << " [-m <model image>] [-s <image sequence format>]\n\n";
  cout << "   -m : specify the name of the model image depicting the planar \n";
  cout << "        object from a frontal viewpoint. Default model.bmp\n";
  cout << "   -s : image sequence format in printf style, e.g. image%04.jpg,\n";
  cout << "        to test detection. If not specified webcam is used as \n";
  cout << "        image source.\n";
  cout << "   -v : video filename to test detection. If not specified webcam\n";
  cout << "        is used as image source.\n";
  cout << "   -h : This help message." << endl;
}

int main(int argc, char ** argv)
/*argc���������еĲ����ĸ���,argv[]��Ӧÿһ������
����
./a.exe 1 bb
argc������3
����
argv[0]�ǡ�./a.exe��
argv[1]�ǡ�1��
argv[2]�ǡ�bb��*/
{
  string model_image     = "model.bmp";
  string sequence_format = "";
  string video_file = "";
  source_type frame_source = webcam_source;

  for(int i = 0; i < argc; ++i) 
  {
    if(strcmp(argv[i], "-h") == 0) 
/*strcmp:�Ƚ������ַ���,��ͷ�ļ�string.h��
extern int strcmp(const char *s1,const char *s2);
��s1<s2ʱ������Ϊ������
��s1=s2ʱ������ֵ= 0��
��s1>s2ʱ������������
���������ַ���������������ַ���ȣ���ASCIIֵ��С��Ƚϣ���ֱ�����ֲ�ͬ���ַ�����'\0'Ϊֹ���磺
"A"<"B" "a">"A" "computer">"compare"*/
    {
      help(argv[0]);
      return 0;
    }
	/*һ������£�C++�������ĺ�����Ҫ�󷵻�һ��ֵ��������ִ���������Ҵﵽ��һ������µ�Ŀ�ģ���ô�ͷ���0��ʾ��ȷ�ĵ����˸ú�����
	���0���Ƿ��ظ�����������֪ͨû�г���ģ�
	������������г�������û�а���һ�����ִ�У���ô�ͷ���1���Ը�֪����������ȡ��Ӧ����*/

    if(strcmp(argv[i], "-m") == 0) 
	{
      if(i == argc - 1) 
	  {
        cerr << "Missing model name after -m\n";
		/*cerr����������׼��������������ʾ������Ϣ
		Ĭ������±���������׼������������������壬Ҳ��˵������Ϣ����ֱ�ӷ��͵���ʾ����������ȵ������������µĻ��з�ʱ���ű���ʾ��һ������²����ض���*/
        help(argv[0]);                                                                                                                                 //��argv[0]��ʼ��
        return -1;
      }
      ++i;//i++  �������ú�����   ++i  �������Ӻ�����
      model_image = argv[i];
    }
    else if(strcmp(argv[i], "-s") == 0) 
	{
      if(i == argc - 1) 
	  {
        cerr << "Missing sequence format after -s\n";
        help(argv[0]);
        return -1;
      }
      ++i;
      sequence_format = argv[i];
      frame_source = sequence_source;
    }
    else if(strcmp(argv[i], "-v") == 0) 
	{
      if(i == argc - 1) 
	  {
        cerr << "Missing  video filename after -v\n";
        help(argv[0]);
        return -1;
      }
      ++i;
      video_file = argv[i];
      frame_source = video_source;
    }
  }

  affine_transformation_range range;

  detector = planar_pattern_detector_builder::build_with_cache(model_image.c_str(),
							       &range,
							       400,
							       5000,
							       0.0,
							       32, 7, 4,
							       30, 12,
							       10000, 200);
/*::
  ��1����ʾ�����������

��������������һ����A����A��������һ����Ա����void f()����û����������������f�Ķ��壬��ô�����ⶨ��fʱ��

������Ҫд��void A::f()����ʾ���f()��������A�ĳ�Ա������

����2��ֱ������ȫ�ֺ���ǰ����ʾ��ȫ�ֺ���

����������VC�������ڵ���API �������API������ǰ�ӣ���

����3����ʾ���ó�Ա�������������������Ա�����

��������System::Math::Sqrt() �൱��System.Math.Sqrt()*/
  if (!detector) 
  {
    cerr << "Unable to build detector.\n";
    return -1;
  }

  detector->set_maximum_number_of_points_to_detect(1000);

  tracker = new template_matching_based_tracker();
  string trackerfn = model_image + string(".tracker_data");
  if (!tracker->load(trackerfn.c_str())) 
  {
    cout << "Training template matching..."<<endl;
    tracker->learn(detector->model_image,
		   5, // number of used matrices (coarse-to-fine)
		   40, // max motion in pixel used to train to coarser matrix
		   20, 20, // defines a grid. Each cell will have one tracked point.
		   detector->u_corner[0], detector->v_corner[1],
		   detector->u_corner[2], detector->v_corner[2],
		   40, 40, // neighbordhood for local maxima selection
		   10000 // number of training samples
		   );
    tracker->save(trackerfn.c_str());
  }
  tracker->initialize();

  cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX,
	     1.0, 1.0, 0.0,
	     3, 8);

  CvCapture * capture = 0;
  IplImage * frame, * gray_frame = 0;
  int frame_id = 1;
  char seq_buffer[max_filename];

  cvNamedWindow("ferns-demo", 1 );

  if(frame_source == webcam_source) 
  {
    capture = cvCaptureFromCAM(0);
  }
  else if(frame_source == video_source) 
  {
    capture = cvCreateFileCapture(video_file.c_str());
  }


  int64 timer = cvGetTickCount();

  bool stop = false;
  do 
{
    if(frame_source == webcam_source || frame_source == video_source) 
	{
      if (cvGrabFrame(capture) == 0) break;
      frame = cvRetrieveFrame(capture);
    }
    else 
	{
      snprintf(seq_buffer, max_filename, sequence_format.c_str(), frame_id);
	  /*���ɱ������(...)����format��ʽ�����ַ�����Ȼ���临�Ƶ�str��
(1) �����ʽ������ַ������� < size���򽫴��ַ���ȫ�����Ƶ�str�У�����������һ���ַ���������('\0')��
(2) �����ʽ������ַ������� >= size����ֻ�����е�(size-1)���ַ����Ƶ�str�У�����������һ���ַ���������('\0')������ֵΪ��д����ַ������ȡ�*/
      frame = cvLoadImage(seq_buffer, 1);
      ++frame_id;
    }

    if (frame == 0) break;

    if (gray_frame == 0)
      gray_frame = cvCreateImage(cvSize(frame->width,frame->height), IPL_DEPTH_8U, 1);

    cvCvtColor(frame, gray_frame, CV_RGB2GRAY);

    if (frame->origin != IPL_ORIGIN_TL)
      cvFlip(gray_frame, gray_frame, 0);

    detect_and_draw(gray_frame);

    int64 now = cvGetTickCount();
    double fps = 1e6 * cvGetTickFrequency()/double(now-timer);
    timer = now;
    clog << "Detection frame rate: " << fps << " fps         \r";

    int key = cvWaitKey(10);
    if (key >= 0) 
	{
      switch(char(key)) 
	  {
      case '0': mode = 0; break;
      case '1': mode = 1; break;
      case '2': mode = 2; break;
      case '3': mode = 3; break;
      case '4': show_tracked_locations = !show_tracked_locations; break;
      case '5': show_keypoints = !show_keypoints; break;
      case 'q': stop = true; break;
      default: ;
      }
      cout << "mode=" << mode << endl;
    }

    if(frame_source == sequence_source) 
	{
      cvReleaseImage(&frame);
    }
 } 
while(!stop);

  clog << endl;����������������������������������������������������
  delete detector;
  delete tracker;

  cvReleaseImage(&gray_frame);
  cvReleaseCapture(&capture);
  cvDestroyWindow("ferns-demo");

  return 0;
}
