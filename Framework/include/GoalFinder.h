// Soccer Vision 
// Author: Andreas Ekadinata Widodo <ekadinataa@gmail.com>

/**
* KRSBI ITB
* Dago Hoogeschool Team
**/



#ifndef GoalFinder_H
#define GoalFinder_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include <cmath>

#include "Point.h"
#include "minIni.h"
#include "Camera.h"


#define COLOR_SECTION   "Find Goal"
#define INVALID_VALUE   -1024.0
#define PI 3.14159265


#define max_val 255
#define max_lowThreshold 500

using namespace cv;

namespace Robot {
	enum GoalStatus
	{
		C2P=1,  // Crossbar with 2 Posts
		NC2P,   // No Crossbar with 2 Posts
		C1P,    // Crossbar with 1 Post
		NC1P,   // No Crossbar with 1 Post
		CO,     // Crossbar Only
		NL      // No Line Detected
	};
	class GoalFinder
	{
		private:
			static GoalFinder* m_UniqueInstance;
			bool flag;

			Mat img;
			Mat hsv;
			Mat th;
			Mat edge;

			int variansi;

			static void on_trackbar(int value, void *userdata);
			void CreateTrackbar();
			void morphOps(Mat &thresh);
			

		public:

			enum
			{
				NONE,
				UNKNOWN_POST,
				RIGHT_POST,
				LEFT_POST,
				POSSIBLE_RIGHT_POST,
				POSSIBLE_LEFT_POST,
				BOTH_POST
			};
			
			enum 
			{
				UNCLEAR, //belum jelas gawang siapa
				OWN_GOAL, //gawang sendiri
				OPPONENT_GOAL //gawang lawan
			};

			std::string color_section;
			GoalStatus goalstate;

			int min_H;
			int max_H;
			int min_S;
			int max_S;
			int min_V;
			int max_V;

			int lowThreshold;
			int ratio;
			int kernel_size;

			int thLine;
			int minLinLength;
			int maxLineGap;

			int r_mean_h ;
		    int r_mean_v ;
		    int r_mean_right ;
		    int r_mean_left ;
		    
		    float teta_mean_h ;
		    float teta_mean_v ;
		    float teta_mean_left;
		    float teta_mean_right;

		    int variansi_v;


			GoalFinder();
			GoalFinder(int min_H, int max_H, int min_S, int max_S, int min_V, int max_V, int lowThreshold, int ratio, int kernel_size, int thLine, int minLinLength, int maxLineGap);
			virtual ~GoalFinder();

			static GoalFinder* GetInstance() { return m_UniqueInstance; }

			vector<Vec2f> lines;
			vector<Vec2f> horizontal;
			vector<Vec2f> vertical;
			vector<Vec2f> right_post;
			vector<Vec2f> left_post;

			float rho;
			float theta;
        	Point pt1, pt2;

			//Corner
			Point2D TR;
			Point2D TL;
			Point2D BR;
			Point2D BL;
			
			void printParam();
			void Process(Mat image, bool OppGoal);
			void Filtering(Mat image);
			void ControlPanel(minIni* ini);
			void getCorner(Mat src);
			
			
			//Belum ditest
			void ClassifyLineHV();
			void CalculateMeanH();
			void CalculateMeanV();
			void CalculateVarianceV();
			void ClassifyLineRL();
			void CalculateMeanRL();
			bool ValidateH();
  			bool ValidateV();
  			void StateCheck();
  			Point CalculateIntersection(int R1, int Teta1, int R2, int Teta2);
			void DrawLine(Mat image, float r_draw, float t_draw);
			void Reset();
			Point RunDown(Mat thresh, Point start, int teta, int step);
			Point RunRight(Mat thresh, Point start, int teta, int step);
			Point RunLeft(Mat thresh, Point start, int teta, int step);
			
			/////////////////////////

			float DegreesToRadians(float degrees);
			float RadiansToDegrees(float radians);

			void LoadINISettings(minIni* ini);
        	void LoadINISettings(minIni* ini, const std::string &section);
        	void SaveINISettings(minIni* ini);
        	void SaveINISettings(minIni* ini, const std::string &section);


	};
}
#endif
