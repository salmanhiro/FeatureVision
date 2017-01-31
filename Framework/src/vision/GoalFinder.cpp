// Soccer Vision 
// Author: Andreas Ekadinata Widodo <ekadinataa@gmail.com>

/**
* KRSBI ITB
* Dago Hoogeschool Team
**/


#include "GoalFinder.h"


using namespace std;
using namespace Robot;


GoalFinder* GoalFinder::m_UniqueInstance = new GoalFinder();

GoalFinder::GoalFinder() : 
        min_H(0),
        max_H(255),
        min_S(0),
        max_S(255),
        min_V(0),
        max_V(255),
        lowThreshold(150),
        ratio(1),
        kernel_size(3),
        thLine(250),
        minLinLength(60),
        maxLineGap(20),
        color_section("")
{ }

GoalFinder::GoalFinder(int min_H, int max_H, int min_S, int max_S, int min_V, int max_V, int lowThreshold, int ratio, int kernel_size, int thLine, int minLinLength, int maxLineGap) : 
        min_H(min_H),
        max_H(max_H),
        min_S(min_S),
        max_S(max_S),
        min_V(min_V),
        max_V(max_V),
        lowThreshold(lowThreshold),
        ratio(ratio),
        kernel_size(kernel_size),
        thLine(thLine),
        minLinLength(minLinLength),
        maxLineGap(maxLineGap),
        color_section("")
{ }
/////////////////////////////////////

GoalFinder::~GoalFinder() { /* NONE */ }

void GoalFinder::on_trackbar(int value, void *userdata){}

void GoalFinder::CreateTrackbar(){
	namedWindow("Kalibrasi");
	createTrackbar( "min_H", "Kalibrasi", &min_H, max_val, on_trackbar );
	createTrackbar( "max_H", "Kalibrasi", &max_H, max_val, on_trackbar );
	createTrackbar( "min_S", "Kalibrasi", &min_S, max_val, on_trackbar );
	createTrackbar( "max_S", "Kalibrasi", &max_S, max_val, on_trackbar );
	createTrackbar( "min_V", "Kalibrasi", &min_V, max_val, on_trackbar );
	createTrackbar( "max_V", "Kalibrasi", &max_V, max_val, on_trackbar );

	createTrackbar("lowThreshold", "Kalibrasi", &lowThreshold, max_lowThreshold, on_trackbar);
    createTrackbar("lowThreshold*ratio", "Kalibrasi", &ratio, 50, on_trackbar);
    createTrackbar("kernel_size", "Kalibrasi", &kernel_size, 50, on_trackbar);

    createTrackbar("minLinLength", "Kalibrasi", &thLine, 500, on_trackbar);
    createTrackbar("minLinLength", "Kalibrasi", &minLinLength, 200, on_trackbar);
    createTrackbar("maxLineGap", "Kalibrasi", &maxLineGap, 200, on_trackbar);
    
}

void GoalFinder::morphOps(Mat &thresh){
        //create structuring element that will be used to "dilate" and "erode" image.
        //the element chosen here is a 3px by 3px rectangle
        Mat erodeElement = getStructuringElement(MORPH_RECT, Size(4, 4));
        //dilate with larger element so make sure object is nicely visible
        Mat dilateElement = getStructuringElement(MORPH_RECT, Size(5, 5));

        erode(thresh, thresh, erodeElement);
        erode(thresh, thresh, erodeElement);
        erode(thresh, thresh, erodeElement);

        dilate(thresh, thresh, dilateElement);
        dilate(thresh, thresh, dilateElement);
        dilate(thresh, thresh, dilateElement);
}

void GoalFinder::Filtering(Mat image)
{
    Mat drawing;
   	Reset();
	//GaussianBlur( image, img, Size(3, 3), 2, 2 );
	//GaussianBlur( img, img, Size(9, 9), 2, 2);
	cvtColor(image,hsv,CV_BGR2HSV);
	inRange(hsv,Scalar(min_H,min_S,min_V),Scalar(max_H,max_S,max_V),th);
	morphOps(th);
	GaussianBlur( th, th, Size(3, 3), 2, 2);
	
	Canny( th, edge, lowThreshold, lowThreshold*ratio, kernel_size);
	//cvtColor(edge,drawing,CV_GRAY2BGR);
    GaussianBlur( edge, edge, Size(5, 5), 3, 3);


    Point horizontal1, horizontal2;
    Point vertikal1_L, vertikal2_L;
    Point vertikal1_R, vertikal2_R;
    int h=0, l=0, r=0;
    
    HoughLines(edge, lines, 1, CV_PI/180, 220, 0, 0 );
    printf("LINE : %d\n",lines.size());
    
    float r_sum_h=0, r_sum_l=0, r_sum_r=0;
    float theta_sum_h=0, theta_sum_l=0, theta_sum_r=0;
    float theta_h, rho_h, theta_r, rho_r, theta_l, rho_l;
    //printf("\n░░░░░░░░░░░░░░░░░░░░░\n");
    for( size_t i = 0; i < lines.size(); i++ )
    {
        rho = lines[i][0]; theta = lines[i][1];//radian
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        line(image, pt1, pt2, Scalar(0,0,255), 1, CV_AA);
        
        //printf("\n░░░░░░░░░░░░░░░░░░░░░\n");
        //printf("RHO:%f\n", rho );
        //printf("TETHA:%f\n", theta );
        //printf("[Titik1] X:%d, Y:%d\n", pt1.x, pt1.y);
        //printf("[Titik2] X:%d, Y:%d\n", pt2.x, pt2.y);
        //printf("░░░░░░░░░░░░░░░░░░░░░░░░\n");

        //ClassifyLineHV(rho,RadiansToDegrees(theta));
        //CalculateMeanH();
    	
        double degtheta = RadiansToDegrees(theta);
        printf("TETHA:%f\n", degtheta );
        //printf("DEGTHETA:%f\n",degtheta);
        if((degtheta >= 45 && degtheta < 135)||(degtheta >= 225 && degtheta < 315)) //revisi 1
        {
            h++;
            horizontal1.x += pt1.x;
            horizontal2.x += pt2.x;
            horizontal1.y += pt1.y;
            horizontal2.y += pt2.y;
            r_sum_h += rho;
            theta_sum_h += theta;
        } else {
        	if(rho < -400 )
	        {
	            r++;
	            vertikal1_R.x += pt1.x;
	            vertikal2_R.x += pt2.x;
	            vertikal1_R.y += pt1.y;
	            vertikal2_R.y += pt2.y;
	        	r_sum_r += rho;
	        	theta_sum_r += theta;
	        }
	        if(rho < 0 && rho > -200)
	        {
	            l++;
	            vertikal1_L.x += pt1.x;
	            vertikal2_L.x += pt2.x;
	            vertikal1_L.y += pt1.y;
	            vertikal2_L.y += pt2.y;
	        	r_sum_l += rho;
	        	theta_sum_l += theta;
	        }
        }
        

    }
    printf("\n░░░░░░░░░░░░░░░░░░░░░\n");
    ClassifyLineHV();
    if(ValidateV())
    {
        CalculateMeanV();
        CalculateVarianceV();
    }
	ClassifyLineRL();
	printf("Jumlah Vertikal : %d\n", vertical.size());
/*
	//Test isi vector Horizontal
    for( size_t i = 0; i < horizontal.size(); i++ )
    {
        float r_H = horizontal[i][0], t_H = horizontal[i][1];
        printf("HORIZONTAL >> R:%f,T:%f\n",r_H,t_H);
    }

    //Test isi vector Vertical
    for( size_t i = 0; i < vertical.size(); i++ )
    {
        float r_V = vertical[i][0], t_V = vertical[i][1];
        printf("VERTICAL >> R:%f,T:%f\n",r_V,t_V);
    }

    
    //Test isi vector right_post
    for( size_t i = 0; i < right_post.size(); i++ )
    {
    	float rp_r = right_post[i][0], tp_r = right_post[i][1];
    	printf("R:%f, T:%f\n",rp_r, tp_r);
    }
*/

    //Test isi vector left_post
    /*
    for( size_t i = 0; i < left_post.size(); i++ )
    {
    	float rp_l = left_post[i][0], tp_l = left_post[i][1];
    	printf("R:%f, T:%f\n",rp_l, tp_l);
    }
    */
  
    DrawLine(image, r_mean_h, DegreesToRadians(teta_mean_h));
    DrawLine(image, r_mean_right, DegreesToRadians(teta_mean_right));
    DrawLine(image, r_mean_left, DegreesToRadians(teta_mean_left));  

    circle(image, CalculateIntersection(r_mean_h, teta_mean_h, r_mean_right, teta_mean_right), 5,  Scalar(0,255,255), 5, 8, 0 );
    circle(image, CalculateIntersection(r_mean_h, teta_mean_h, r_mean_left, teta_mean_left), 5,  Scalar(0,255,255), 5, 8, 0 );
    Point RightIntersec = CalculateIntersection(r_mean_h, teta_mean_h, r_mean_right, teta_mean_right);
    cout << "Right Intersec = " << RightIntersec.x << " " << RightIntersec.y << endl;
    Point LeftIntersec = CalculateIntersection(r_mean_h, teta_mean_h, r_mean_left, teta_mean_left);
    cout << "Left Intersec = " << LeftIntersec.x << " " << LeftIntersec.y << endl;
    circle(image, RunDown(th, RightIntersec, teta_mean_right, 1), 5, Scalar(0,255,255), 5, 8, 0);
    Point RightDown = RunDown(th, RightIntersec, teta_mean_right, 1);
    circle(image, RunDown(th, LeftIntersec, teta_mean_left, 1), 5, Scalar(0,255,255), 5, 8, 0);
    Point LeftDown = RunDown(th, LeftIntersec, teta_mean_left, 1);
    cout << "Right Down = " << RightDown.x << " " << RightDown.y << endl;
    cout << "Left Down = " << LeftDown.x << " " << LeftDown.y << endl;
    waitKey(30);
	
    
	namedWindow("Image", CV_WINDOW_NORMAL);
	imshow( "Image", image );

	//namedWindow("drawing", CV_WINDOW_NORMAL);
	//imshow( "drawing", drawing );
	
}

void GoalFinder::ControlPanel(minIni* ini)
{
    CreateTrackbar();
    
    namedWindow("Edge", CV_WINDOW_NORMAL);
    imshow( "Edge", edge );
    
    namedWindow("Threshold", CV_WINDOW_NORMAL);
    imshow( "Threshold", th );

    
    SaveINISettings(ini);

    waitKey(30);
}

void GoalFinder::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, COLOR_SECTION);
}

void GoalFinder::LoadINISettings(minIni* ini, const std::string &section)
{
    int value = -2;
    if((value = ini->geti(section, "min_H", INVALID_VALUE)) != INVALID_VALUE) min_H = value;
    if((value = ini->geti(section, "max_H", INVALID_VALUE)) != INVALID_VALUE) max_H = value;
    if((value = ini->geti(section, "min_S", INVALID_VALUE)) != INVALID_VALUE) min_S = value;
    if((value = ini->geti(section, "max_S", INVALID_VALUE)) != INVALID_VALUE) max_S = value;
    if((value = ini->geti(section, "min_V", INVALID_VALUE)) != INVALID_VALUE) min_V = value;
    if((value = ini->geti(section, "max_V", INVALID_VALUE)) != INVALID_VALUE) max_V = value;

    if((value = ini->geti(section, "lowThreshold", INVALID_VALUE)) != INVALID_VALUE) lowThreshold = value;
    if((value = ini->geti(section, "ratio", INVALID_VALUE)) != INVALID_VALUE)        ratio = value;
    if((value = ini->geti(section, "kernel_size", INVALID_VALUE)) != INVALID_VALUE)  kernel_size = value;

    color_section = section;
}

void GoalFinder::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, COLOR_SECTION);
}

void GoalFinder::SaveINISettings(minIni* ini, const std::string &section)
{
    ini->put(section,   "min_H", min_H);
    ini->put(section,   "max_H", max_H);
    ini->put(section,   "min_S", min_S);
    ini->put(section,   "max_S", max_S);
    ini->put(section,   "min_V", min_V);
    ini->put(section,   "max_V", max_V);

    ini->put(section,   "lowThreshold", lowThreshold);
    ini->put(section,   "ratio", ratio);
    ini->put(section,   "kernel_size", kernel_size);

    color_section = section;
}

void GoalFinder::printParam()
{
    printf("  ══════════════════╦══════════\n\r");
    printf("   Parameter        ║ Nilai       \n\r");
    printf("  ══════════════════╬══════════\n\r");
    printf("   min_H            ║   %d   \n\r", min_H);
    printf("   max_H            ║   %d   \n\r", max_H);
    printf("   min_S            ║   %d   \n\r", min_S);
    printf("   max_S            ║   %d   \n\r", max_S);
    printf("   min_V            ║   %d   \n\r", min_V);
    printf("   max_V            ║   %d   \n\r", max_V);
    printf("  ══════════════════╬══════════\n");
    printf("   lowThreshold     ║   %d   \n\r", lowThreshold);
    printf("   ratio            ║   %d   \n\r", ratio);
    printf("   kernel_size      ║   %d   \n\r", kernel_size);
    printf("  ══════════════════╩══════════\n\r");
}

void GoalFinder::getCorner(Mat src)
{
    int thresh = 201;
    Mat gray;
    
    // Load source image and convert it to gray
    cvtColor( src, gray, CV_BGR2GRAY );
    Mat dst, dst_norm, dst_norm_scaled;
    dst = Mat::zeros( src.size(), CV_32FC1 );

    //GaussianBlur( src, src, Size(9, 9), 4, 4);
 
    // Detecting corners
    cornerHarris( gray, dst, 2, 3, 0.01, BORDER_DEFAULT );
 
    // Normalizing
    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );
 
    // Drawing a circle around corners
    for( int j = 0; j < dst_norm.rows ; j++ )
    { 
        for( int i = 0; i < dst_norm.cols; i++ )
        {
            if( (int) dst_norm.at<float>(j,i) > thresh )
            {
                circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0,255,255), 1, 8, 0 );
                printf("(%d,%d)\n",i,j );
            }
        }
    }

    namedWindow( "corners_window", CV_WINDOW_NORMAL );
    imshow( "corners_window", dst_norm_scaled );
}

void GoalFinder::ClassifyLineHV() //theta dalam degree
{
	for( size_t i = 0; i < lines.size(); i++ )
    {
    //for(std::vector<Vec2f>::iterator iter = lines.begin() ; iter < lines.end() ; iter++)
    //{
    	float rho = lines[i][0]; 
    	float theta = RadiansToDegrees(lines[i][1]);//radian
    	
    	if((theta >= 45 && theta <= 135) || (theta >= 225) && (theta <= 315)) // HORIZONTAL
    	{
        	// Insert to Vector Horizontal
        	horizontal.push_back(Vec2f(rho,theta));
    	}
    	else // VERTICAL
    	{
    		//printf("t:%f\n",theta);
        	//Insert to Vector Vertical
        	vertical.push_back(Vec2f(rho,theta));
    	}
    }
    
}

void GoalFinder::CalculateMeanH()
{
    int r_sum = 0;
    float teta_sum_sin = 0;
    float teta_sum_cos = 0;
    for(std::vector<Vec2f>::iterator iter = horizontal.begin() ; iter < horizontal.end() ; iter++)
    {
        r_sum += (int)(*iter)[0];
        teta_sum_sin += sin((*iter)[1] *PI/180);
        teta_sum_cos += cos((*iter)[1] *PI/180);
    }
    r_mean_h = r_sum / horizontal.size();
    teta_mean_h = atan2(teta_sum_sin/2,teta_sum_cos/2)/PI*180;
}

void GoalFinder::CalculateMeanV()
{
    int r_sum = 0;
    float teta_sum_sin = 0;
    float teta_sum_cos = 0;
    
    for(std::vector<Vec2f>::iterator iter = vertical.begin() ; iter < vertical.end() ; iter++)
    {
        r_sum += (int)(*iter)[0];
        teta_sum_sin += sin((*iter)[1] *PI/180);
        teta_sum_cos += cos((*iter)[1] *PI/180);
    }
    r_mean_v = r_sum / vertical.size();
    teta_mean_v = atan2(teta_sum_sin/2,teta_sum_cos/2)/PI*180;
}

void GoalFinder::CalculateVarianceV()
{
    int temp_var = 0;
    //for(vector<Vec2f>::iterator iter = vertical.begin(); iter<vertical.end() ;iter++)
    //{
    for( size_t i = 0; i < vertical.size(); i++ )
    {
    	printf(">>>>>>>>%d\n",((int)vertical[i][0]));
        temp_var += (((int)vertical[i][0]) - r_mean_v) * (((int)vertical[i][0]) - r_mean_v);
    }
    variansi_v = temp_var / (vertical.size());
    printf("VARIANSI_V:%d\n",variansi_v);
    printf("t : %d\n", vertical.size());
}

void GoalFinder::ClassifyLineRL()
{
    for(std::vector<Vec2f>::iterator iter=vertical.begin() ; iter<vertical.end() ; iter++)
    {
        if((int)(*iter)[0] > r_mean_v)
            right_post.push_back(*iter);
        else
            left_post.push_back(*iter);
    }
}

void GoalFinder::CalculateMeanRL()
{
    int r_sum = 0;
    float teta_sum_sin = 0;
    float teta_sum_cos = 0;

    // MEAN RIGHT
    for(std::vector<Vec2f>::iterator iter=right_post.begin();iter<right_post.end();iter++)
    {
        r_sum += (int)(*iter)[0];
        teta_sum_sin += sin((*iter)[1] *PI/180);
        teta_sum_cos += cos((*iter)[1] *PI/180);
    }
    r_mean_right = r_sum / right_post.size();
    teta_mean_right = atan2(teta_sum_sin/2,teta_sum_cos/2)/PI*180;

    //MEAN LEFT
    r_sum = 0;
    teta_sum_sin = 0;
    teta_sum_cos = 0;

    for(std::vector<Vec2f>::iterator iter=left_post.begin();iter<left_post.end();iter++)
    {
        r_sum += (int)(*iter)[0];
        teta_sum_sin += sin((*iter)[1] *PI/180);
        teta_sum_cos += cos((*iter)[1] *PI/180);
    }
    r_mean_left = r_sum / left_post.size();
    teta_mean_left = atan2(teta_sum_sin/2,teta_sum_cos/2)/PI*180;
}

bool GoalFinder::ValidateH()
{
    return(horizontal.size() != 0);
}

bool GoalFinder::ValidateV()
{
    return(vertical.size() > 0);
}

void GoalFinder::StateCheck()
{
    if((!ValidateH()) && (!ValidateV()))
        goalstate = NL;
    else if(ValidateH())
    {
        if(!ValidateV())
            goalstate = CO;
        else if(variansi_v >= variansi)
            goalstate = C2P;
        else if(variansi_v < variansi)
            goalstate = C1P;
    }
    else if(ValidateV())
    {
        if(variansi_v >= variansi)
            goalstate = NC2P;
        else if(variansi_v < variansi)
            goalstate = NC1P;
    }
}

Point GoalFinder::CalculateIntersection(int R1, float Teta1, int R2, float Teta2)
{
	float point_x;
	float point_y;
	if((Teta1 == 90)||(Teta1 == 270)) // Straight Horizontal
	{
		float teta_rad_v = DegreesToRadians(Teta2);
		point_x = R2*cos(teta_rad_v)- R1*tan(teta_rad_v) + R2*sin(teta_rad_v)*tan(teta_rad_v) ; 
		point_y = R1;
	}
	else if ((Teta2 == 0)||(Teta2 == 180)) // Straight Vertical
	{
		float teta_rad_h = DegreesToRadians(Teta1);
		point_x = R2;
		point_y = R1*sin(teta_rad_h) - R2/tan(teta_rad_h) + R1*cos(teta_rad_h)/tan(teta_rad_h) ;
	}
	else
	{
		float teta_rad_h = DegreesToRadians(Teta1);
		float teta_rad_v = DegreesToRadians(Teta2);
		float c_vert = R2*(cos(teta_rad_v)+(sin(teta_rad_v)*tan(teta_rad_v)))/tan(teta_rad_v);
		float c_horz = R1*(sin(teta_rad_h)+(cos(teta_rad_h)/tan(teta_rad_h))); 
		point_x = (c_vert - c_horz)*tan(teta_rad_v)*tan(teta_rad_h)/(tan(teta_rad_h) - tan(teta_rad_v));
		point_y = c_vert - (point_x / tan(teta_rad_v));
	}
	return(Point(point_x,point_y));
}

float GoalFinder::DegreesToRadians(float degrees){
    return (degrees*PI/180);
}

float GoalFinder::RadiansToDegrees(float radians){
    return (radians*180/PI);
}

void GoalFinder::DrawLine(Mat image, float r_draw, float t_draw) {
    printf("R:%f, T:%f\n",r_draw,t_draw);
    double a = cos(t_draw), b = sin(t_draw);
    double x0 = a*r_draw, y0 = b*r_draw;
    Point p1,p2;
    p1.x = cvRound(x0 + 1000*(-b));
    p1.y = cvRound(y0 + 1000*(a));
    p2.x = cvRound(x0 - 1000*(-b));
    p2.y = cvRound(y0 - 1000*(a));
    printf("x1:%d, y1:%d, x2:%d, y2:%d\n", p1.x,p1.y,p2.x,p2.y);
    line(image, p1, p2, Scalar(0,0,0), 3, CV_AA);
}

void GoalFinder::Process(Mat image, bool OppGoal)
{
	Mat hsv;
	Mat th;
	Mat edge;

	cvtColor(image,hsv,CV_BGR2HSV);
	inRange(hsv,Scalar(min_H,min_S,min_V),Scalar(max_H,max_S,max_V),th);
	morphOps(th);
	Canny(th,edge,lowThreshold,lowThreshold*ratio,kernel_size);
    GaussianBlur( edge, edge, Size(5, 5), 3, 3);
    HoughLines(edge, lines, 1, CV_PI/180, 220, 0, 0 );
    ClassifyLineHV();
    if(ValidateV())
    {
    	CalculateMeanV();
    	CalculateVarianceV();
    }
    StateCheck();

    switch(goalstate)
    {
        case C2P:
                
                break;

        case C1P:
                
				break;

        case CO :
                
                break;

        case NC2P :
                
                break;

        case NC1P :
                
                break;

        case NL :
                
                break;
    }
}

void GoalFinder::Reset()
{
	lines.clear();
	horizontal.clear();
    vertical.clear();
    left_post.clear();
    right_post.clear();
    goalstate = NL;
}

oint GoalFinder::RunDown(Mat thresh, Point start, int teta, int step)
{
	int x;
	int xstart = start.x;
	int ystart = start.y;
	int lasty = -1;
	int lastx = -1;
	int count = 0;
	int miss ;
	miss = 0;	
    if(teta == 0)
	{
		x = xstart;
		for(int y = ystart ; y < Camera::HEIGHT ; y+=step)
		{
			count = 0;
            Scalar intensity = thresh.at<uchar>(y,x);

			for(int n = 0 ; n <= 3 ; n++)
			{
                if( intensity.val[0] == 255)
				{
					count++;
				}
			}
			
			// Extra check added, Considered as end of post when miss 3X 
			if(count < 1)
			{
				miss++;
			}
			else
			{
				miss = 0;
			}
			
			if(miss >= 3)
			{
				lastx = x;
				lasty = y;
				break;
			}
			else if((y >= Camera::HEIGHT - step)&&(miss<3))
			{
				lastx = x;
				lasty = Camera::HEIGHT;
			}
		}
	}
	else
	{ 
        float m = -1 / tan(DegreesToRadians(teta));	
		for(int y = ystart ; y < Camera::HEIGHT ; y+=step)
		{
			count = 0;
            x = xstart + ((y - ystart) / m);
            Scalar intensity = thresh.at<uchar>(y,x);
			for(int n = 0 ; n <= 3 ; n++)
			{
                if( intensity.val[0] == 255)
				{
					count++;
				}
			}
			
			if(count < 1)
			{
				miss++;
			}
			else
			{
				miss = 0;
			}
			
			if(miss >= 3)
			{
				lastx = x;
				lasty = y;
				break;
			}
			else if((y >= Camera::HEIGHT - step)&&(miss<3))
			{
				lastx = xstart + (Camera::HEIGHT - ystart) / m;
				lasty = Camera::HEIGHT;
			}
		}
	}
	return Point(lastx ,lasty);
}

Point GoalFinder::RunRight(Mat thresh, Point start, int teta, int step)
{
    int lasty = -1;
    int lastx = -1;
    int count = 0;
    int y;
    int xstart = start.x;
    int ystart = start.y;
    if(teta == 90)
    {
        y = ystart;
        for(int x = xstart ; x < Camera::WIDTH ; x += step)
        {
            count = 0;
            Scalar intensity = thresh.at<uchar>(y,x);
            for(int n = -3 ; n <=3 ; n++)
            {
                if(intensity.val[0] == 255)
                {
                    count++;
                }
            }
            if(count < 1)
            {
                lastx = x;
                lasty = y;
                break;
            }
            else if((x >= Camera::WIDTH - step)&&(count>=1))
            {
                lastx = Camera::WIDTH;
                lasty = y;
            }
        }
    }
    else
    {
        double m = -1 / tan(DegreesToRadians(teta));  
        for(int x = xstart ; x < Camera::WIDTH ; x += step)
        {
            y = ystart + (x-xstart) * m ;
            Scalar intensity = thresh.at<uchar>(y,x);
            count = 0;
            for(int n = -3 ; n <=3 ; n++)
            {
                if(intensity.val[0] == 255)
                {
                    count++;
                }
            }
            if(count < 1)
            {
                lastx = x;
                lasty = y;
                break;
            }
            else if((x >= Camera::WIDTH - step)&&(count>=1))
            {
                lastx = Camera::WIDTH;
                lasty = ystart + (Camera::WIDTH-xstart) * m;
            }
            
    
        }
    }
    return Point(lastx,lasty);
}

Point GoalFinder::RunLeft(Mat thresh, Point start, int teta, int step)
{
    int lasty = -1;
    int lastx = -1;
    int count = 0;
    int y;
    int xstart = start.x;
    int ystart = start.y;
    if(teta == 90)
    {
        y = ystart;
        for(int x = xstart ; x > 0 ; x -= step)
        {
            count = 0;
            Scalar intensity = thresh.at<uchar>(y,x);
            for(int n = -3 ; n <=3 ; n++)
            {
                if(intensity.val[0] == 255)
                {
                    count++;
                }
            }
            if(count < 1)
            {
                lastx = x;
                lasty = y;
                break;
            }
            else if((x <= 0 + step)&&(count>=1))
            {
                lastx = 0;
                lasty = y;
            }
        }
    }
    else
    {
        double m = -1 / tan(DegreesToRadians(teta));  
        for(int x = xstart ; x > 0 ; x -= step)
        {
            y = ystart + (x-xstart) * m;
            count = 0;
            Scalar intensity = thresh.at<uchar>(y,x);
            for(int n = -3 ; n <=3 ; n++)
            {
                if(intensity.val[0] == 255)
                {
                    count++;
                }
            }
            if(count < 1)
            {
                lastx = x;
                lasty = y;
                break;
            }
            else if((x <= 0 + step)&&(count>=1))
            {
                lastx = 0;
                lasty = ystart + (Camera::WIDTH-xstart) * m;
            }
            
    
        }
    }
    return Point(lastx,lasty);
}
