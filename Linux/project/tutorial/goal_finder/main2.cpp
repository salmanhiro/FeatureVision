/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <unistd.h>
#include <string.h>
#include <libgen.h>

//#include "Camera.h"
#include "Point.h"
#include "mjpg_streamer.h"
#include "minIni.h"
//#include "LinuxCamera.h"
//#include "ColorFinder.h"
#include "GoalFinder.h"

#define INI_FILE_PATH       "../../../../Data/config.ini"

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
    
    Mat image;
    printf( "\n===== Color filtering Tutorial for DARwIn =====\n\n");

    //change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);
    GoalFinder::GetInstance()->LoadINISettings(ini);


    //BallFinder* finder = new BallFinder();
    //finder->LoadINISettings(ini);
    VideoCapture cap(1);
    while(1)
    {
        
        Point2D pos;
        //cap.set(CV_CAP_PROP_EXPOSURE, 1000);
        //printf("░░░░░░░░░░░░░░░░░░░░░░░░░░░\nX:%lf, Y:%lf\n░░░░░░░░░░░░░░░░░░░░░░░░░░░\n", pos.X, pos.Y);
        cap.read(image);
        GoalFinder::GetInstance()->Process(image);
        GoalFinder::GetInstance()->ControlPanel(ini);
        
    }

    return 0;
}
