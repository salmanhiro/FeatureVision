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
#include "BallFinder.h"

#define INI_FILE_PATH       "../../../../Data/config.ini"

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
    VideoCapture cap("Adidas ball Size 1.mp4");
    Mat image;
    printf( "\n===== Color filtering Tutorial for DARwIn =====\n\n");

    //change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);
    BallFinder::GetInstance()->LoadINISettings(ini);


    //BallFinder* finder = new BallFinder();
    //finder->LoadINISettings(ini);

    while(1)
    {
        Point2D pos;
        
        pos = BallFinder::GetInstance()->getPosition();
        //printf("░░░░░░░░░░░░░░░░░░░░░░░░░░░\nX:%lf, Y:%lf\n░░░░░░░░░░░░░░░░░░░░░░░░░░░\n", pos.X, pos.Y);
        cap.read(image);
        BallFinder::GetInstance()->Process(image);
        BallFinder::GetInstance()->ControlPanel(ini);
        
    }

    return 0;
}
