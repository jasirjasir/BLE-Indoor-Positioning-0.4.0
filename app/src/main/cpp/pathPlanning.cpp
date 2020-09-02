#include <jni.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <android/log.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <vector>

using namespace cv;
using namespace std;
int path_count=0;
float cell_factor = 0.042;//(25/640)
class value;
vector<int> cord_vec;


void getPathThroughMaze(unsigned char* outputImage, unsigned char* binaryImage, int w, int h, int startX, int startY, int endX, int endY, bool drawCostMap,
                        Mat floor_img)
{

    unsigned char* visited = (unsigned char*)calloc(w * h, 1);
    unsigned int* pathValue = (unsigned int*)calloc(w * h, sizeof(unsigned int));



    std::queue<std::pair<int, int>> currentPoints;
    currentPoints.push(std::pair<int, int>(startX, startY));
    pathValue[startX + startY * w] = 1;
    visited[startX + startY * w] = 1;

    __android_log_print(ANDROID_LOG_VERBOSE, "pahPlanning", "running Digkstra algorithm");

    bool endReached = false;
    unsigned int maxPathValue = 0;


    int iter_count = 0;
    int temp1 = 0; int temp2 = 0;
    while (iter_count <= 3)
    {
        if (iter_count == 0) {
            temp1 = 0;
            temp2 = 0;
        }
        else if (iter_count == 1) {
            temp1 = 0;
            temp2 = -1;
        }
        else if (iter_count == 2) {
            temp1 = -1;
            temp2 = 0;
        }
        else if (iter_count == 3) {
            temp1 = -1;
            temp2 = -1;
        }
        __android_log_print(ANDROID_LOG_VERBOSE, "pahPlanning", "iteration count %d",iter_count);
        currentPoints.push(std::pair<int, int>(startX, startY));
        while (currentPoints.size() > 0)
        {
            std::pair<int, int> currPair = currentPoints.front();
            currentPoints.pop();

            int currX = currPair.first;
            int currY = currPair.second;

            unsigned int currPathValue = pathValue[currX + currY * w];
            if (currPathValue > maxPathValue)
                maxPathValue = currPathValue;

            if (currX == endX && currY == endY)
            {
                cout << "end reached" << endl;
                __android_log_print(ANDROID_LOG_VERBOSE, "beacon", "end reached ");
                endReached = true;
                break;
            }

            for (int i = temp1; i < 2; i++)
            {
                for (int j = temp2; j <2; j++)
                {
                    if (currX + j < 0 || currY + i < 0 || currX + j >= w || currY + i >= h)
                        continue;
                    if (binaryImage[(currX + j) + (currY + i) * w] && !visited[(currX + j) + (currY + i) * w])
                    {
                        pathValue[(currX + j) + (currY + i) * w] = currPathValue + 1;
                        visited[(currX + j) + (currY + i) * w] = 1;
                        currentPoints.push(std::pair<int, int>(currX + j, currY + i));

                    }
                }
            }

        }

        if (!endReached)
        {
            cout << "iter count=" << iter_count << endl;
            visited = (unsigned char*)calloc(w * h, 1);
            pathValue = (unsigned int*)calloc(w * h, sizeof(unsigned int));
            if (iter_count == 3) {
                printf("Digkstra can't fine any suitable path\n");
                __android_log_print(ANDROID_LOG_VERBOSE, "beacon", "igkstra can't fine any suitable path ");
                return;
            }

            iter_count++;

        }
        else {
            break;
        }
    }
    if (drawCostMap)
    {
        double multiplier = 1.0 / maxPathValue;
        for (int x = 0; x < w; x++)
        {
            for (int y = 0; y < h; y++)
            {
                for (int i = 0; i < 3; i++)
                    outputImage[3 * (x + y * w) + i] = static_cast<unsigned char>(255 * multiplier * pathValue[x + y * w]);
            }
        }

    }

    //printf("Drawing path\n");
    unsigned int currPathValue = pathValue[endX + endY * w];
    int currX = endX;
    int currY = endY;
    while (currPathValue != 1)
    {
        outputImage[3 * (currX + currY * w) + 0] = 0;
        outputImage[3 * (currX + currY * w) + 1] = 255;
        outputImage[3 * (currX + currY * w) + 2] = 0;

        int bestMoveX = 0;
        int bestMoveY = 0;
        unsigned int bestMovePathVal = maxPathValue;
        for (int i = -1; i < 2; i++)
        {
            for (int j = -1; j < 2; j++)
            {
                if (i == 0 && j == 0)
                    continue;
                if (currX + j < 0 || currY + i < 0 || currX + j >= w || currY + i >= h)
                    continue;
                if (pathValue[(currX + j) + (currY + i) * w] == 0)
                    continue;
                if (pathValue[(currX + j) + (currY + i) * w] < bestMovePathVal)
                {
                    bestMovePathVal = pathValue[(currX + j) + (currY + i) * w];
                    bestMoveX = currX + j;
                    bestMoveY = currY + i;
                }
            }
        }

        currX = bestMoveX;
        currY = bestMoveY;
        cord_vec.push_back(currX);
        cord_vec.push_back(currY);
        currPathValue = pathValue[currX + currY * w];



    }

}


extern "C" {

JNIEXPORT jintArray JNICALL Java_com_nexenio_bleindoorpositioningdemo_ui_beaconview_CvUtil_processPathPlanning
        (JNIEnv *env, jclass obj, jlong addrImg,jintArray cord_array) {

    const char *LOGTAG="beacon";
    cv::Mat *pMatRgb = (cv::Mat *) addrImg;
    cv::Mat image = *pMatRgb;
    Mat image_out = image.clone();

    if (image.empty()) {
        __android_log_print(ANDROID_LOG_VERBOSE, LOGTAG, "%s", "Could not open or find the image");
    }
    __android_log_print(ANDROID_LOG_VERBOSE,LOGTAG, "%s", "Apply Digkstra on maze");
    __android_log_print(ANDROID_LOG_VERBOSE, "beacon", "height =%d  width=%d   ",image.rows,image.cols);
    //__android_log_print(ANDROID_LOG_VERBOSE, "beacon", "height =%d  width=%d   ",image.rows,image.cols);
    Mat grayImage;
    cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    Mat binaryImage(grayImage.size(), grayImage.type());
    threshold(grayImage, binaryImage, 128, 255, cv::THRESH_BINARY);


    jint *body = (env)->GetIntArrayElements( cord_array, 0);
    int startX = body[0];
    int startY = body[1];
    int endX = body[2];
    int endY = body[3];

  /*  startX = 788;
    startY = 295;
    endX = 460;
    endY = 73;*/

    __android_log_print(ANDROID_LOG_VERBOSE,LOGTAG, "startx =%d  starty=%d    endx =%d  endy=%d   ",startX,startY,endX,endY);
    int w = image.cols;
    int h = image.rows;
    unsigned char* outputImage;
    outputImage = new unsigned char[w * h * 3]();
    path_count = 0;
    getPathThroughMaze(outputImage, binaryImage.data, w, h, startX, startY, endX, endY, false,image_out);
    int distance = path_count * cell_factor;
    /*for (unsigned int i = 0; i < cord_vec.size(); i+=2)
        __android_log_print(ANDROID_LOG_DEBUG, "beacon", "X=%d -Y=%d" ,cord_vec[i], cord_vec[i+1]);*/
    __android_log_print(ANDROID_LOG_DEBUG, LOGTAG, "number of cells travelled %d",path_count);

  //send data to java
    jintArray result;
    result = env->NewIntArray(cord_vec.size());
    // fill a temp structure to use to populate the java int array
    jint cord_arr[cord_vec.size()];
    copy(cord_vec.begin(), cord_vec.end(), cord_arr);
    env->SetIntArrayRegion(result, 0,cord_vec.size(), cord_arr);
    cord_vec.clear();
    return result ;

}


}

