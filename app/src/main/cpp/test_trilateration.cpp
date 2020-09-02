/** test_trilateration.cpp
 *
 * Author: Aleksei Smirnov <aleksei.smirnov@navigine.com>
 * Copyright (c) 2014 Navigine. All rights reserved.
 *
 */

#include "test_trilateration.h"
#include <iostream>
#include <android/log.h>
#include <string>
#include <sstream>


using namespace std;
struct beacon_data {
    string name;
    string uuid;
    double rssi;
    double distance;
    double x_cord;
    double y_cord;
    int majorId;
    int minorId;
    string floor;


};

void PrintHelp()
{
  printf( "This is unit test that demonstrates how you can use \n"
          "Navigine trilateration class in order to get planar coordinates \n"
          "of the user on the basis of distance measurements \n"
          "For more questions please contact aleksei.smirnov@navigine.com \n\n"
          );
}

extern "C" {

bool splitData(string data);
void setConstDataToBeacon();
beacon_data beacon1,beacon2,beacon3,beacon4,beacon5,beacon6,beacon7,beacon8,beacon9;


JNIEXPORT jintArray  JNICALL Java_com_nexenio_bleindoorpositioningdemo_ui_beaconview_CvUtil_SendBeaconDataToCpp(JNIEnv *env, jclass clazz,jstring msg) {



    setConstDataToBeacon();
    // TODO: implement processPathPlanningcpp1()
    __android_log_print(ANDROID_LOG_VERBOSE, "pahPlanning", "%s", "Inside JNI..JNI call suceess");
    //const char *cStr = (*env)->GetStringUTFChars(env, data, NULL);
    const char *cname;
    cname = env->GetStringUTFChars(msg, NULL);
    string data = cname;
  //  data="3~MINOR_ID:58104~RSSI:-70~DISTANCE:450~MINOR_ID:26324~RSSI:-67~DISTANCE:250~MINOR_ID:56039~RSSI:-68~DISTANCE:270";

 /*   __android_log_print(ANDROID_LOG_VERBOSE, "beacon", "data in test trilateration %s",
                        data.c_str());*/
    vector<string> tokens;
    stringstream check1(data);
    string temp;

    while (getline(check1, temp, '~')) {
        tokens.push_back(temp);
    }
    /*for (int i = 0; i < tokens.size(); i++) {
        __android_log_print(ANDROID_LOG_VERBOSE, "beacon", "token data = %s", tokens[i].c_str());

    }*/
    vector<BeaconMeas> beaconMeasurements;
    vector<Beacon> mapBeacons;
    Trilateration trilateration;
    int count = stoi(tokens[0]);
    int minor_id, rssi;
    double distance;
    try {
        for (int i = 0; i < count; i++) {
            minor_id = stoi(tokens[(3 * i) + 1].substr(tokens[(3 * i) + 1].find(":") + 1));
            rssi = stoi(tokens[(3 * i) + 2].substr(tokens[(3 * i) + 2].find(":") + 1));
            distance = stod(tokens[(3 * i) + 3].substr(tokens[(3 * i) + 3].find(":") + 1));
            /*__android_log_print(ANDROID_LOG_VERBOSE, "beacon","  minor ids = %s rssi =%s  distance =%s",
                                to_string(minor_id).c_str(), to_string(rssi).c_str(),
                                to_string(distance).c_str());*/
            Beacon beacon;
            BeaconMeas measurement;
            switch (minor_id) {
                case 58104:
                    //__android_log_print(ANDROID_LOG_VERBOSE, "beacon", "distance from  %s = %s", beacon1.name.c_str() , to_string(distance).c_str());
                    beacon.fillData(beacon1.x_cord, beacon1.y_cord, "(" + to_string(beacon1.majorId) + "," +to_string(beacon1.minorId) + "," + beacon1.uuid + ")", beacon1.name, beacon1.floor);
                    measurement.setBeaconId("(" + to_string(beacon1.majorId) + "," + to_string(beacon1.minorId) + "," + beacon1.uuid + ")");
                    break;
                case 26324:
                    //__android_log_print(ANDROID_LOG_VERBOSE, "beacon", "distance from  %s = %s", beacon2.name.c_str() , to_string(distance).c_str());
                    beacon.fillData(beacon2.x_cord, beacon2.y_cord,"(" + to_string(beacon2.majorId) + "," +to_string(beacon2.minorId) + "," + beacon2.uuid + ")",beacon2.name, beacon2.floor);
                    measurement.setBeaconId("(" + to_string(beacon2.majorId) + "," + to_string(beacon2.minorId) +"," + beacon2.uuid + ")");
                    break;
                case 56039:
                    //__android_log_print(ANDROID_LOG_VERBOSE, "beacon", "distance from  %s = %s", beacon3.name.c_str() , to_string(distance).c_str());
                    beacon.fillData(beacon3.x_cord, beacon3.y_cord,"(" + to_string(beacon3.majorId) + "," +to_string(beacon3.minorId) + "," + beacon3.uuid + ")",beacon3.name, beacon3.floor);
                    measurement.setBeaconId("(" + to_string(beacon3.majorId) + "," + to_string(beacon3.minorId) +"," + beacon3.uuid + ")");
                    break;
                case 63458 :
                    //__android_log_print(ANDROID_LOG_VERBOSE, "beacon", "distance from  %s = %s", beacon4.name.c_str() , to_string(distance).c_str());
                    beacon.fillData(beacon4.x_cord, beacon4.y_cord,"(" + to_string(beacon4.majorId) + "," + to_string(beacon4.minorId) + "," + beacon4.uuid + ")", beacon4.name, beacon4.floor);
                    measurement.setBeaconId("(" + to_string(beacon4.majorId) + "," + to_string(beacon4.minorId) + "," + beacon4.uuid + ")");
                    break;
                case 31369 :
                    //__android_log_print(ANDROID_LOG_VERBOSE, "beacon", "distance from  %s = %s", beacon5.name.c_str() , to_string(distance).c_str());
                    beacon.fillData(beacon5.x_cord, beacon5.y_cord,"(" + to_string(beacon5.majorId) + "," + to_string(beacon5.minorId) + "," + beacon5.uuid + ")",beacon5.name, beacon5.floor);
                    measurement.setBeaconId("(" + to_string(beacon5.majorId) + "," + to_string(beacon5.minorId) +"," + beacon5.uuid + ")");
                    break;
                case 52844 :
                   // __android_log_print(ANDROID_LOG_VERBOSE, "beacon", "distance from  %s = %s", beacon6.name.c_str() , to_string(distance).c_str());
                    beacon.fillData(beacon6.x_cord, beacon6.y_cord,"(" + to_string(beacon6.majorId) + "," + to_string(beacon6.minorId) + "," + beacon6.uuid + ")",beacon6.name, beacon6.floor);
                    measurement.setBeaconId("(" + to_string(beacon6.majorId) + "," + to_string(beacon6.minorId) +"," + beacon6.uuid + ")");
                    break;
                case 25389 :
                    //__android_log_print(ANDROID_LOG_VERBOSE, "beacon", "distance from  %s = %s", beacon7.name.c_str() , to_string(distance).c_str());
                    beacon.fillData(beacon7.x_cord, beacon7.y_cord,"(" + to_string(beacon7.majorId) + "," + to_string(beacon7.minorId) + "," + beacon7.uuid + ")",beacon7.name, beacon7.floor);
                    measurement.setBeaconId("(" + to_string(beacon7.majorId) + "," + to_string(beacon7.minorId) +"," + beacon7.uuid + ")");
                    break;
                case 44784 :
                    //__android_log_print(ANDROID_LOG_VERBOSE, "beacon", "distance from  %s = %s", beacon8.name.c_str() , to_string(distance).c_str());
                    beacon.fillData(beacon8.x_cord, beacon8.y_cord,"(" + to_string(beacon8.majorId) + "," + to_string(beacon8.minorId) + "," + beacon8.uuid + ")",beacon8.name, beacon8.floor);
                    measurement.setBeaconId("(" + to_string(beacon8.majorId) + "," + to_string(beacon8.minorId) +"," + beacon8.uuid + ")");
                    break;

                case 30190 :
                    //__android_log_print(ANDROID_LOG_VERBOSE, "beacon", "distance from  %s = %s", beacon9.name.c_str() , to_string(distance).c_str());
                    beacon.fillData(beacon9.x_cord, beacon9.y_cord,"(" + to_string(beacon9.majorId) + "," + to_string(beacon9.minorId) + "," + beacon9.uuid + ")",beacon9.name, beacon9.floor);
                    measurement.setBeaconId("(" + to_string(beacon9.majorId) + "," + to_string(beacon9.minorId) +"," + beacon9.uuid + ")");
                    break;
            }
            mapBeacons.push_back(beacon);
            measurement.setRssi(rssi);
            measurement.setDistance(distance);
            beaconMeasurements.push_back(measurement);

        }

        if (beaconMeasurements.size() == 0) {
            //printf( "Number of measurements == 0\n" );
            return nullptr;
        }
        trilateration.updateMeasurements(beaconMeasurements);
        trilateration.setCurrentLocationId(0);
        trilateration.fillLocationBeacons(mapBeacons);
        int errorCode = trilateration.calculateCoordinates();
        if (errorCode)
            return nullptr;
        double x = trilateration.getX();
        double y = trilateration.getY();
        int  cord_arr[] = {(int)x, (int)y};
        __android_log_print(ANDROID_LOG_VERBOSE, "beacon", "predicted x = %s y= %s",to_string(x).c_str(), to_string(y).c_str());

        //sendng  data to java
        jintArray result;
        result = env->NewIntArray(2);
        env->SetIntArrayRegion(result, 0,2, cord_arr);
        return result ;
    }
    catch (const char* e){
        __android_log_print(ANDROID_LOG_VERBOSE, "beacon", "exception occured", e);
        return nullptr;
    }
}

}

void setConstDataToBeacon(){
    const string UUID="f7826da6-4fa2-4e98-8024-bc5b71e0893e";
    const string floor="ground floor";

    // beacon 1
    beacon1.name="beacon1";
    beacon1.x_cord=140;
    beacon1.y_cord=0;
    beacon1.majorId=7026;
    beacon1.uuid=UUID;
    beacon1.floor=floor;

    // beacon 2
    beacon2.name="beacon2";
    beacon2.x_cord=0;
    beacon2.y_cord=240;
    beacon2.majorId=30179;
    beacon2.uuid=UUID;
    beacon2.floor=floor;

    // beacon 3
    beacon3.name="beacon3";
    beacon3.x_cord=150;
    beacon3.y_cord=420;
    beacon3.majorId=51907;
    beacon3.uuid=UUID;
    beacon3.floor=floor;

    // beacon 4
    beacon4.name="beacon4";
    beacon4.x_cord=550;
    beacon4.y_cord=190;
    beacon4.majorId=14640;
    beacon4.uuid=UUID;
    beacon4.floor=floor;

    // beacon 5
    beacon5.name="beacon5";
    beacon5.x_cord=320;
    beacon5.y_cord=420;
    beacon5.majorId=1356;
    beacon5.uuid=UUID;
    beacon5.floor=floor;

    // beacon 6
    beacon6.name="beacon6";
    beacon6.x_cord=520;
    beacon6.y_cord=580;
    beacon6.majorId=59776;
    beacon6.uuid=UUID;
    beacon6.floor=floor;

    // beacon 7
    beacon7.name="beacon7";
    beacon7.x_cord=412;
    beacon7.y_cord=225;
    beacon7.majorId=25389;
    beacon7.uuid=UUID;
    beacon7.floor=floor;

    // beacon 8
    beacon8.name="beacon8";
    beacon8.x_cord=38;
    beacon8.y_cord=400;
    beacon8.majorId=44784;
    beacon8.uuid=UUID;
    beacon8.floor=floor;
    // beacon 9
    beacon9.name="beacon9";
    beacon9.x_cord=254;
    beacon9.y_cord=442;
    beacon9.majorId=30190;
    beacon9.uuid=UUID;
    beacon9.floor=floor;
}





