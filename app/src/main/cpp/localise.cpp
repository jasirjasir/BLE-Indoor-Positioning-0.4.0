//
// Created by Jasir on 07-07-2020.
//
#include <jni.h>
#include <android/log.h>
#include <iostream>
#include "trilateration.h"

extern "C" {

/*
JNIEXPORT jintArray JNICALL Java_com_nexenio_bleindoorpositioningdemo_ui_beaconview_CvUtil_SendStringDataToCpp(JNIEnv *env, jclass clazz,jstring msg) {


    // TODO: implement processPathPlanningcpp1()
    __android_log_print(ANDROID_LOG_VERBOSE, "pahPlanning", "%s", "Inside JNI..JNI call suceess");
    //const char *cStr = (*env)->GetStringUTFChars(env, data, NULL);
    const char *cname;
    cname = env->GetStringUTFChars(msg, NULL);
    std::string name = cname;
    __android_log_print(ANDROID_LOG_VERBOSE, "beacon", "%s", name.c_str());

    return nullptr;


}*/
JNIEXPORT jintArray JNICALL Java_com_nexenio_bleindoorpositioningdemo_ui_beaconview_CvUtil_SendIntArrayDataToCpp(JNIEnv *env, jclass clazz,jintArray result) {

    __android_log_print(ANDROID_LOG_VERBOSE, "beacon", "%s", "Inside JNI..JNI call success");
    const jsize length = env->GetArrayLength(result);
    __android_log_print(ANDROID_LOG_VERBOSE, "beacon", "array size %d", length);
    jint *body = (env)->GetIntArrayElements(result, NULL);
    int i;
    for (i=0; i<length; i++)
        __android_log_print(ANDROID_LOG_VERBOSE, "beacon", "data received are %d", body[i]);
    return nullptr;

}
}

