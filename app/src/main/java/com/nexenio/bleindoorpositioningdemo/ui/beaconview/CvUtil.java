package com.nexenio.bleindoorpositioningdemo.ui.beaconview;

import android.util.Log;

import org.opencv.core.Mat;

public class CvUtil {

   public static int[] SendData_string(String val){
       return SendBeaconDataToCpp(val);
       /*//Log.d("beacon", String.valueOf(array[0]));
       if (cordArray!=null){
           Log.d("beacon", "co ordinates received in java x="+cordArray[0]+" y="+cordArray[1]);
       }else {

       }*/

   }
    public static void  SendData_intArray(int[] array){
        Log.d("beacon", "data0 ="+array[0]);
        SendIntArrayDataToCpp(array);


    }
    public static int[] processPathPlanning(Mat srcMat, int[] input_array) {
        int[] output_array= processPathPlanning(srcMat.getNativeObjAddr(),input_array);
        return output_array;
    }

    public static native  int[] SendBeaconDataToCpp(String val);
    public static native  int[] SendIntArrayDataToCpp(int[] val);
    public static native  int[] processPathPlanning(long matAddr,int input_array[]);

}
