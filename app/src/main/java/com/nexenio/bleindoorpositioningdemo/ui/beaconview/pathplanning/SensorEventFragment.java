package com.nexenio.bleindoorpositioningdemo.ui.beaconview.pathplanning;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

import android.os.SystemClock;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;
import android.widget.Toast;

import com.nexenio.bleindoorpositioning.ble.advertising.AdvertisingPacket;
import com.nexenio.bleindoorpositioning.ble.advertising.AdvertisingPacketUtil;
import com.nexenio.bleindoorpositioningdemo.R;

import java.util.ArrayList;
import java.util.List;

/**
 * A simple {@link Fragment} subclass.
 * Use the {@link SensorEventFragment#newInstance} factory method to
 * create an instance of this fragment.
 */
public class SensorEventFragment extends Fragment implements SensorEventListener {

    // TODO: Rename parameter arguments, choose names that match
    // the fragment initialization parameters, e.g. ARG_ITEM_NUMBER
    private static final String ARG_PARAM1 = "param1";
    private static final String ARG_PARAM2 = "param2";

    // TODO: Rename and change types of parameters
    private String mParam1;
    private String mParam2;

    private static final float STRIDE_LENGTH = 74.0f;

    private SensorManager sensorManager;
    private DynamicStepCounter dynamicStepCounter;

    private final float[] accelerometerReading = new float[3];
    private final float[] magnetometerReading = new float[3];
    private final float[] linAccReading = new float[3];

    private final float[] rotationMatrix = new float[9];

    private double heading;
    private long positionUpdateTime;
    private long sensorStartTime;

    private float[] DRPose = new float[2];

    private float[] estimatedPose = new float[2];
    private float errorCovariance;
    private boolean updatePosition = false;

    ArrayList<Integer> beaconPoseList_X = new ArrayList<Integer>();
    ArrayList<Integer> beaconPoseList_Y=new ArrayList<Integer>();
    TextView tv1,tv2;
    private boolean stepDetector=true;

    public SensorEventFragment() {
        // Required empty public constructor
        Log.d("beacon", "inside SensorEventFragment " );
    }

    /**
     * Use this factory method to create a new instance of
     * this fragment using the provided parameters.
     *
     * @param param1 Parameter 1.
     * @param param2 Parameter 2.
     * @return A new instance of fragment SensorEventFragment.
     */
    // TODO: Rename and change types and number of parameters
    public static SensorEventFragment newInstance(String param1, String param2) {
        SensorEventFragment fragment = new SensorEventFragment();
        Bundle args = new Bundle();
        args.putString(ARG_PARAM1, param1);
        args.putString(ARG_PARAM2, param2);
        fragment.setArguments(args);
        return fragment;
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        if (getArguments() != null) {
            mParam1 = getArguments().getString(ARG_PARAM1);
            mParam2 = getArguments().getString(ARG_PARAM2);
        }

        sensorManager = (SensorManager) getActivity().getSystemService(Context.SENSOR_SERVICE);

        dynamicStepCounter = new DynamicStepCounter(1.0);
        heading = 0;
        estimatedPose[0] = 0;
        estimatedPose[1] = 0;
        errorCovariance = 0;

        positionUpdateTime = SystemClock.elapsedRealtime();
        sensorStartTime = SystemClock.elapsedRealtime();

    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // Inflate the layout for this fragment
        View view = inflater.inflate(R.layout.fragment_sensor_event, container, false);

        Bundle bundle = this.getArguments();
        if (bundle != null) {
            double[] beacon_pose = bundle.getDoubleArray("beacon_pose");
            beaconPoseList_X.add((int) beacon_pose[0]);
            beaconPoseList_Y.add((int) beacon_pose[1]);
        }
        Log.d("beacon", "inside fragment " );

        return view;
    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        tv1=(TextView)getView().findViewById(R.id.tv_1);
        tv2=(TextView)getView().findViewById(R.id.tv_2);
    }

    @Override
    public void onResume() {

        super.onResume();

        Sensor accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        if (accelerometer != null) {
            sensorManager.registerListener (this, accelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        }
        Sensor magneticField = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        if (magneticField != null) {
            sensorManager.registerListener(this, magneticField, SensorManager.SENSOR_DELAY_FASTEST);
        }
        if (stepDetector) {
            Sensor step_Detector = sensorManager.getDefaultSensor(Sensor.TYPE_STEP_DETECTOR);
            if(step_Detector != null) {
                sensorManager.registerListener(this, step_Detector, SensorManager.SENSOR_DELAY_FASTEST);
            }
        } else {
            Sensor linearAccelaration = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
            if (linearAccelaration != null) {
                sensorManager.registerListener(this, linearAccelaration, SensorManager.SENSOR_DELAY_FASTEST);
//                    SensorManager.SENSOR_DELAY_NORMAL, SensorManager.SENSOR_DELAY_UI);
            }
        }


    }

    @Override
    public void onPause() {

        super.onPause();

        sensorManager.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {

        //Restart sensors in every 10 sec to reduce the ingration error
        if ( (SystemClock.elapsedRealtime() - sensorStartTime) > 10000) {
            sensorStartTime = SystemClock.elapsedRealtime();
            onResume();
        }

        if (sensorEvent.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            System.arraycopy(sensorEvent.values, 0, accelerometerReading,0, accelerometerReading.length);
        }
        else if (sensorEvent.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            System.arraycopy(sensorEvent.values, 0, magnetometerReading, 0, magnetometerReading.length);
            SensorManager.getRotationMatrix(rotationMatrix, null, accelerometerReading, magnetometerReading);
            heading = Math.atan2(rotationMatrix[3], rotationMatrix[0]);
//            Log.d("onSensorChanged ", " sensor:M " + sensorEvent.sensor.getType() + ", heading: " + heading);

        }
        else if (sensorEvent.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
            System.arraycopy(sensorEvent.values, 0, linAccReading,0, linAccReading.length);
            float norm = calcNorm(sensorEvent.values[0] + sensorEvent.values[1] + sensorEvent.values[2]);
            Log.d("Norm ", ""+norm);
            boolean stepFound = dynamicStepCounter.findStep(norm);
            Log.d("step found ", ""+stepFound);
            if (stepFound) {
//                Log.d("onSensorChanged ", " estimatedPose: (" + estimatedPose[0] + ", " + estimatedPose[1] + ")" );

                float rPointX = (float) (estimatedPose[0] - STRIDE_LENGTH * Math.sin(heading) );
                float rPointY = (float) (estimatedPose[1] + STRIDE_LENGTH * Math.cos(heading) );

                Log.d("onSensorChanged ", "Sensor_Pose: (x, y) = " + rPointX + ", " + rPointY + ", heading: " + heading);
                tv1.setText("Sensor_Pose: (x, y) = " + rPointX + " , " + rPointY + "\n heading: " + heading);


                DRPose[0] = rPointX;
                DRPose[1] = rPointY;

                updatePosition = true;
                positionUpdateTime = SystemClock.elapsedRealtime();

            }
            else {
                DRPose = estimatedPose;
            }

        }
        else if (sensorEvent.sensor.getType() == Sensor.TYPE_STEP_DETECTOR) {

            Log.d("onSensorChanged", "Step detector: " + sensorEvent.values[0] );

            boolean stepFound = (sensorEvent.values[0] == 1);
            if (stepFound) {
//                Log.d("onSensorChanged ", " estimatedPose: (" + estimatedPose[0] + ", " + estimatedPose[1] + ")" );

                float rPointX = (float) (estimatedPose[0] - STRIDE_LENGTH * Math.sin(heading) );
                float rPointY = (float) (estimatedPose[1] + STRIDE_LENGTH * Math.cos(heading) );

                // Log.d("onSensorChanged ", "Sensor_Pose: (x, y) = " + rPointX + ", " + rPointY + ", heading: " + heading);
                // tv1.setText("Sensor_Pose: (x, y) = " + rPointX + " , " + rPointY + "\n heading: " + heading);


                DRPose[0] = rPointX;
                DRPose[1] = rPointY;

                updatePosition = true;
                positionUpdateTime = SystemClock.elapsedRealtime();

            }
            else {
                DRPose = estimatedPose;
            }

        }
        if ( (!updatePosition) && ((SystemClock.elapsedRealtime() - positionUpdateTime) > 5000) ) {

            updatePosition = true;
            Log.d("onSensorChanged", "Time elapsed....");
            positionUpdateTime = SystemClock.elapsedRealtime();
        }
//        Log.d("onSensorChanged : ", " accelerometerReading: " + accelerometerReading[0] + ", " + accelerometerReading[1] + ", " + accelerometerReading[2]
//                    + " magnetometerReading: " + magnetometerReading[0] + ", " + magnetometerReading[1] + ", " + magnetometerReading[2]
//                    + "; LinAcc: " + linAccReading[0] + ", " + linAccReading[1] + ", " + linAccReading[2]);
//
//                Log.d("onSensorChanged ", " sensor: " + sensorEvent.sensor.getType() + ", heading: " + heading + "(" + DRPose[0] + ", " + DRPose[1] + ")" );

        if (updatePosition) {

            updatePosition = false;

            float[] meanBeaconPose = new float[2];

            if (beaconPoseList_X.size() != 0) {

                meanBeaconPose = calculateMean();
                beaconPoseList_X.clear();
                beaconPoseList_Y.clear();

                calculateKalmanDistance(DRPose, meanBeaconPose);

            }else {
                estimatedPose = DRPose;
            }

            Log.d("onSensorChanged ", " DR_pose: (" + DRPose[0] + ", " + DRPose[1] + "), beacon_pose: (" + meanBeaconPose[0] + ", " + meanBeaconPose[1]
                    + "), fusion_pose: (" + estimatedPose[0] + ", " + estimatedPose[1] + ")" );

            tv2.setText(" DR_pose: (" + DRPose[0] + ", " + DRPose[1] + ") \n beacon_pose: (" + meanBeaconPose[0] + ", " + meanBeaconPose[1]
                    + ")\n fusion_pose: (" + estimatedPose[0] + ", " + estimatedPose[1] + ")" );
//            Bundle bundle = new Bundle();
//            bundle.putFloatArray("fusion_pose", estimatedPose);
//            BeaconNavigateFragment beaconNavigateFragment = new BeaconNavigateFragment();
//            beaconNavigateFragment.setArguments(bundle);
//            getFragmentManager().beginTransaction().commit();
        }

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public static float calcNorm(double... args) {
        double sumSq = 0;
        for (double arg : args)
            sumSq += Math.pow(arg, 2);
        return (float)Math.sqrt(sumSq);
    }

    public float[] calculateMean() {

        float[] mean_beacon_pose = {0, 0};
        for (int i = 0; i < beaconPoseList_X.size(); i++)
        {
            mean_beacon_pose[0] += beaconPoseList_X.get(i);
            mean_beacon_pose[1] += beaconPoseList_Y.get(i);
        }
        mean_beacon_pose[0] /= beaconPoseList_X.size();
        mean_beacon_pose[1] /= beaconPoseList_X.size();

        return mean_beacon_pose;

    }

    private float[] calculateKalmanDistance(float[] DR_pose, float[] mean_beacon_pose) {

//        float errorCovarianceRssi;
        float DRNoise = 1.25f;
        float beaconNoise = 0.005f;

        //prediction
        float[] pose_apriori = { (estimatedPose[0] + DR_pose[0]) , (estimatedPose[1] + DR_pose[1]) };
        float errorCovariance_apriori = errorCovariance + DRNoise;

        //updation
        float[] z_t = mean_beacon_pose;
        float kalmanGain = errorCovariance_apriori / (errorCovariance_apriori + beaconNoise);
        estimatedPose[0] = pose_apriori[0] + kalmanGain * (z_t[0] - pose_apriori[0]);
        estimatedPose[1] = pose_apriori[1] + kalmanGain * (z_t[1] - pose_apriori[1]);
        errorCovariance = (1 - kalmanGain) * errorCovariance_apriori;

//        //prediction
//        estimatedPose[0] += DR_pose[0];
//        estimatedPose[1] += DR_pose[1];
//        lastErrorCovariance += DRNoise;
//
//        //updation
//        float[] z_t = mean_beacon_pose;
//        float kalmanGain = lastErrorCovariance / (lastErrorCovariance + beaconNoise);
//        errorCovarianceRssi = (1 - kalmanGain) * lastErrorCovariance;
//        estimatedPose[0] += kalmanGain * (z_t[0] - estimatedPose[0]);
//        estimatedPose[1] += kalmanGain * (z_t[1] - estimatedPose[1]);

        return estimatedPose;

//        private static float calculateKalmanRssi(List<AdvertisingPacket> advertisingPackets,
//        float processNoise, float measurementNoise, float meanRssi) {
//            float errorCovarianceRssi;
//            float lastErrorCovarianceRssi = 1;
//            float estimatedRssi = meanRssi;
//            for (AdvertisingPacket advertisingPacket : advertisingPackets) {
//                float kalmanGain = lastErrorCovarianceRssi / (lastErrorCovarianceRssi + measurementNoise);
//                estimatedRssi = estimatedRssi + (kalmanGain * (advertisingPacket.getRssi() - estimatedRssi));
//                errorCovarianceRssi = (1 - kalmanGain) * lastErrorCovarianceRssi;
//                lastErrorCovarianceRssi = errorCovarianceRssi + processNoise;
//            }
//            return estimatedRssi;
//        }


    }
}