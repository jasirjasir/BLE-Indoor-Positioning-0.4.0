package com.nexenio.bleindoorpositioningdemo;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Build;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.os.SystemClock;
import android.util.Log;

import com.nexenio.bleindoorpositioningdemo.ui.beaconview.pathplanning.DynamicStepCounter;

import java.util.ArrayList;

/**
 * @hide This class registers two sensor listeners for accelerometer and gyroscope to the device
 * {@link SensorManager} and broadcasts all received SensorEvent to registered listeners.
 * <p>This class launches its own thread when {@link #start()} is called.
 */
public class DeviceSensorLooper {

    private static final String LOG_TAG = DeviceSensorLooper.class.getSimpleName();

    /**
     * Is the inner looper thread started.
     */
    private boolean isRunning;

    /**
     * Sensor manager used to register and unregister listeners.
     */
    private SensorManager sensorManager;

    /**
     * Looper thread that listen to SensorEvent
     */
    private Looper sensorLooper;

    /**
     * Sensor event listener for the internal sensors event.
     */
    private SensorEventListener sensorEventListener;

    /**
     * List of registered listeners see .
     */
    private final ArrayList<SensorEventListener> registeredListeners =
            new ArrayList<SensorEventListener>();

    private boolean firstRun = true;
    private boolean firstBeaconPoseReceived = false;

    ArrayList<Integer> beaconPoseList_X = new ArrayList<Integer>();
    ArrayList<Integer> beaconPoseList_Y = new ArrayList<Integer>();

    private double heading;
    private DynamicStepCounter dynamicStepCounter;
    private float[] estimatedPose = new float[2];
    private long startTime, eventStartTime, mTempStartTime, beaconAvgTime, sensorStartTime;

    private int mStepCount, mTempStepCount;

    private final float[] accelerometerReading = new float[3];
    private final float[] magnetometerReading = new float[3];

    private final float[] rotationMatrix = new float[9];
    private final float[] linAccReading = new float[3];
    private static final float STRIDE_LENGTH = 50.0f;   //74.0f;

    private float[] DRPose = new float[2];
    private boolean updatePosition = false;
    private long positionUpdateTime;
    private float errorCovariance;

    private double mTempSensitivity = 0.5;
    private int THRESHOLD_STEPS = 10;

    private double mSensitivity = 0.8;

    private SensorDataListener mSensorDataListener;

    /**
     * Default constructor.
     *
     * @param vContext Android context to create SensorManager
     */
    public DeviceSensorLooper(Context vContext, SensorDataListener vSensorDataListener) {

        sensorManager = (SensorManager) vContext.getSystemService(Context.SENSOR_SERVICE);

        mSensorDataListener = vSensorDataListener;

        dynamicStepCounter = new DynamicStepCounter(mSensitivity);
        heading = 0;

        estimatedPose[0] = 0;  //100; //600;  //2200;  //1740 ; //0.0f;
        estimatedPose[1] = 0;  //290; //1385;  //660;  //0.0f;
        errorCovariance = 0.0f;

        positionUpdateTime = SystemClock.elapsedRealtime();
        sensorStartTime = SystemClock.elapsedRealtime();
        registerSensors();
    }

    private Sensor getUncalibratedGyro() {
        // Don't use uncalibrated gyro on HTC phones as it can make the phones unusable.
        // See b/21444644 for details.
        if (Build.MANUFACTURER.equals("HTC")) {
            return null;
        }
        return sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED);
    }

    /**
     * This registers two {@link SensorEventListener} and start a Looper.
     */
    public void start() {
        if (isRunning) {
            // The looper is already started nothing needs to be done.
            return;
        }

        sensorEventListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent sensorEvent) {
                // Pass the event to all the listeners.
                if (firstRun) {
                    Log.d("onSensorChanged", "firstRun: " + firstRun + " beaconAvgTime: " + beaconAvgTime);
                    if (((SystemClock.elapsedRealtime() - beaconAvgTime) > 3000) && firstBeaconPoseReceived) {
                        firstRun = false;
                        eventStartTime = sensorEvent.timestamp;
                        estimatedPose = calculateMean();
                        Log.d("onSensorChanged", "firstRun: " + firstRun
                                + " (" + estimatedPose[0] + ", " + estimatedPose[1]);
                    } else
                        return;
                }
                //Restart sensors in every 10 sec to reduce the ingration error
                if ((SystemClock.elapsedRealtime() - sensorStartTime) > 10000) {
                    sensorStartTime = SystemClock.elapsedRealtime();
                    registerSensors();
                }
                if ((SystemClock.elapsedRealtime() - mTempStartTime) > 10000) {
                    mTempStartTime = SystemClock.elapsedRealtime();
                    //isCheck = false;
                    mTempStepCount = 0;
                }
                if (sensorEvent.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
                    System.arraycopy(sensorEvent.values, 0, magnetometerReading, 0, magnetometerReading.length);
                    SensorManager.getRotationMatrix(rotationMatrix, null, accelerometerReading, magnetometerReading);
                    heading = Math.atan2(rotationMatrix[3], rotationMatrix[0]);
                } else if (sensorEvent.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
                    System.arraycopy(sensorEvent.values, 0, linAccReading, 0, linAccReading.length);
                    float norm = calcNorm(sensorEvent.values[0] + sensorEvent.values[1] + sensorEvent.values[2]);
                    boolean stepFound = dynamicStepCounter.findStep(norm, (sensorEvent.timestamp - eventStartTime));

                    if (stepFound) {
                        mTempStepCount++;
                        mTempStartTime = SystemClock.elapsedRealtime();
//                        if (dynamicStepCounter.getSensitivity() == mSensitivity) {
                        float rPointX, rPointY;
                        mStepCount++;
                        Log.d("onSensorChanged ", " estimatedPose: (" + estimatedPose[0] + ", " + estimatedPose[1] + ")");
                        rPointX = (float) (estimatedPose[0] - STRIDE_LENGTH * Math.sin(heading));
                        rPointY = (float) (estimatedPose[1] + STRIDE_LENGTH * Math.cos(heading));
                        DRPose[0] = rPointX;
                        DRPose[1] = rPointY;
                        updatePosition = true;
                        positionUpdateTime = SystemClock.elapsedRealtime();
                    } else {
                        DRPose = estimatedPose;
                    }
                }
                mSensorDataListener.onSensorChange(updatePosition, DRPose);

                /*synchronized (registeredListeners) {
                    for (SensorEventListener listener : registeredListeners) {
                        listener.onSensorChanged(sensorEvent);
                    }
                }*/

            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int accuracy) {
                /*synchronized (registeredListeners) {
                    for (SensorEventListener listener : registeredListeners) {
                        listener.onAccuracyChanged(sensor, accuracy);
                    }
                }*/
            }
        };

        HandlerThread sensorThread = new HandlerThread("sensor") {
            @Override
            protected void onLooperPrepared() {
                Handler handler = new Handler(Looper.myLooper());

                // Initialize the accelerometer.
                Sensor accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
                sensorManager.registerListener(sensorEventListener, accelerometer,
                        SensorManager.SENSOR_DELAY_GAME, handler);

                // Initialize the gyroscope.
                // If it's available, prefer to use the uncalibrated gyroscope sensor.
                // The regular gyroscope sensor is calibrated with a bias offset in the system. As we cannot
                // influence the behavior of this algorithm and it will affect the gyro while moving,
                // it is safer to initialize to the uncalibrated one and handle the gyro bias estimation
                // ourselves in a way which is optimized for our use case.
                Sensor gyroscope = getUncalibratedGyro();
                if (gyroscope == null) {
                    Log.i(LOG_TAG, "Uncalibrated gyroscope unavailable, default to regular gyroscope.");
                    gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
                }

                sensorManager.registerListener(sensorEventListener, gyroscope, SensorManager.SENSOR_DELAY_GAME, handler);

                //init the magnetometer so we can point north properly
                Sensor magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
                sensorManager.registerListener(sensorEventListener, magnetometer, SensorManager.SENSOR_DELAY_GAME, handler);
            }
        };

        sensorThread.start();
        sensorLooper = sensorThread.getLooper();  // Blocks till looper is ready.
        isRunning = true;
    }

    /**
     * Stops the looper and deregister the listener from the sensor manager.
     */

    public void stop() {
        if (!isRunning) {
            // No need to stop.
            return;
        }

        //sensorManager.unregisterListener(sensorEventListener);
        //sensorEventListener = null;

        sensorLooper.quit();
        sensorLooper = null;
        isRunning = false;
    }

    public void firstBeaconPoseReceived(){
        firstBeaconPoseReceived = true;
    }


    /*public void registerListener(SensorEventListener listener) {
        synchronized (registeredListeners) {
            registeredListeners.add(listener);
        }
    }


    public void unregisterListener(SensorEventListener listener) {
        synchronized (registeredListeners) {
            registeredListeners.remove(listener);
        }
    }*/

    public static float calcNorm(double... args) {
        double sumSq = 0;
        for (double arg : args)
            sumSq += Math.pow(arg, 2);
        return (float) Math.sqrt(sumSq);
    }

    public float[] calculateMean() {
        float[] mean_beacon_pose = {0, 0};
        for (int i = 0; i < beaconPoseList_X.size(); i++) {
            mean_beacon_pose[0] += beaconPoseList_X.get(i);
            mean_beacon_pose[1] += beaconPoseList_Y.get(i);
        }
        mean_beacon_pose[0] /= beaconPoseList_X.size();
        mean_beacon_pose[1] /= beaconPoseList_X.size();
        return mean_beacon_pose;
    }

    private void registerSensors() {
        Sensor accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        if (accelerometer != null) {
            sensorManager.registerListener(sensorEventListener, accelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        }
        Sensor magneticField = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        if (magneticField != null) {
            sensorManager.registerListener(sensorEventListener, magneticField, SensorManager.SENSOR_DELAY_FASTEST);
        }
        Sensor linearAccelaration = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        if (linearAccelaration != null) {
            sensorManager.registerListener(sensorEventListener, linearAccelaration, SensorManager.SENSOR_DELAY_FASTEST);
        }
    }
}
