package com.nexenio.bleindoorpositioningdemo.ui.beaconview.pathplanning;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.graphics.drawable.BitmapDrawable;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.SystemClock;
import android.util.Log;
import android.view.View;
import android.view.inputmethod.InputMethodManager;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.AutoCompleteTextView;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.nexenio.bleindoorpositioning.ble.advertising.AdvertisingPacketUtil;
import com.nexenio.bleindoorpositioning.ble.beacon.Beacon;
import com.nexenio.bleindoorpositioning.ble.beacon.BeaconUpdateListener;
import com.nexenio.bleindoorpositioning.location.Location;
import com.nexenio.bleindoorpositioning.location.LocationListener;
import com.nexenio.bleindoorpositioning.location.multilateration.Multilateration;
import com.nexenio.bleindoorpositioning.location.provider.LocationProvider;
import com.nexenio.bleindoorpositioningdemo.DeviceSensorLooper;
import com.nexenio.bleindoorpositioningdemo.R;
import com.nexenio.bleindoorpositioningdemo.SensorDataListener;
import com.nexenio.bleindoorpositioningdemo.bluetooth.BeaconLoc;
import com.nexenio.bleindoorpositioningdemo.bluetooth.BeaconStore;
import com.nexenio.bleindoorpositioningdemo.ui.beaconview.BeaconViewFragment;
import com.nexenio.bleindoorpositioningdemo.ui.beaconview.CvUtil;

import org.apache.commons.math3.fitting.leastsquares.LeastSquaresOptimizer;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

public class BeaconNavigateFragment extends BeaconViewFragment implements SensorDataListener {


    Bitmap ImageBitmap, tempBitmap, path_bitmap, bmp_mono, copy_bmp;
    Canvas tempCanvas, path_canvas;

    Paint paint = new Paint();
    Paint paintOut = new Paint();
    Mat srcMat = new Mat();

    ImageView mImageView;
    AutoCompleteTextView start_act, stop_act;
    TextView tv_reset, tv_display, tv_debug1, tv_debug2;
    CheckBox beacon_show, debug_box;

    private SensorEventListener sensorEventListener;
    private SensorManager sensorManager;
    private final float[] accelerometerReading = new float[3];
    private final float[] magnetometerReading = new float[3];
    private final float[] gyroscopeReading = new float[3];

    List<String[]> beacon_data_list;
    String start_loc, stop_loc;
    String[] start_locations = null, stop_locations = null;

    HashMap<String, int[]> hm_location = new HashMap<String, int[]>();
    BitmapFactory.Options options = new BitmapFactory.Options();


    double[] mean_coordinates = new double[2];
    ArrayList<Integer> x_list = new ArrayList<Integer>();
    ArrayList<Integer> y_list = new ArrayList<Integer>();
    ArrayList<String> list;
    ArrayList<double[][]> locations = new ArrayList<double[][]>();
    ArrayList<Double> distances = new ArrayList<Double>();

    int[] input_cordinates = new int[4];
    int[] output_coordinates = null;
    private double[] localized_position = null;
    double[][] coordinate_list;
    double[][] digkistra_coordinates = null;

    // Indu chechi changes
    private double heading;
    private DynamicStepCounter dynamicStepCounter;
    private float[] estimatedPose = new float[2];
    private float errorCovariance;
    private long positionUpdateTime;
    private long sensorStartTime;
    private final float[] rotationMatrix = new float[9];
    private final float[] linAccReading = new float[3];
    private static final float STRIDE_LENGTH = 50.0f;   //74.0f;
    private static final int INITIAL_STEP_COUNT = 4;
    private float[] DRPose = new float[2];
    private boolean updatePosition = false;
    ArrayList<Integer> beaconPoseList_X = new ArrayList<Integer>();
    ArrayList<Integer> beaconPoseList_Y = new ArrayList<Integer>();
    ArrayList<Long> beaconTime = new ArrayList<Long>();

    private int minimum_beacon_count = 3;
    private int scaling_factor = 3;
    private int[] corridor_boundary = new int[]{840, 880, 1318, 2};

    private boolean isNear = false;
    private boolean isDest = false;

    private float[] expectedPose = {0.0f, 0.0f};
    private int mStepCount, mTempStepCount;
    private double mTempSensitivity = 0.5;
    private int THRESHOLD_STEPS = 10;

    private double mSensitivity = 0.8;
    private Boolean isCheck = false;
    private boolean firstRun = true;
    private boolean firstBeaconPoseReceived = false;

    private long startTime, eventStartTime, mTempStartTime, beaconAvgTime;
    public static String LOGTAG = "beacon";
    StringBuffer sb = new StringBuffer();

    private static int plan_height = 0;
    private static int plan_width = 0;

    private DeviceSensorLooper mDeviceSensorLooper;

    @Override
    public void onDestroy() {
        super.onDestroy();
        mDeviceSensorLooper.stop();
    }

    public BeaconNavigateFragment() {
        super();

        //mDeviceSensorLooper.start();

        /*sensorEventListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent sensorEvent) {
                // Log.d(LOGTAG,"Onsensor changed");
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
                    isCheck = false;
                    mTempStepCount = 0;
                }
               if (sensorEvent.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
                    System.arraycopy(sensorEvent.values, 0, magnetometerReading, 0, magnetometerReading.length);
                    SensorManager.getRotationMatrix(rotationMatrix, null, accelerometerReading, magnetometerReading);
                    heading = Math.atan2(rotationMatrix[3], rotationMatrix[0]);
                } else if (sensorEvent.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
                    System.arraycopy(sensorEvent.values, 0, linAccReading, 0, linAccReading.length);
                    tv_display.setText("Sensitivity : " + dynamicStepCounter.getSensitivity() + " \n" + "Steps : " + mStepCount);
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
                if ((!updatePosition) && ((SystemClock.elapsedRealtime() - positionUpdateTime) > 5000)) {
                    updatePosition = true;
                    positionUpdateTime = SystemClock.elapsedRealtime();
                }
                if (updatePosition) {
                    updatePosition = false;
                    float[] meanBeaconPose = new float[2];
                    if (beaconPoseList_X.size() != 0) {
                        meanBeaconPose = calculateMean();
                        calculateKalmanDistance(DRPose, meanBeaconPose);
                    } else {
                        estimatedPose = DRPose;
                    }
                    Log.d("onSensorChanged ", " DR_pose: (" + DRPose[0] + ", " + DRPose[1] + "), beacon_pose: (" + meanBeaconPose[0] + ", " + meanBeaconPose[1]
                            + "), fusion_pose: (" + estimatedPose[0] + ", " + estimatedPose[1] + ")");
                    tv_debug2.setText("Estimated POS  X =" + (int) estimatedPose[0] + " Y= " + (int) estimatedPose[1]);

                    //drawLocation_sensor(scaling_factor*(int)estimatedPose[0],scaling_factor*(int)estimatedPose[1]);
                    drawLocation_sensor(scaling_factor * (2490 - (int) estimatedPose[1]), scaling_factor * (1477 - (int) estimatedPose[0]));
                }

            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int i) {

            }
        };*/
    }

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

    private float[] calculateKalmanDistance(float[] DR_pose, float[] mean_beacon_pose) {

//        float errorCovarianceRssi;
        float DRNoise = 1.0f;
        float beaconNoise = 0.001f;

        //prediction
        float[] pose_apriori = {(estimatedPose[0] + DR_pose[0]), (estimatedPose[1] + DR_pose[1])};
        float errorCovariance_apriori = errorCovariance + DRNoise;

        //updation
        float[] z_t = mean_beacon_pose;
        float kalmanGain = errorCovariance_apriori / (errorCovariance_apriori + beaconNoise);
        Log.d("onSensorChanged", "ERRORcOV: " + errorCovariance + ", " + errorCovariance_apriori
                + " KGain: " + kalmanGain);
        estimatedPose[0] = pose_apriori[0] + kalmanGain * (z_t[0] - pose_apriori[0]);
        estimatedPose[1] = pose_apriori[1] + kalmanGain * (z_t[1] - pose_apriori[1]);
        errorCovariance = (1 - kalmanGain) * errorCovariance_apriori;

        return estimatedPose;

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

    @Override
    public void onAttach(Context context) {
        super.onAttach(context);

        mDeviceSensorLooper = new DeviceSensorLooper(context, this);

        mDeviceSensorLooper.start();
        /*sensorManager = (SensorManager) getActivity().getSystemService(Context.SENSOR_SERVICE);

        dynamicStepCounter = new DynamicStepCounter(mSensitivity);
        heading = 0;

        estimatedPose[0] = 0;  //100; //600;  //2200;  //1740 ; //0.0f;
        estimatedPose[1] = 0;  //290; //1385;  //660;  //0.0f;
        errorCovariance = 0.0f;

        positionUpdateTime = SystemClock.elapsedRealtime();
        sensorStartTime = SystemClock.elapsedRealtime();
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
        }*/
    }

    @Override
    protected LocationListener createDeviceLocationListener() {
        return new LocationListener() {
            @Override
            public void onLocationUpdated(LocationProvider locationProvider, Location location) {
                // TODO: remove artificial noise

            }
        };
    }

    @Override
    protected BeaconUpdateListener createBeaconUpdateListener() {
        return new BeaconUpdateListener() {
            @Override
            public void onBeaconUpdated(Beacon updatedBeacon) {
                sb = new StringBuffer();
                int beacon_count = 0;
                List<Beacon> beaconlist = getBeacons();
                // sb.append(beaconlist.size());
                // add to csv
                beacon_data_list = new ArrayList<String[]>();
                list = new ArrayList<String>();
                for (Beacon beacon : beaconlist) {
                    if (beacon.getMinorId() != 0 && beacon.getRssi() > -85) {
                        //github
                        // Log.d(LOGTAG,"name= "+beacon.getName()+" -> distance= "+beacon.getDistance()*100);
                        beacon_count++;
                        //sb.append("NAME:" + beacon.getName());
                        // sb.append("~MINOR_ID:" + beacon.getMinorId());
                        //  sb.append("~RSSI:" + beacon.getRssi());
                        //sb.append("~DISTANCE:" + beacon.getDistance()*100*.8);


                        //added for multilateration
                        locations.add(new double[][]{{Double.valueOf(beacon.getX()), Double.valueOf(beacon.getY())}});
                        //  double distanceToBeacon=Math.sqrt(Math.abs(Math.pow(beacon.getDistance()*100,2)-Math.pow(1.5*100,2)));
                        double distanceToBeacon = beacon.getDistance() * 100 * .9;
                        distances.add(distanceToBeacon);
                        // Log.d(LOGTAG,"name= "+beacon.getName()+" -> distance= "+distanceToBeacon);


                        sb.append(beacon.getName() + "-> ");
                        sb.append((int) beacon.getRssi() + " dB -> ");
                        sb.append((int) distanceToBeacon + " cm \n");
                    }
                    // Log.d(LOGTAG,"distance ="+beacon.getDistance());
                }
                //  tv_display.setText(sb.toString());
                sb.insert(0, String.valueOf(beacon_count));
                // Log.d(LOGTAG,sb.toString());
                //Multilateration
                if (locations.size() >= minimum_beacon_count) {
                    double[][] pos_arr = new double[locations.size()][2];
                    double[] dis_arr = new double[distances.size()];
                    for (int i = 0; i < locations.size(); i++) {
                        pos_arr[i][0] = locations.get(i)[0][0];
                        pos_arr[i][1] = locations.get(i)[0][1];
                    }
                    for (int i = 0; i < distances.size(); i++)
                        dis_arr[i] = distances.get(i);

                    // Log.d("beacon","pos_arr="+pos_arr.length);
                    //  Log.d("beacon","dis_arr="+dis_arr.length);
                    LeastSquaresOptimizer.Optimum optimum = Multilateration.findOptimum(pos_arr, dis_arr);
                    double[] result_pos = optimum.getPoint().toArray();

                    addTolist(result_pos);
                   /* if (x_list.size()>=5) {
                        localized_position =calculateAverageVal();
                        if(output_coordinates!=null){
                            double[] result= findNearestPixel(result_pos,digkistra_coordinates);
                            if (input_cordinates != null) {
                                double dis = disatanceToDestination(result_pos, new int[]{input_cordinates[2], input_cordinates[3]});
                                if (dis < 500 && !isNear) {
                                    Toast.makeText(getActivity().getApplicationContext(), "You are " + (int)(dis/100)*//*String.format("%.0f", dis / 100)*//* + " meter away from the destination !!", Toast.LENGTH_SHORT).show();
                                    isNear = true;
                                }
                                else if(dis<250 && !isDest){
                                    drawLocation(scaling_factor*input_cordinates[2], scaling_factor*input_cordinates[3]);
                                    Toast.makeText(getActivity().getApplicationContext(), "You have reached the destination", Toast.LENGTH_LONG).show();
                                    isDest=true;
                                }
                            }
                            if (!isDest) {
                                if (result[2] < 200.0)
                                    drawLocation(scaling_factor * (int) result[0], scaling_factor * (int) result[1]);
                                else
                                    drawLocation(scaling_factor * (int) localized_position[0], scaling_factor * (int) localized_position[1]);
                            }
                        }else
                            drawLocation(scaling_factor * (int) localized_position[0], scaling_factor* (int) localized_position[1]);

                        x_list.clear();
                        y_list.clear();
                    }
*/
                }
//               else if(locations.size()==1) {
//                   if(distances.get(0)<100)
//                         drawLocation(scaling_factor * (int) locations.get(0)[0][0], scaling_factor* (int) locations.get(0)[0][1]);
//               }
                locations.clear();
                distances.clear();

            }
        };
    }

    public void addTolist(double[] array) {
        if (!firstBeaconPoseReceived) {
            firstBeaconPoseReceived = true;
            mDeviceSensorLooper.firstBeaconPoseReceived();
            beaconAvgTime = SystemClock.elapsedRealtime();

//            estimatedPose[0] = (float)array[0];
//            estimatedPose[1] = (float)array[1];
        }
        x_list.add((int) array[0]);
        y_list.add((int) array[1]);
        beaconPoseList_X.add((int) array[0]);
        beaconPoseList_Y.add((int) array[1]);
        beaconTime.add(SystemClock.elapsedRealtime());
        tv_debug1.setText("BLE predicted X =" + (int) array[0] + " Y= " + (int) array[1]);

        for (int i = beaconPoseList_X.size() - 1; i >= 0; i--) {
            if ((SystemClock.elapsedRealtime() - beaconTime.get(i)) > 3000) {
                beaconPoseList_X.remove(i);
                beaconPoseList_Y.remove(i);
                beaconTime.remove(i);
            }
        }
        //Log.d(LOGTAG,"predicted X ="+(int) array[0] +" Y= "+(int) array[1]);

    }

    public double[] calculateAverageVal() {
        int[] arr = new int[x_list.size()];
        for (int i = 0; i < x_list.size(); i++)
            arr[i] = x_list.get(i);
        mean_coordinates[0] = AdvertisingPacketUtil.calculateMean(arr);
        arr = new int[x_list.size()];
        for (int i = 0; i < y_list.size(); i++)
            arr[i] = y_list.get(i);
        mean_coordinates[1] = AdvertisingPacketUtil.calculateMean(arr);
        return mean_coordinates;
    }


    @Override
    protected int getLayoutResourceId() {
        //Create a new image bitmap and attach a brand new canvas to it

        Log.d("beacon", "sending ui ");
        return R.layout.fragment_beacon_navigate;

    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        Log.d("beacon", "-----view created--------- ");
        ImageBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.gadgeon_sec);


        options.inScaled = false;
        bmp_mono = BitmapFactory.decodeResource(getResources(), R.drawable.gadgeon_sec_mono, options);
        plan_height = bmp_mono.getHeight();
        plan_width = bmp_mono.getWidth();
        Log.d(LOGTAG, "height =" + plan_height + "width= " + plan_width);

        paint.setStyle(Paint.Style.FILL);
        paint.setColor(getResources().getColor(R.color.md_red_900));
        paint.setAntiAlias(true);

        paintOut.setStyle(Paint.Style.FILL);
        paintOut.setColor(getResources().getColor(R.color.md_red_A700_trans));
        paintOut.setAntiAlias(true);

        mImageView = (ImageView) getView().findViewById(R.id.plan);
        tv_reset = (TextView) getView().findViewById(R.id.reset_path);
        tv_display = (TextView) getView().findViewById(R.id.id_display);
        tv_debug1 = (TextView) getView().findViewById(R.id.id_debug1);
        tv_debug2 = (TextView) getView().findViewById(R.id.id_debug2);

        beacon_show = (CheckBox) getView().findViewById(R.id.beacon_show);
        debug_box = (CheckBox) getView().findViewById(R.id.debug);
        Log.d("beacon", "view created ");

        tempBitmap = Bitmap.createBitmap(ImageBitmap.getWidth(), ImageBitmap.getHeight(), Bitmap.Config.RGB_565);
        path_bitmap = Bitmap.createBitmap(ImageBitmap.getWidth(), ImageBitmap.getHeight(), Bitmap.Config.RGB_565);
        tempCanvas = new Canvas(tempBitmap);
        path_canvas = new Canvas(path_bitmap);
        tempCanvas.drawBitmap(ImageBitmap, 0, 0, null);

        //tempCanvas1.drawBitmap(ImageBitmap, 0, 0, null);
        ImageView Go_button = (ImageView) getView().findViewById(R.id.go_button);
        start_act = (AutoCompleteTextView) getView().findViewById(R.id.start_act);
        stop_act = (AutoCompleteTextView) getView().findViewById(R.id.stop_act);

        mStepCount = 0;
        mTempStepCount = 0;
        SetSpinnerData(); //set spinner data
        startTime = SystemClock.elapsedRealtime();
        mTempStartTime = SystemClock.elapsedRealtime();

        Go_button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                start_loc = start_act.getText().toString();
                stop_loc = stop_act.getText().toString();
                if ((start_loc.trim().equalsIgnoreCase("Start") || stop_loc.trim().equalsIgnoreCase("Stop") || (start_loc.trim().equalsIgnoreCase(stop_loc.trim())))) {
                    Toast.makeText(getActivity().getApplicationContext(), "Please choose correct locations !!", Toast.LENGTH_SHORT).show();

                } else if (start_loc.trim().equalsIgnoreCase("My location")) {
                    if (localized_position != null) {
                        double[] result = findNearestPixel(localized_position, coordinate_list);
                        input_cordinates = new int[]{(int) result[0], (int) result[1], hm_location.get(stop_loc)[0], hm_location.get(stop_loc)[1]};
                        Log.d("beacon", "distance=" + result[2]);
                        performNavigation(input_cordinates);
                    } else {
                        Toast.makeText(getActivity().getApplicationContext(), "Could not read your current location !!", Toast.LENGTH_SHORT).show();
                    }
                } else {
                    try {
                        input_cordinates = new int[]{hm_location.get(start_loc)[0], hm_location.get(start_loc)[1], hm_location.get(stop_loc)[0], hm_location.get(stop_loc)[1]};
                    } catch (NullPointerException e) {
                        Toast.makeText(getContext(), "Enter valid location", Toast.LENGTH_SHORT).show();
                        e.printStackTrace();
                    }
                    Log.d("beacon", LOGTAG + input_cordinates);
                    performNavigation(input_cordinates);
                }
                isNear = false;
                isDest = false;

            }
        });

        tv_reset.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Toast.makeText(getActivity().getApplicationContext(), "Path reset", Toast.LENGTH_SHORT).show();
                if (output_coordinates != null) {
                    output_coordinates = null;
                    digkistra_coordinates = null;
                    copy_bmp = null;
                    // mImageView.setImageDrawable(new BitmapDrawable(getResources(), ImageBitmap));
                    mImageView.setImageResource(R.drawable.gadgeon_sec);
                }

            }
        });

        beacon_show.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean iscChecked) {
                if (iscChecked) {
                    Toast.makeText(getActivity().getApplicationContext(), "Viewing beacon location", Toast.LENGTH_SHORT).show();
                    ArrayList<BeaconLoc> beaconlist = BeaconStore.getItemDataArrayList();
                    for (BeaconLoc beacon : beaconlist) {
                        drawBeacon(3 * (2490 - beacon.getKy()), 3 * (1485 - beacon.getKx()), beacon.getName());
                    }
                } else {
                    //Toast.makeText(getActivity().getApplicationContext(), "close Show beacon", Toast.LENGTH_SHORT).show();
                    mImageView.setImageResource(R.drawable.gadgeon_sec);
                }
            }
        });
        debug_box.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean iscChecked) {
                if (iscChecked) {
                    tv_display.setVisibility(View.VISIBLE);
                } else {
                    tv_display.setVisibility(View.GONE);

                }
            }
        });
        addTolist(new double []{1,2,3,4});
    }

    public double[] findNearestPixel(double[] position, double[][] list) {

        ArrayList<Double> distances = new ArrayList<>();
        for (double[] pos : list) {
            distances.add(Math.sqrt(Math.pow((pos[0] - position[0]), 2) + Math.pow((pos[1] - position[1]), 2)));
        }
        int indexOfMinimum = distances.indexOf(Collections.min(distances));
        Log.d("beacon", "nearest co-ordinate at =" + indexOfMinimum);
        Log.d("beacon", "distance =" + distances.get(indexOfMinimum));

        return new double[]{list[indexOfMinimum][0], list[indexOfMinimum][1], distances.get(indexOfMinimum)};

    }

    private void performNavigation(int[] input_coordinates) {
        try {
            //modified_coordinates co-ordinates are added for getting x,y w.r.t to plan-Indu chechi
            int[] modified_coordinates = {plan_width - input_coordinates[1], plan_height - input_coordinates[0],
                    plan_width - input_coordinates[3], plan_height - input_coordinates[2]};
            input_cordinates = modified_coordinates;
            Log.d(LOGTAG, "input coordinates=" + input_coordinates[0] + " -" + input_coordinates[1] + " -"
                    + input_coordinates[2] + " -" + input_coordinates[3]);

            Utils.bitmapToMat(bmp_mono, srcMat);
            output_coordinates = CvUtil.processPathPlanning(srcMat, modified_coordinates);
            Log.d(LOGTAG, "received coordinates with size=" + output_coordinates.length);
            digkistra_coordinates = new double[output_coordinates.length / 2][2];
            if (output_coordinates.length != 0) {

                final float[] arr = new float[output_coordinates.length];
                int index = 0;
                for (int i = 0; i < output_coordinates.length; i++) {
                    arr[index++] = 3 * output_coordinates[i];
                }
                drawRoute(arr);
                double distance_to_travel = (output_coordinates.length / 2) / 100.0;
                Toast.makeText(getActivity().getApplicationContext(), "Distance to travel= " + distance_to_travel + " meter", Toast.LENGTH_SHORT).show();
                for (int i = 0; i < (output_coordinates.length - 2); i += 2) {
                    digkistra_coordinates[i / 2][0] = output_coordinates[i];
                    digkistra_coordinates[i / 2][1] = output_coordinates[i + 1];
                }
            } else
                Toast.makeText(getActivity().getApplicationContext(), "Sorry. No root found !!", Toast.LENGTH_SHORT).show();

        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    private void SetSpinnerData() {
        getLocationFromJSON();
        ArrayAdapter<String> start_adapter = new ArrayAdapter<>(getActivity(), R.layout.custom_spinner, start_locations);
        ArrayAdapter<String> stop_adapter = new ArrayAdapter<>(getActivity(), R.layout.custom_spinner, stop_locations);
        start_adapter.setDropDownViewResource(R.layout.custom_spinner_dropdown);
        stop_adapter.setDropDownViewResource(R.layout.custom_spinner_dropdown);
        start_act.setThreshold(1);
        stop_act.setThreshold(1);
        start_act.setAdapter(start_adapter);
        stop_act.setAdapter(stop_adapter);

        start_act.setOnItemClickListener(new AdapterView.OnItemClickListener() {

            @Override
            public void onItemClick(AdapterView<?> parent, View arg1, int pos,
                                    long id) {
                InputMethodManager imm = (InputMethodManager) getContext().getSystemService(Context.INPUT_METHOD_SERVICE);
                imm.hideSoftInputFromWindow(start_act.getWindowToken(), 0);
            }
        });

        stop_act.setOnItemClickListener(new AdapterView.OnItemClickListener() {

            @Override
            public void onItemClick(AdapterView<?> parent, View arg1, int pos,
                                    long id) {
                InputMethodManager imm = (InputMethodManager) getContext().getSystemService(Context.INPUT_METHOD_SERVICE);
                imm.hideSoftInputFromWindow(stop_act.getWindowToken(), 0);
            }
        });


    }

    private void getLocationFromJSON() {
        try {
            JSONObject obj = new JSONObject(loadJSONFromAsset("cubicles_New.json"));
            JSONArray m_jArry = obj.getJSONArray("locations");
            start_locations = new String[m_jArry.length() + 1];
            stop_locations = new String[m_jArry.length()];
            start_locations[0] = "My location";
            coordinate_list = new double[m_jArry.length()][2];
            for (int i = 0; i < m_jArry.length(); i++) {
                JSONObject jo_inside = m_jArry.getJSONObject(i);
                Log.d("ID-->", "" + jo_inside.getString("Cubicle"));
                String cubicle = jo_inside.getString("Cubicle");
                int X = jo_inside.getInt("X");
                int Y = jo_inside.getInt("Y");

                //Add your values in your `ArrayList` as below:
                hm_location.put(cubicle, new int[]{X, Y});
                start_locations[i + 1] = cubicle;
                stop_locations[i] = cubicle;
                coordinate_list[i][0] = X;
                coordinate_list[i][1] = Y;
            }
        } catch (JSONException e) {
            e.printStackTrace();
        }
    }

    private String loadJSONFromAsset(String fileName) {
        String json = null;
        try {
            InputStream is = getActivity().getAssets().open(fileName);
            int size = is.available();
            byte[] buffer = new byte[size];
            is.read(buffer);
            is.close();
            json = new String(buffer, "UTF-8");
        } catch (IOException ex) {
            ex.printStackTrace();
            return null;
        }
        return json;
    }

    private void drawLocation(int x, int y) {

        if (x > ImageBitmap.getWidth()) x = ImageBitmap.getWidth() - 100;
        if (x < 0) x = 0 + 100;
        if (y > ImageBitmap.getHeight()) y = ImageBitmap.getHeight() - 100;
        if (y < 0) y = 0 + 100;
        //Added to remove prediction outside corridor
        if ((x > scaling_factor * corridor_boundary[0] && x < scaling_factor * corridor_boundary[2]
                && y < scaling_factor * corridor_boundary[1] && y > scaling_factor * corridor_boundary[3])) {
            x = scaling_factor * 1069;
            y = scaling_factor * 985;
        }
        //tempCanvas= new Canvas(tempBitmap);

        if (copy_bmp == null) {
            tempCanvas.drawBitmap(ImageBitmap, 0, 0, null);
        } else {
            tempCanvas.drawBitmap(copy_bmp, 0, 0, null);
        }

        tempCanvas.drawCircle(x, y, 50, paint);
        tempCanvas.drawCircle(x, y, 100, paintOut);

        mImageView.setImageDrawable(new BitmapDrawable(getResources(), tempBitmap));
    }

    private void drawLocation_sensor(int x, int y) {
        // Log.d("beacon","image width is ="+ImageBitmap.getWidth());
        //Log.d("beacon","image width is ="+ImageBitmap.getHeight());
        if (x > ImageBitmap.getWidth()) x = ImageBitmap.getWidth() - 100;
        if (x < 0) x = 0 + 100;
        if (y > ImageBitmap.getHeight()) y = ImageBitmap.getHeight() - 100;
        if (y < 0) y = 0 + 100;


        //tempCanvas= new Canvas(tempBitmap);

        if (copy_bmp == null) {
            tempCanvas.drawBitmap(ImageBitmap, 0, 0, null);
        } else {
            tempCanvas.drawBitmap(copy_bmp, 0, 0, null);
        }
        paint.setColor(Color.GREEN);
        tempCanvas.drawCircle(x, y, 50, paint);
        tempCanvas.drawCircle(x, y, 100, paintOut);

        mImageView.setImageDrawable(new BitmapDrawable(getResources(), tempBitmap));
    }

    private void drawBeacon(int x, int y, String beaconName) {
        paint.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        paint.setColor(getResources().getColor(R.color.md_pink_900));
        paint.setTextSize(200);
        tempCanvas.drawText(beaconName.replace("beacon", "B-"), x - 120, y - 110, paint);
        paint.setColor(getResources().getColor(R.color.primary_dark_trans));
        tempCanvas.drawRect(x - 100, y - 100, x + 100, y + 100, paint);

        mImageView.setImageDrawable(new BitmapDrawable(getResources(), tempBitmap));
    }

    private void drawRoute(float[] array) {


        path_canvas.drawBitmap(ImageBitmap, 0, 0, null);
        Paint custom_paint = new Paint();
        custom_paint.setColor(Color.GREEN);
        custom_paint.setStyle(Paint.Style.FILL);
        custom_paint.setStrokeWidth(30);
        paint.setAntiAlias(true);
        // tempCanvas.drawLines(array,custom_paint);
        try {
            for (int i = 0; i <= array.length - 4; i += 2) {

                path_canvas.drawLine(array[i], array[i + 1], array[i + 2], array[i + 3], custom_paint);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }


        custom_paint.setColor(Color.BLUE);
        //modified_coordinates co-ordinates are added for getting x,y w.r.t to plan-Indu chechi
        path_canvas.drawCircle(scaling_factor * input_cordinates[0], scaling_factor * input_cordinates[1], 50, custom_paint);
        path_canvas.drawCircle(scaling_factor * input_cordinates[2], scaling_factor * input_cordinates[3], 50, custom_paint);

        mImageView.setImageDrawable(new BitmapDrawable(getResources(), path_bitmap));
        copy_bmp = path_bitmap;

    }

    private double disatanceToDestination(double[] source, int[] destination) {
        return Math.sqrt(Math.pow((destination[0] - source[0]), 2) + Math.pow((destination[1] - source[1]), 2));
    }

    @Override
    public void onSensorChange(boolean updatePosition, float[] DRPose) {
        if ((!updatePosition) && ((SystemClock.elapsedRealtime() - positionUpdateTime) > 5000)) {
            updatePosition = true;
            positionUpdateTime = SystemClock.elapsedRealtime();
        }
        if (updatePosition) {
            updatePosition = false;
            float[] meanBeaconPose = new float[2];
            if (beaconPoseList_X.size() != 0) {
                meanBeaconPose = calculateMean();
                calculateKalmanDistance(DRPose, meanBeaconPose);
            } else {
                estimatedPose = DRPose;
            }
            Log.d("onSensorChanged ", " DR_pose: (" + DRPose[0] + ", " + DRPose[1] + "), beacon_pose: (" + meanBeaconPose[0] + ", " + meanBeaconPose[1]
                    + "), fusion_pose: (" + estimatedPose[0] + ", " + estimatedPose[1] + ")");
            tv_debug2.setText("Estimated POS  X =" + (int) estimatedPose[0] + " Y= " + (int) estimatedPose[1]);

            //drawLocation_sensor(scaling_factor*(int)estimatedPose[0],scaling_factor*(int)estimatedPose[1]);
            drawLocation_sensor(scaling_factor * (2490 - (int) estimatedPose[1]), scaling_factor * (1477 - (int) estimatedPose[0]));
        }
    }
}
