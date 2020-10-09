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
import com.nexenio.bleindoorpositioningdemo.R;
import com.nexenio.bleindoorpositioningdemo.bluetooth.BeaconLoc;
import com.nexenio.bleindoorpositioningdemo.bluetooth.BeaconStore;
import com.nexenio.bleindoorpositioningdemo.bluetooth.BluetoothClient;
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

public class BeaconNavigateFragment extends BeaconViewFragment  {
    public static String LOGTAG="beacon";
    public static String beacon_data="";
    StringBuffer sb=new StringBuffer();
    Bitmap myBitmap,tempBitmap,path_bitmap;
    Canvas tempCanvas,path_canvas;
    Paint paint =new Paint();
    Paint paintOut = new Paint();
    ImageView mImageView;
    private SensorEventListener sensorEventListener;
    private SensorManager sensorManager;
    private final float[] accelerometerReading = new float[3];
    private final float[] magnetometerReading = new float[3];
    private final float[] gyroscopeReading = new float[3];
   // String csv_filename;
  //  CSVWriter writer = null;
    List<String[]> beacon_data_list;
    ArrayList<String> list;
    String start_loc,stop_loc;
    String[] start_locations=null,stop_locations=null;
    AutoCompleteTextView start_act,stop_act;
    HashMap<String,int[]> hm_location=new HashMap<String, int[]>();
    BitmapFactory.Options options = new BitmapFactory.Options();

    Bitmap bmp_mono = null;
    Mat srcMat = new Mat();
    int [] input_cordinates=new int[4];
    Bitmap copy_bmp=null;
    ArrayList<Integer> x_list=new ArrayList<Integer>();
    ArrayList<Integer> y_list=new ArrayList<Integer>();
    double[] mean_coordinates =new double[2];
    ArrayList<double[][]> locations =new ArrayList<double[][]>();
    ArrayList<Double> distances =new ArrayList<Double>();
    int [] output_coordinates=null;
    private double[] localized_position =null;
  //  double[][] coordinate_list=new double[][]{{570,304},{114,101},{115,325},{782,406},{460,73},{292,625}};
    double[][] coordinate_list;
    double[][] digkistra_coordinates=null;
    TextView tv_reset;
    CheckBox beacon_show;
   // Indu chechi changes
   private double heading;
   private DynamicStepCounter dynamicStepCounter;
    private float[] estimatedPose = new float[2];
    private float errorCovariance;
    private long positionUpdateTime;
    private long sensorStartTime;
    private final float[] rotationMatrix = new float[9];
    private final float[] linAccReading = new float[3];
    private static final float STRIDE_LENGTH = 74.0f;
    private float[] DRPose = new float[2];
    private boolean updatePosition = false;
    ArrayList<Integer> beaconPoseList_X = new ArrayList<Integer>();
    ArrayList<Integer> beaconPoseList_Y=new ArrayList<Integer>();
    public BeaconNavigateFragment() {
        super();
        //uncomment to add uuid filter
        //beaconFilters.add(uuidFilter);

        sensorEventListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent sensorEvent) {
               // Log.d(LOGTAG,"Onsensor changed");

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
               //  Log.d("onSensorChanged ", " sensor:M " + sensorEvent.sensor.getType() + ", heading: " + heading);

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
                   // Log.d("Norm ", ""+norm);
                    boolean stepFound = dynamicStepCounter.findStep(norm);
                    //Log.d("step found ", ""+stepFound);
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
                    //Log.d("onSensorChanged", "Time elapsed....");
                    positionUpdateTime = SystemClock.elapsedRealtime();
                }
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

                   // Log.d("onSensorChanged ", " DR_pose: (" + DRPose[0] + ", " + DRPose[1] + "), beacon_pose: (" + meanBeaconPose[0] + ", " + meanBeaconPose[1]
                            //+ "), fusion_pose: (" + estimatedPose[0] + ", " + estimatedPose[1] + ")" );

//            Bundle bundle = new Bundle();
//            bundle.putFloatArray("fusion_pose", estimatedPose);
//            BeaconNavigateFragment beaconNavigateFragment = new BeaconNavigateFragment();
//            beaconNavigateFragment.setArguments(bundle);
//            getFragmentManager().beginTransaction().commit();
                }
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int i) {

            }
        };
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


    @Override
    public void onAttach(Context context) {
        super.onAttach(context);
        sensorManager = (SensorManager) getActivity().getSystemService(Context.SENSOR_SERVICE);

        dynamicStepCounter = new DynamicStepCounter(1.0);
        heading = 0;
        estimatedPose[0] = 0;
        estimatedPose[1] = 0;
        errorCovariance = 0;

        positionUpdateTime = SystemClock.elapsedRealtime();
        sensorStartTime = SystemClock.elapsedRealtime();
        Sensor accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        if (accelerometer != null) {
            sensorManager.registerListener (sensorEventListener, accelerometer, SensorManager.SENSOR_DELAY_FASTEST);
        }
        Sensor magneticField = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        if (magneticField != null) {
            sensorManager.registerListener(sensorEventListener, magneticField, SensorManager.SENSOR_DELAY_FASTEST);
        }
        Sensor linearAccelaration = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        if (linearAccelaration != null) {
            sensorManager.registerListener(sensorEventListener, linearAccelaration, SensorManager.SENSOR_DELAY_FASTEST);
        }
      //  sensorManager.registerListener(sensorEventListener, sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_NORMAL);
     //   sensorManager.registerListener(sensorEventListener, sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE), SensorManager.SENSOR_DELAY_NORMAL);
      //  sensorManager.registerListener(sensorEventListener, sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_NORMAL);

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
                sb=new StringBuffer();
                int beacon_count=0;
                List<Beacon> beaconlist=getBeacons();
               // sb.append(beaconlist.size());
                // add to csv
                beacon_data_list = new ArrayList<String[]>();
                list = new ArrayList<String>();
                for (Beacon beacon : beaconlist){
                    if (beacon.getMinorId()!=0 && beacon.getRssi()>-90 ) {
                        //github
                       // Log.d(LOGTAG,"name= "+beacon.getName()+" -> distance= "+beacon.getDistance()*100);
                        beacon_count++;
                        //sb.append("~NAME:" + beacon.getName());
                        sb.append("~MINOR_ID:" + beacon.getMinorId());
                        sb.append("~RSSI:" + beacon.getRssi());
                        sb.append("~DISTANCE:" + beacon.getDistance()*100*.8);


                        //addded for multilateration
                        locations.add(new double[][]{{Double.valueOf(beacon.getX()), Double.valueOf(beacon.getY())}});
                        distances.add((Math.sqrt(Math.abs(Math.pow(beacon.getDistance()*100,2)-Math.pow(1.5*100,2)))));
                        //distances.add((beacon.getDistance()*100*.9));
                    }
                   // Log.d(LOGTAG,"distance ="+beacon.getDistance());
                }
                sb.insert(0, String.valueOf(beacon_count));
               // Log.d(LOGTAG,sb.toString());
                //Multilateration
               if (locations.size()>=3){
                    double[][] pos_arr=new double[locations.size()][2];
                    double[] dis_arr= new double[distances.size()];
                    for (int i =0; i < locations.size(); i++) {
                        pos_arr[i][0] = locations.get(i)[0][0];
                        pos_arr[i][1] = locations.get(i)[0][1];
                    }
                    for (int i =0; i < distances.size(); i++)
                        dis_arr[i]=distances.get(i);

                   // Log.d("beacon","pos_arr="+pos_arr.length);
                  //  Log.d("beacon","dis_arr="+dis_arr.length);
                    LeastSquaresOptimizer.Optimum optimum = Multilateration.findOptimum(pos_arr, dis_arr);
                    double[] result_pos = optimum.getPoint().toArray();

                    addTolist(result_pos);
                    if (x_list.size()>=5) {
                        localized_position =calculateAverageVal();
                        if(output_coordinates!=null){
                            double[] result= findNearestPixel(result_pos,digkistra_coordinates);
                            if (result[2]<200.0) drawPoint(3 * (int) result[0], 3 * (int) result[1]);
                            else   drawPoint(3 * (int) localized_position[0], 3 * (int) localized_position[1]);

                        }else
                            drawPoint(3 * (int) localized_position[0], 3 * (int) localized_position[1]);

                        x_list.clear();
                        y_list.clear();
                    }

                }
                locations.clear();
                distances.clear();

       }
        };
    }
    public void addTolist(double []array){

        x_list.add((int) array[0]);
        y_list.add((int) array[1]);
    }
    public double[] calculateAverageVal(){
        int[] arr = new int[x_list.size()];
        for (int i =0; i < x_list.size(); i++)
            arr[i] = x_list.get(i);
        mean_coordinates[0]=AdvertisingPacketUtil.calculateMean(arr);
        arr = new int[x_list.size()];
        for (int i =0; i < y_list.size(); i++)
            arr[i] = y_list.get(i);
        mean_coordinates[1]=AdvertisingPacketUtil.calculateMean(arr);
        return mean_coordinates;
    }


    @Override
    protected int getLayoutResourceId() {
        //Create a new image bitmap and attach a brand new canvas to it

        Log.d("beacon","sending ui ");
        return R.layout.fragment_beacon_navigate;

    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        Log.d("beacon","-----view created--------- ");
        myBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.gadgeon_sec);
        options.inScaled = false;
        bmp_mono=BitmapFactory.decodeResource(getResources(), R.drawable.gadgeon_sec_mono, options);
        paint.setStyle(Paint.Style.FILL);
        paint.setColor(getResources().getColor(R.color.md_red_900));
        paint.setAntiAlias(true);
        paintOut.setStyle(Paint.Style.FILL);
        paintOut.setColor(getResources().getColor(R.color.md_red_A700_trans));
        paintOut.setAntiAlias(true);
        mImageView=(ImageView)getView().findViewById(R.id.plan);
        tv_reset=(TextView)getView().findViewById(R.id.reset_path);
        beacon_show=(CheckBox) getView().findViewById(R.id.beacon_show);
        Log.d("beacon","view created ");
        tempBitmap = Bitmap.createBitmap(myBitmap.getWidth(), myBitmap.getHeight(), Bitmap.Config.RGB_565);
        path_bitmap = Bitmap.createBitmap(myBitmap.getWidth(), myBitmap.getHeight(), Bitmap.Config.RGB_565);
        tempCanvas= new Canvas(tempBitmap);
        path_canvas= new Canvas(path_bitmap);
        tempCanvas.drawBitmap(myBitmap, 0, 0, null);
        //tempCanvas1.drawBitmap(myBitmap, 0, 0, null);
        ImageView Go_button= (ImageView)getView().findViewById(R.id.go_button);
        start_act = (AutoCompleteTextView) getView().findViewById(R.id.start_act);
        stop_act = (AutoCompleteTextView)getView().findViewById(R.id.stop_act);
        SetSpinnerData(); //set spinner data
        Go_button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                start_loc = start_act.getText().toString();
                stop_loc = stop_act.getText().toString();
                if ((start_loc.trim().equalsIgnoreCase("Start")||stop_loc.trim().equalsIgnoreCase("Stop")||(start_loc.trim().equalsIgnoreCase(stop_loc.trim())))) {
                    Toast.makeText(getActivity().getApplicationContext(), "Please choose correct locations !!", Toast.LENGTH_SHORT).show();

                }else if (start_loc.trim().equalsIgnoreCase("My location")) {
                   if (localized_position!=null){
                        double[] result=findNearestPixel(localized_position,coordinate_list);
                       input_cordinates=new int[]{(int) result[0],(int) result[1],hm_location.get(stop_loc)[0],hm_location.get(stop_loc)[1]};
                       Log.d("beacon","distance="+result[2]);
                       performNavigation(input_cordinates);
                   }else {
                       Toast.makeText(getActivity().getApplicationContext(), "Could not read your current location !!", Toast.LENGTH_SHORT).show();
                   }
                }else{
                    try {
                        input_cordinates=new int[]{hm_location.get(start_loc)[0],hm_location.get(start_loc)[1],hm_location.get(stop_loc)[0],hm_location.get(stop_loc)[1]};
                    } catch (NullPointerException e) {
                        Toast.makeText(getContext(), "Enter valid location", Toast.LENGTH_SHORT).show();
                        e.printStackTrace();
                    }
                    Log.d("beacon",LOGTAG+input_cordinates);
                    performNavigation(input_cordinates);
                }


            }
        });

       tv_reset.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Toast.makeText(getActivity().getApplicationContext(), "Path reset", Toast.LENGTH_SHORT).show();
               if(output_coordinates!=null) {
                    output_coordinates = null;
                    digkistra_coordinates = null;
                    copy_bmp = null;
                   // mImageView.setImageDrawable(new BitmapDrawable(getResources(), myBitmap));
                    mImageView.setImageResource(R.drawable.gadgeon_sec);
                }

            }
        });

       beacon_show.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
           @Override
           public void onCheckedChanged(CompoundButton compoundButton, boolean iscChecked) {
               if(iscChecked){
                   Toast.makeText(getActivity().getApplicationContext(), "Viewing beacon location", Toast.LENGTH_SHORT).show();
                   ArrayList<BeaconLoc> beaconlist= BeaconStore.getItemDataArrayList();
                   for (BeaconLoc beacon: beaconlist){
                       drawBeacon(3*beacon.getKx(),3*beacon.getKy(),beacon.getName());
                   }
               }
               else {
                   //Toast.makeText(getActivity().getApplicationContext(), "close Show beacon", Toast.LENGTH_SHORT).show();
                   mImageView.setImageResource(R.drawable.gadgeon_sec);
               }
           }
       });

    }

    public double[] findNearestPixel(double[] position, double[][] list){

        ArrayList<Double> distances= new ArrayList<>();
        for (double[] pos: list){
            distances.add(Math.sqrt(Math.pow((pos[0]-position[0]),2)+Math.pow((pos[1]-position[1]),2)));
        }
        int indexOfMinimum = distances.indexOf(Collections.min(distances));
        Log.d("beacon","nearest co-ordinate at ="+indexOfMinimum);
        Log.d("beacon","distance ="+distances.get(indexOfMinimum));

        return new double[] {list[indexOfMinimum][0],list[indexOfMinimum][1],distances.get(indexOfMinimum)};

    }

    private void performNavigation(int[] input_coordinates) {
        try {
            Utils.bitmapToMat(bmp_mono, srcMat);
            output_coordinates = CvUtil.processPathPlanning(srcMat, input_coordinates);
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

        }catch (Exception e){
            e.printStackTrace();
        }

    }

    private void SetSpinnerData() {
        getLocationFromJSON();
        ArrayAdapter<String> start_adapter = new ArrayAdapter<>( getActivity(), R.layout.custom_spinner,start_locations);
        ArrayAdapter<String> stop_adapter = new ArrayAdapter<>( getActivity(), R.layout.custom_spinner,stop_locations);
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
                InputMethodManager imm = (InputMethodManager)getContext().getSystemService(Context.INPUT_METHOD_SERVICE);
                imm.hideSoftInputFromWindow(start_act.getWindowToken(), 0);
            }
        });

        stop_act.setOnItemClickListener(new AdapterView.OnItemClickListener() {

            @Override
            public void onItemClick(AdapterView<?> parent, View arg1, int pos,
                                    long id) {
                InputMethodManager imm = (InputMethodManager)getContext().getSystemService(Context.INPUT_METHOD_SERVICE);
                imm.hideSoftInputFromWindow(stop_act.getWindowToken(), 0);
            }
        });


    }
    private void getLocationFromJSON() {
        try {
            JSONObject obj = new JSONObject(loadJSONFromAsset("cubicles.json"));
            JSONArray m_jArry = obj.getJSONArray("locations");
            start_locations=new String[m_jArry.length()+1];
            stop_locations=new String[m_jArry.length()];
            start_locations[0]="My location";
            coordinate_list=new double[m_jArry.length()][2];
            for (int i = 0; i < m_jArry.length(); i++) {
                JSONObject jo_inside = m_jArry.getJSONObject(i);
                Log.d("ID-->", "" + jo_inside.getString("Cubicle"));
                String cubicle = jo_inside.getString("Cubicle");
                int X = jo_inside.getInt("X");
                int Y = jo_inside.getInt("Y");

                //Add your values in your `ArrayList` as below:
                hm_location.put(cubicle,new int[]{X, Y});
                start_locations[i+1]=cubicle;
                stop_locations[i]=cubicle;
                coordinate_list[i][0]=X;
                coordinate_list[i][1]=Y;
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

    private void drawPoint(int x,int y) {
       // Log.d("beacon","image width is ="+myBitmap.getWidth());
        //Log.d("beacon","image width is ="+myBitmap.getHeight());
        if (x>myBitmap.getWidth()) x=myBitmap.getWidth()-100;
        if (x<0) x=0+100;
        if (y>myBitmap.getHeight()) y=myBitmap.getHeight()-100;
        if (y<0) y=0+100;


        //tempCanvas= new Canvas(tempBitmap);

        if (copy_bmp==null){ tempCanvas.drawBitmap(myBitmap, 0, 0, null);}

         else {
             tempCanvas.drawBitmap(copy_bmp, 0, 0, null);
         }

        tempCanvas.drawCircle(x ,y,50,paint);
        tempCanvas.drawCircle(x, y, 100, paintOut);

        mImageView.setImageDrawable(new BitmapDrawable(getResources(), tempBitmap));
    }
    private void drawBeacon(int x,int y,String beaconName) {
        paint.setTypeface(Typeface.create(Typeface.DEFAULT, Typeface.BOLD));
        paint.setColor(getResources().getColor(R.color.md_pink_900));
        paint.setTextSize(200);
        tempCanvas.drawText(beaconName.replace("beacon","B-"),x-120,y-110,paint);
        paint.setColor(getResources().getColor(R.color.primary_dark_trans));
        tempCanvas.drawRect(x-100 ,y-100,x+100,y+100,paint);

        mImageView.setImageDrawable(new BitmapDrawable(getResources(), tempBitmap));
    }
    private void drawRoute(float [] array) {


        path_canvas.drawBitmap(myBitmap, 0, 0, null);
        Paint custom_paint=new Paint();
        custom_paint.setColor(Color.GREEN);
        custom_paint.setStyle(Paint.Style.FILL);
        custom_paint.setStrokeWidth(30);
        paint.setAntiAlias(true);
       // tempCanvas.drawLines(array,custom_paint);
        try {
            for (int i = 0; i <= array.length-4; i += 2) {

                path_canvas.drawLine(array[i], array[i + 1], array[i + 2], array[i + 3], custom_paint);
            }
        }
        catch (Exception e){
            e.printStackTrace();
        }
        custom_paint.setColor(Color.BLUE);
        path_canvas.drawCircle(3*input_cordinates[0],3*input_cordinates[1],50,custom_paint);
        path_canvas.drawCircle(3*input_cordinates[2],3*input_cordinates[3],50,custom_paint);
        mImageView.setImageDrawable(new BitmapDrawable(getResources(), path_bitmap));
        copy_bmp=path_bitmap;

    }


}
