package com.nexenio.bleindoorpositioningdemo.ui.beaconview.pathplanning;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.drawable.BitmapDrawable;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ImageView;
import android.widget.Spinner;
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
import com.nexenio.bleindoorpositioningdemo.ui.beaconview.BeaconViewFragment;
import com.nexenio.bleindoorpositioningdemo.ui.beaconview.CvUtil;

import org.apache.commons.math3.fitting.leastsquares.LeastSquaresOptimizer;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

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
    ImageView mImageView;
    TextView tv_reset;
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
    Spinner start_spinner,stop_spinner;
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
    double[][] coordinate_list=new double[][]{{22,89},{187,327},{187,572},{573,215},{573,518}};
    double[][] digkistra_coordinates=null;
    boolean isNear=false;
    public BeaconNavigateFragment() {
        super();
        //uncomment to add uuid filter
        //beaconFilters.add(uuidFilter);

        sensorEventListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent sensorEvent) {
                switch (sensorEvent.sensor.getType()) {
                    case Sensor.TYPE_ACCELEROMETER: {
                        System.arraycopy(sensorEvent.values, 0, accelerometerReading, 0, accelerometerReading.length);
                        break;
                    }
                    case Sensor.TYPE_MAGNETIC_FIELD: {
                        System.arraycopy(sensorEvent.values, 0, magnetometerReading, 0, magnetometerReading.length);
                        break;
                    }
                    case Sensor.TYPE_GYROSCOPE: {
                        System.arraycopy(sensorEvent.values, 0, gyroscopeReading, 0, gyroscopeReading.length);
                        break;
                    }
                }

            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int i) {

            }
        };
    }
    @Override
    public void onAttach(Context context) {
        super.onAttach(context);
       //   sensorManager = (SensorManager) getActivity().getSystemService(Context.SENSOR_SERVICE);
       //   sensorManager.registerListener(sensorEventListener, sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_NORMAL);
      //    sensorManager.registerListener(sensorEventListener, sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE), SensorManager.SENSOR_DELAY_NORMAL);
      //    sensorManager.registerListener(sensorEventListener, sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_NORMAL);

    }

    @Override
    protected LocationListener createDeviceLocationListener() {
        return new LocationListener() {
            @Override
            public void onLocationUpdated(LocationProvider locationProvider, Location location) {
                // TODO: remove artificial noise
                //location.setLatitude(location.getLatitude() + Math.random() * 0.0002);
                //location.setLongitude(location.getLongitude() + Math.random() * 0.0002);
               // beaconChart.setDeviceLocation(location);
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
                        //Log.d(LOGTAG,"beacon name="+beacon.getName()+"   Distance = "+beacon.getDistance());

                        //addded for multilateration
                        locations.add(new double[][]{{Double.valueOf(beacon.getX()), Double.valueOf(beacon.getY())}});
                        distances.add((beacon.getDistance()*100*.9));
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
                    if (x_list.size()>=10) {
                        localized_position =calculateAverageVal();
                        Log.d("beacon", "localized_position X= "+(int)localized_position[0]+"   -Y= "+(int)localized_position[1]);
                        if(output_coordinates!=null){
                            Log.d("beacon", "output_coordinates not null");
                            double[] result= findNearestPixel(result_pos,digkistra_coordinates);
                            if (input_cordinates!=null && !isNear) {
                                double dis= DisatanceToDestination(result_pos,new int[]{input_cordinates[2],input_cordinates[3]});
                                if(dis<200) {
                                    Toast.makeText(getActivity().getApplicationContext(), "You are "+String.format("%.2f", dis/100)+" meter away from destination !!", Toast.LENGTH_SHORT).show();
                                    isNear=true;
                                }

                            }
                            if (result[2]<200.0) drawPoint(3 * (int) result[0], 3 * (int) result[1]);
                            else   drawPoint(3 * (int) localized_position[0], 3 * (int) localized_position[1]);

                        }else{
                            Log.d("beacon", "output_coordinates is  null");
                            drawPoint(3 * (int) localized_position[0], 3 * (int) localized_position[1]);
                        }


                        x_list.clear();
                        y_list.clear();
                    }

                }
                locations.clear();
                distances.clear();
              //Trilateration
             /*   if(beacon_count>=3) {

                    // Log.d(LOGTAG,sb.toString());
                    int[] cordArray= CvUtil.SendData_string(sb.toString());
                    if (cordArray!=null){
                       // Log.d("beacon", "coordinates received in java x="+cordArray[0]+" y="+cordArray[1]);
                        //multiply each coordinate by 3 for scaling properly
                        addTolist(cordArray);
                        if (x_list.size()>=10) {
                            double[] result=calculateAverageVal();
                           // Log.d("beacon","result[0]="+result[0]+"  result[1]="+result[1]);
                            drawPoint(3 * (int)result[0], 3 * (int)result[1]);
                            x_list=new ArrayList<>();
                            y_list=new ArrayList<>();
                        }


                      // Adding data to csv file
                        list.add(String.valueOf(cordArray[0]));
                        list.add(String.valueOf(cordArray[1]));
                        //Adding IMU data to CSV
                        list.add(String.valueOf(accelerometerReading[0])+","+String.valueOf(accelerometerReading[1])+","+String.valueOf(accelerometerReading[2]));
                        list.add(String.valueOf(gyroscopeReading[0])+","+String.valueOf(gyroscopeReading[1])+","+String.valueOf(gyroscopeReading[2]));
                        list.add(String.valueOf(magnetometerReading[0])+","+String.valueOf(magnetometerReading[1])+","+String.valueOf(magnetometerReading[2]));

                        // beacon_data_list.add(new String[]{"Trilateration","", "" ,"",String.valueOf(cordArray[0]),String.valueOf(cordArray[1])});
                        String[] tempArray=list.toArray(new String[list.size()]);
                        beacon_data_list.add(tempArray);
                       // writer.writeAll(beacon_data_list);
                        beacon_data_list.clear();
                        list.clear();
                    }
                }
                else {
                    beacon_data_list.clear();
                    list.clear();
                }*/
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
        myBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.second_floor_plan);
        options.inScaled = false;
        bmp_mono=BitmapFactory.decodeResource(getResources(), R.drawable.second_floor_route, options);
        tv_reset=(TextView)getView().findViewById(R.id.reset_path);
        //tempCanvas.drawBitmap(myBitmap, 0, 0, null);
       // paint = new Paint();
        paint.setStyle(Paint.Style.FILL);
        paint.setColor(Color.RED);
        paint.setAntiAlias(true);
        mImageView=(ImageView)getView().findViewById(R.id.plan);
        Log.d("beacon","view created ");
        tempBitmap = Bitmap.createBitmap(myBitmap.getWidth(), myBitmap.getHeight(), Bitmap.Config.RGB_565);
        path_bitmap = Bitmap.createBitmap(myBitmap.getWidth(), myBitmap.getHeight(), Bitmap.Config.RGB_565);
        tempCanvas= new Canvas(tempBitmap);
        path_canvas= new Canvas(path_bitmap);
        tempCanvas.drawBitmap(myBitmap, 0, 0, null);
        //tempCanvas1.drawBitmap(myBitmap, 0, 0, null);
        ImageView Go_button= (ImageView)getView().findViewById(R.id.go_button);
        start_spinner = (Spinner)getView().findViewById(R.id.start_spinner);
        stop_spinner = (Spinner)getView().findViewById(R.id.stop_spinner);
        SetItemstoView(); //set spinner data
        Go_button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
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
                    isNear=false;
                    input_cordinates=new int[]{hm_location.get(start_loc)[0],hm_location.get(start_loc)[1],hm_location.get(stop_loc)[0],hm_location.get(stop_loc)[1]};
                    performNavigation(input_cordinates);
                }


            }
        });

        tv_reset.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Toast.makeText(getActivity().getApplicationContext(), "Path reset", Toast.LENGTH_SHORT).show();
                 output_coordinates=null;
                 digkistra_coordinates=null;
                 copy_bmp=null;
                 mImageView.setImageDrawable(new BitmapDrawable(getResources(), myBitmap));


            }
        });

    }
    public double[] findNearestPixel(double[] position, double[][] list){

        ArrayList<Double> distances= new ArrayList<>();
        for (double[] pos: list){
            distances.add(Math.sqrt(Math.pow((pos[0]-position[0]),2)+Math.pow((pos[1]-position[1]),2)));
        }
        //Log.d("beacon","minimum value ="+Collections.min(distances));
        int indexOfMinimum = distances.indexOf(Collections.min(distances));
        //Log.d("beacon","nearest co-ordinate at ="+indexOfMinimum);
        //Log.d("beacon","distance ="+distances.get(indexOfMinimum));

        return new double[] {list[indexOfMinimum][0],list[indexOfMinimum][1],distances.get(indexOfMinimum)};

    }

    private double DisatanceToDestination(double[] source, int[] destination) {

        return Math.sqrt(Math.pow((destination[0]-source[0]),2)+Math.pow((destination[1]-source[1]),2));
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
                Toast.makeText(getActivity().getApplicationContext(), "Distance to travel= " + (int)distance_to_travel + " meter", Toast.LENGTH_SHORT).show();
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

    private void SetItemstoView() {
        ArrayAdapter<String> start_adapter = new ArrayAdapter<>( getActivity(), R.layout.custom_spinner,getResources().getStringArray(R.array.start_roomlist));
        ArrayAdapter<String> stop_adapter = new ArrayAdapter<>( getActivity(), R.layout.custom_spinner,getResources().getStringArray(R.array.stop_roomlist));
        start_adapter.setDropDownViewResource(R.layout.custom_spinner_dropdown);
        stop_adapter.setDropDownViewResource(R.layout.custom_spinner_dropdown);
        start_spinner.setAdapter(start_adapter);
        stop_spinner.setAdapter(stop_adapter);
        start_spinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> adapterView, View view, int position, long l) {
                start_loc=start_spinner.getItemAtPosition(position).toString();

            }

            @Override
            public void onNothingSelected(AdapterView<?> adapterView) {

            }
        });
        stop_spinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> adapterView, View view, int position, long l) {
                stop_loc=stop_spinner.getItemAtPosition(position).toString();
            }

            @Override
            public void onNothingSelected(AdapterView<?> adapterView) {

            }
        });
     /*   hm_location.put("Hall",new int[]{570,304});
        hm_location.put("Bedroom-1",new int[]{114,101});
        hm_location.put("Bedroom-2",new int[]{115,325});
        hm_location.put("Bedroom-3",new int[]{782,406});
        hm_location.put("Kitchen",new int[]{460,73});
        hm_location.put("Sitout",new int[]{292,625});
        hm_location.put("H-1",new int[]{81,107});
        hm_location.put("H-2",new int[]{284,107});
        hm_location.put("H-3",new int[]{81,393});
        hm_location.put("H-4",new int[]{273,393});
        hm_location.put("D-1",new int[]{380,153});
        hm_location.put("D-2",new int[]{557,153});
        hm_location.put("D-3",new int[]{376,541});
        hm_location.put("D-4",new int[]{540,541});*/
        hm_location.put("Hardware Lab1",new int[]{22,89});
        hm_location.put("Hardware Lab2",new int[]{187,327});
        hm_location.put("Hardware Lab3",new int[]{187,572});
        hm_location.put("Code Lab",new int[]{573,215});
        hm_location.put("Conference Room",new int[]{573,518});


        hm_location.put("Entrance",new int[]{17,977});
        hm_location.put("A1C1",new int[]{414,784});
        hm_location.put("A1C2",new int[]{290,784});
        hm_location.put("A1C3",new int[]{168,784});
        hm_location.put("A1C4",new int[]{56,784});
        hm_location.put("A1C5",new int[]{56,694});
        hm_location.put("A1C6",new int[]{168,694});
        hm_location.put("A1C7",new int[]{290,694});
        hm_location.put("A1C8",new int[]{416,694});

        hm_location.put("A2C1",new int[]{137,240});
        hm_location.put("A2C2",new int[]{135,117});
        hm_location.put("A2C3",new int[]{218,116});
        hm_location.put("A2C4",new int[]{220,240});

        hm_location.put("A3C1",new int[]{470,239});
        hm_location.put("A3C2",new int[]{470,112});
        hm_location.put("A3C3",new int[]{557,112});
        hm_location.put("A3C4",new int[]{558,238});

        hm_location.put("A4C1",new int[]{873,78});
        hm_location.put("A4C2",new int[]{987,78});
        hm_location.put("A4C3",new int[]{1105,76});
        hm_location.put("A4C4",new int[]{1107,164});
        hm_location.put("A5C5",new int[]{989,165});
        hm_location.put("A5C6",new int[]{870,164});

        hm_location.put("A5C1",new int[]{870,313});
        hm_location.put("A5C2",new int[]{985,308});
        hm_location.put("A5C3",new int[]{1107,308});
        hm_location.put("A5C4",new int[]{1107,392});
        hm_location.put("A5C5",new int[]{983,395});
        hm_location.put("A5C6",new int[]{869,394});


        hm_location.put("A6C1",new int[]{875,553});
        hm_location.put("A6C2",new int[]{986,550});
        hm_location.put("A6C3",new int[]{1107,550});
        hm_location.put("A6C4",new int[]{1107,637});
        hm_location.put("A6C5",new int[]{983,637});
        hm_location.put("A6C6",new int[]{867,634});

        hm_location.put("A7C1",new int[]{866,782});
        hm_location.put("A7C2",new int[]{987,783});
        hm_location.put("A7C3",new int[]{1106,782});
        hm_location.put("A7C4",new int[]{1105,871});
        hm_location.put("A7C5",new int[]{988,866});

        hm_location.put("A8C1",new int[]{985,1049});
        hm_location.put("A8C2",new int[]{1105,1043});
        hm_location.put("A8C3",new int[]{1106,1129});
        hm_location.put("A8C4",new int[]{987,1135});

        hm_location.put("A9C1",new int[]{989,1291});
        hm_location.put("A9C2",new int[]{1106,1293});
        hm_location.put("A9C3",new int[]{1109,1379});
        hm_location.put("A9C4",new int[]{989,1377});
        hm_location.put("A9C5",new int[]{836,1367});


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
