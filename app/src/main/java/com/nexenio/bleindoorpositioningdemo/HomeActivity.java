package com.nexenio.bleindoorpositioningdemo;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Bundle;
import androidx.annotation.NonNull;
import com.google.android.material.bottomnavigation.BottomNavigationView;
import androidx.coordinatorlayout.widget.CoordinatorLayout;
import com.google.android.material.snackbar.Snackbar;

import androidx.core.app.ActivityCompat;
import androidx.fragment.app.Fragment;
import androidx.appcompat.app.AppCompatActivity;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;

import com.nexenio.bleindoorpositioning.location.Location;
import com.nexenio.bleindoorpositioning.location.multilateration.Multilateration;
import com.nexenio.bleindoorpositioningdemo.bluetooth.BeaconLoc;
import com.nexenio.bleindoorpositioningdemo.bluetooth.BeaconStore;
import com.nexenio.bleindoorpositioningdemo.bluetooth.BluetoothClient;
import com.nexenio.bleindoorpositioningdemo.location.AndroidLocationProvider;
import com.nexenio.bleindoorpositioningdemo.ui.beaconview.chart.BeaconChartFragment;
import com.nexenio.bleindoorpositioningdemo.ui.beaconview.map.BeaconMapFragment;
import com.nexenio.bleindoorpositioningdemo.ui.beaconview.pathplanning.BeaconNavigateFragment;
import com.nexenio.bleindoorpositioningdemo.ui.beaconview.pathplanning.SensorEventFragment;
import com.nexenio.bleindoorpositioningdemo.ui.beaconview.radar.BeaconRadarFragment;

import org.apache.commons.math3.fitting.leastsquares.LeastSquaresOptimizer;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.opencv.android.OpenCVLoader;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collections;

public class HomeActivity extends AppCompatActivity implements BottomNavigationView.OnNavigationItemSelectedListener {

    private static final String TAG = "beacon";

    private CoordinatorLayout coordinatorLayout;
    private BottomNavigationView bottomNavigationView;
    static {
        if (!OpenCVLoader.initDebug()) {
            Log.d("pahPlanning", "Unable to load OpenCV");
        } else {
            System.loadLibrary("localize-lib");
            System.loadLibrary("trilateration-lib");
            System.loadLibrary("pathplan-lib");

        }
    }


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,WindowManager.LayoutParams.FLAG_FULLSCREEN);
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_home);
        Log.d(TAG, "App started");
        Log.d(TAG, BluetoothClient.class.getSimpleName());
        // setup UI
        coordinatorLayout = findViewById(R.id.coordinatorLayout);
        /*bottomNavigationView = findViewById(R.id.bottomNavigationView);
        bottomNavigationView.setOnNavigationItemSelectedListener(this);
        bottomNavigationView.setSelectedItemId(R.id.navigation_radar);*/
        //set beacon locations
        BeaconStore.setBeaconArrayList(getBeaconsFromJSON());

        // setup locationc
        AndroidLocationProvider.initialize(this);

        // setup bluetooth
        BluetoothClient.initialize(this);

       // GetLocation ();
        ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE}, 400);


        getSupportFragmentManager().beginTransaction().replace(R.id.frameLayout, new BeaconNavigateFragment()).commit();
    }
    private ArrayList<BeaconLoc> getBeaconsFromJSON() {
        ArrayList<BeaconLoc> vBeaconLocArrayList = new ArrayList<>();
        try {
            JSONObject obj = new JSONObject(loadJSONFromAsset("beacons.json"));
            JSONArray m_jArry = obj.getJSONArray("beacon");

            for (int i = 0; i < m_jArry.length(); i++) {
                JSONObject jo_inside = m_jArry.getJSONObject(i);
                Log.d("ID-->", "" + jo_inside.getInt("id"));
                int id = jo_inside.getInt("id");
                int ky = jo_inside.getInt("ky");
                int kx = jo_inside.getInt("kx");
                int minor = jo_inside.getInt("minor");
                int major = jo_inside.getInt("major");
                String uuid = jo_inside.getString("uuid");
                String name = jo_inside.getString("name");

                //Add your values in your `ArrayList` as below:
                vBeaconLocArrayList.add(new BeaconLoc(id, ky, kx, minor, major, uuid, name));
            }
            return vBeaconLocArrayList;
        } catch (JSONException e) {
            e.printStackTrace();
            return null;
        }
    }

    private String loadJSONFromAsset(String fileName) {
        String json = null;
        try {
            InputStream is = HomeActivity.this.getAssets().open(fileName);
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
    private void GetLocation() {
        // Step 1: Outdoor reference location
        double measuredLatitude = 10.019895668819222;
        double measuredLongitude = 76.35076547277124;
        Location outdoorReferenceLocation = new Location(measuredLatitude, measuredLongitude);

// Step 2: Indoor reference or beacon location
        double distance = 7.05; // in meters
        double angle = 30; // in degrees (0° is North)
        Location indoorReferenceLocation = outdoorReferenceLocation.getShiftedLocation(distance, angle);
        Log.d("beacon","co-ordinates are :"+indoorReferenceLocation); // print latitude and longitude
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.menu_filter:
                Log.d(TAG, "BeaconFilter");
                return true;
            default:
                break;
        }
        return false;
    }

    @Override
    public boolean onNavigationItemSelected(@NonNull MenuItem item) {
        Fragment selectedFragment = null;
        switch (item.getItemId()) {
           /* case R.id.navigation_map: {
                selectedFragment = new BeaconMapFragment();
                break;
            }*/
            case R.id.navigation_radar: {
                selectedFragment = new BeaconRadarFragment();
                break;
            }
            case R.id.navigation_chart: {
                selectedFragment = new BeaconChartFragment();
                break;
            }
            case R.id.navigation_nav: {
                selectedFragment = new BeaconNavigateFragment();
                break;
            }
        }
        if (selectedFragment != null) {
            getSupportFragmentManager().beginTransaction().replace(R.id.frameLayout, selectedFragment).commit();
        }
        return true;
    }

    @Override
    protected void onResume() {
        super.onResume();

        // observe location
        if (!AndroidLocationProvider.hasLocationPermission(this)) {
            AndroidLocationProvider.requestLocationPermission(this);
        } else if (!AndroidLocationProvider.isLocationEnabled(this)) {
            requestLocationServices();
        }
        AndroidLocationProvider.startRequestingLocationUpdates();
        AndroidLocationProvider.requestLastKnownLocation();

        // observe bluetooth
        if (!BluetoothClient.isBluetoothEnabled()) {
            requestBluetooth();
        }
        BluetoothClient.startScanning();
    }

    @Override
    protected void onPause() {
        // stop observing location
        AndroidLocationProvider.stopRequestingLocationUpdates();
        Log.d(TAG,"on pause to stop scan");
        // stop observing bluetooth
        BluetoothClient.stopScanning();

        super.onPause();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        switch (requestCode) {
            case AndroidLocationProvider.REQUEST_CODE_LOCATION_PERMISSIONS: {
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    Log.d(TAG, "Location permission granted");
                    AndroidLocationProvider.startRequestingLocationUpdates();
                } else {
                    Log.d(TAG, "Location permission not granted. Wut?");
                }
                break;
            }
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        switch (requestCode) {
            case BluetoothClient.REQUEST_CODE_ENABLE_BLUETOOTH: {
                if (resultCode == RESULT_OK) {
                    Log.d(TAG, "Bluetooth enabled, starting to scan");
                    BluetoothClient.startScanning();
                } else {
                    Log.d(TAG, "Bluetooth not enabled, invoking new request");
                    BluetoothClient.requestBluetoothEnabling(this);
                }
                break;
            }
        }
    }

    private void requestLocationServices() {
        Snackbar snackbar = Snackbar.make(
                coordinatorLayout,
                R.string.error_location_disabled,
                Snackbar.LENGTH_INDEFINITE
        );
        snackbar.setAction(R.string.action_enable, new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                AndroidLocationProvider.requestLocationEnabling(HomeActivity.this);
            }
        });
        snackbar.show();
    }

    private void requestBluetooth() {
        Snackbar snackbar = Snackbar.make(
                coordinatorLayout,
                R.string.error_bluetooth_disabled,
                Snackbar.LENGTH_INDEFINITE
        );
        snackbar.setAction(R.string.action_enable, new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                BluetoothClient.requestBluetoothEnabling(HomeActivity.this);
            }
        });
        snackbar.show();
    }
    public void findNearestPixel(){
        int[] input=new int[]{76,76};
        double[][] position_list=new double[][]{{50,50},{100,100},{0,0}};
        ArrayList<Double> distances= new ArrayList<>();
        for (double[] pos: position_list){
            distances.add(Math.sqrt(Math.pow((pos[0]-input[0]),2)+Math.pow((pos[1]-input[1]),2)));
        }
        int indexOfMinimum = distances.indexOf(Collections.min(distances));
        Log.d("beacon","nearest co-ordinate at ="+indexOfMinimum);
        Log.d("beacon","distance ="+distances.get(indexOfMinimum));

    }
}
