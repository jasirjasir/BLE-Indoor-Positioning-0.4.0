package com.nexenio.bleindoorpositioningdemo.bluetooth;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothManager;
import android.content.Context;
import android.content.Intent;
import android.os.Handler;
import android.util.Log;
import android.widget.Toast;

import com.nexenio.bleindoorpositioning.ble.advertising.AdvertisingPacket;
import com.nexenio.bleindoorpositioning.ble.beacon.Beacon;
import com.nexenio.bleindoorpositioning.ble.beacon.BeaconManager;
import com.nexenio.bleindoorpositioning.ble.beacon.IBeacon;
import com.nexenio.bleindoorpositioning.location.Location;
import com.nexenio.bleindoorpositioning.location.provider.IBeaconLocationProvider;
import com.nexenio.bleindoorpositioningdemo.ui.beaconview.pathplanning.BeaconNavigateFragment;
import com.polidea.rxandroidble.RxBleClient;
import com.polidea.rxandroidble.scan.ScanResult;
import com.polidea.rxandroidble.scan.ScanSettings;

import androidx.annotation.NonNull;
import rx.Observer;
import rx.Subscription;

/**
 * Created by steppschuh on 24.11.17.
 */


public class BluetoothClient {

    private static final String TAG = "beacon";//BluetoothClient.class.getSimpleName();
    public static final int REQUEST_CODE_ENABLE_BLUETOOTH = 10;

    private static BluetoothClient instance;

    public static Context mContext;
    private BluetoothManager bluetoothManager;
    private BluetoothAdapter bluetoothAdapter;
    private BeaconManager beaconManager = BeaconManager.getInstance();

    private RxBleClient rxBleClient;
    private Subscription scanningSubscription;
    static Handler handler = new Handler();

    public BluetoothClient() {

    }


    public static BluetoothClient getInstance() {
        if (instance == null) {
            instance = new BluetoothClient();
        }
        return instance;
    }

    public static void initialize(@NonNull Context context) {
        Log.v(TAG, "Initializing with context: " + context);
        mContext = context;
        BluetoothClient instance = getInstance();
        instance.rxBleClient = RxBleClient.create(context);
        instance.bluetoothManager = (BluetoothManager) context.getSystemService(Context.BLUETOOTH_SERVICE);
        instance.bluetoothAdapter = instance.bluetoothManager.getAdapter();
        if (instance.bluetoothAdapter == null) {
            Log.e(TAG, "Bluetooth adapter is not available");
        }
    }

    public static void  startScanning() {
        if (isScanning()) {
            return;
        }

        final BluetoothClient instance = getInstance();
        Log.d(TAG, "Starting to scan for beacons");

        ScanSettings scanSettings = new ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .setCallbackType(ScanSettings.CALLBACK_TYPE_ALL_MATCHES)
                .build();
        Log.d(TAG, "Starting to scan for beacons after scan build");
        instance.scanningSubscription = instance.rxBleClient.scanBleDevices(scanSettings)
                .subscribe(new Observer<ScanResult>() {
                    @Override
                    public void onCompleted() {

                    }

                    @Override
                    public void onError(Throwable e) {
                        Log.e(TAG, "Bluetooth scanning error", e);
                    }

                    @Override
                    public void onNext(ScanResult scanResult) {
                        instance.processScanResult(scanResult);
                    }
                });
    }

    public static void stopScanning() {
        if (!isScanning()) {
            return;
        }

        BluetoothClient instance = getInstance();
        Log.d(TAG, "Stopping to scan for beacons");
        instance.scanningSubscription.unsubscribe();
    }

    public static boolean isScanning() {
        Subscription subscription = getInstance().scanningSubscription;
        return subscription != null && !subscription.isUnsubscribed();
    }

    public static boolean isBluetoothEnabled() {
        BluetoothClient instance = getInstance();
        return instance.bluetoothAdapter != null && instance.bluetoothAdapter.isEnabled();
    }

    public static void requestBluetoothEnabling(@NonNull Activity activity) {
        Log.d(TAG, "Requesting bluetooth enabling");
        Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
        activity.startActivityForResult(enableBtIntent, REQUEST_CODE_ENABLE_BLUETOOTH);
    }

    private void processScanResult(@NonNull ScanResult scanResult) {
        String macAddress = scanResult.getBleDevice().getMacAddress();
        byte[] data = scanResult.getScanRecord().getBytes();
        AdvertisingPacket advertisingPacket = BeaconManager.processAdvertisingData(macAddress, data, scanResult.getRssi());

        if (advertisingPacket != null) {
            Beacon beacon = BeaconManager.getBeacon(macAddress, advertisingPacket);
            if (beacon instanceof IBeacon && !beacon.hasLocation()) {

                beacon.setLocationProvider(createDebuggingLocationProvider((IBeacon) beacon,beacon));
            }
        }
    }

    public static   IBeaconLocationProvider<IBeacon> createDebuggingLocationProvider(IBeacon iBeacon, Beacon beacon) {
        final Location beaconLocation = new Location();

        try {
            final BeaconLoc vBeaconLoc = BeaconStore.getCurrentItem(iBeacon.getMinor());
            Log.d("beacon","inside " + vBeaconLoc.getName());
            beacon.setMajorId(iBeacon.getMajor());
            beacon.setMinorId(iBeacon.getMinor());
            beacon.setX(vBeaconLoc.getKx());
            beacon.setY(vBeaconLoc.getKy());
            beacon.setName(vBeaconLoc.getName());
            beaconLocation.setLatitude(10.0199105668819222);
            beaconLocation.setLongitude(76.35076967277124);
            Toast.makeText(mContext,"Registering  "+vBeaconLoc.getName(),Toast.LENGTH_SHORT).show();

            handler.postDelayed(new Runnable()
            {
                @Override
                public void run()
                {
                  //  Toast.makeText(mContext,"Registering  "+vBeaconLoc.getName(),Toast.LENGTH_SHORT).show();
                }
            }, 3000);
        } catch (Exception e) {
            Log.d("beacon","Errorr while registering beacon :  " + iBeacon.getMinor());
          //  if (iBeacon.getMinor()==80){
                beaconLocation.setLatitude(10.0);
                beaconLocation.setLongitude(10.0);
                Log.d("beacon","Load dummy data due to noise ");
           // }
            e.printStackTrace();
        }


        /*switch (iBeacon.getMinor()) {

            case 58104: {
                Log.d("beacon","inside beacon1 ");
                beaconLocation.setLatitude(10.0199105668819222);
                beaconLocation.setLongitude(76.35076967277124);
                beaconLocation.setAltitude(2);
                beacon.setMajorId(iBeacon.getMajor());
                beacon.setMinorId(iBeacon.getMinor());
                beacon.setX(140);
                beacon.setY(0);
                beacon.setName("beacon1");
                break;
            }
            case 26324: {
                Log.d("beacon","inside beacon2 ");
                beaconLocation.setLatitude(10.0199105668819222);
                beaconLocation.setLongitude(76.35079997277124);
                beaconLocation.setAltitude(2);
                beacon.setMajorId(iBeacon.getMajor());
                beacon.setMinorId(iBeacon.getMinor());
                beacon.setX(0);
                beacon.setY(240);
                beacon.setName("beacon2");
                break;
            }
            case 56039: {
                Log.d("beacon","inside beacon3 ");
                beaconLocation.setLatitude(10.019932007511654);
                beaconLocation.setLongitude(76.35075204170859);
                beaconLocation.setAltitude(2);
                beacon.setMajorId(iBeacon.getMajor());
                beacon.setMinorId(iBeacon.getMinor());
                beacon.setX(150);
                beacon.setY(420);
                beacon.setName("beacon3");
                break;
            }
            case 63458: {
                Log.d("beacon","inside beacon4 ");
                beaconLocation.setLatitude(10.019950321917987);
                beaconLocation.setLongitude(76.35075389610897);
                beaconLocation.setAltitude(2);
                beacon.setMajorId(iBeacon.getMajor());
                beacon.setMinorId(iBeacon.getMinor());
                beacon.setX(550);
                beacon.setY(190);
                beacon.setName("beacon4");
                break;
            }
            case 31369: {
                Log.d("beacon","inside beacon5 ");
                beaconLocation.setLatitude(10.019950576710334);
                beaconLocation.setLongitude(76.35079766487668);
                beaconLocation.setAltitude(2);
                beacon.setMajorId(iBeacon.getMajor());
                beacon.setMinorId(iBeacon.getMinor());
                beacon.setX(320);
                beacon.setY(420);
                beacon.setName("beacon5");
                break;
            }
            case 52844: {
                Log.d("beacon","inside beacon 6 ");
                beaconLocation.setLatitude(10.01995034545);
                beaconLocation.setLongitude(76.350797656767);
                beaconLocation.setAltitude(2);
                beacon.setMajorId(iBeacon.getMajor());
                beacon.setMinorId(iBeacon.getMinor());
                beacon.setX(520);
                beacon.setY(580);
                beacon.setName("beacon6");
                break;
            }
            case 25389: {
                Log.d("beacon","inside beacon 7");
                beaconLocation.setLatitude(10.019950345422);
                beaconLocation.setLongitude(76.350797656767);
                beaconLocation.setAltitude(2);
                beacon.setMajorId(iBeacon.getMajor());
                beacon.setMinorId(iBeacon.getMinor());
                beacon.setX(412);
                beacon.setY(225);
                beacon.setName("beacon7");
                break;
            }
            case 44784: {
                Log.d("beacon","inside beacon 8 ");
                beaconLocation.setLatitude(10.01995034542225);
                beaconLocation.setLongitude(76.350797656767);
                beaconLocation.setAltitude(2);
                beacon.setMajorId(iBeacon.getMajor());
                beacon.setMinorId(iBeacon.getMinor());
                beacon.setX(38);
                beacon.setY(400);
                beacon.setName("beacon8");
                break;
            }
            case 30190: {
                Log.d("beacon","inside beacon 9");
                beaconLocation.setLatitude(10.019950345422232);
                beaconLocation.setLongitude(76.350797656767);
                beaconLocation.setAltitude(2);
                beacon.setMajorId(iBeacon.getMajor());
                //beacon.setMinorId(iBeacon.getMinor());
                beacon.setX(254);
                beacon.setY(442);
                beacon.setName("beacon9");
                break;
            }
            case 36303: {
                Log.d("beacon","inside beacon 10");
                beaconLocation.setLatitude(10.019950345456);
                beaconLocation.setLongitude(76.350797656767);
                beaconLocation.setAltitude(2);
                beacon.setMajorId(iBeacon.getMajor());
                beacon.setMinorId(iBeacon.getMinor());
                beacon.setX(345);
                beacon.setY(342);
                beacon.setName("beacon10");
                break;
            }
            case 19641: {
                Log.d("beacon","inside beacon 11");
                beaconLocation.setLatitude(10.019950345456);
                beaconLocation.setLongitude(76.350797656767);
                beaconLocation.setAltitude(2);
                beacon.setMajorId(iBeacon.getMajor());
                 beacon.setMinorId(iBeacon.getMinor());
                beacon.setX(345);
                beacon.setY(342);
                beacon.setName("beacon11");
                break;
            }
        }*/
        return new IBeaconLocationProvider<IBeacon>(iBeacon) {
            @Override
            protected void updateLocation() {
                this.location = beaconLocation;
            }

            @Override
            protected boolean canUpdateLocation() {
                return true;
            }
        };
    }

}
