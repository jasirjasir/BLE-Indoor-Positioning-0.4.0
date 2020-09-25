package com.nexenio.bleindoorpositioningdemo.ui.beaconview.map;


import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Point;
import android.os.Bundle;
import androidx.annotation.CallSuper;

import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import com.nexenio.bleindoorpositioning.IndoorPositioning;
import com.nexenio.bleindoorpositioning.ble.beacon.Beacon;
import com.nexenio.bleindoorpositioning.ble.beacon.BeaconUpdateListener;
import com.nexenio.bleindoorpositioning.location.Location;
import com.nexenio.bleindoorpositioning.location.LocationListener;
import com.nexenio.bleindoorpositioning.location.provider.LocationProvider;
import com.nexenio.bleindoorpositioningdemo.R;
import com.nexenio.bleindoorpositioningdemo.location.AndroidLocationProvider;
import com.nexenio.bleindoorpositioningdemo.ui.beaconview.BeaconViewFragment;

public class BeaconMapFragment extends BeaconViewFragment {

    private BeaconMap beaconMap;

    public BeaconMapFragment() {
        super();
        beaconFilters.add(uuidFilter);
    }

    @Override
    protected int getLayoutResourceId() {
        return R.layout.fragment_beacon_map;
    }

    @Override
    protected LocationListener createDeviceLocationListener() {
        return new LocationListener() {
            @Override
            public void onLocationUpdated(LocationProvider locationProvider, Location location) {
                if (locationProvider instanceof IndoorPositioning) {
                   beaconMap.setDeviceLocation(location);
                    beaconMap.setPredictedDeviceLocation(IndoorPositioning.getLocationPredictor().getLocation());
                    beaconMap.fitToCurrentLocations();
                    Log.d("beacon","Location updated after trilateration");
                } else if (locationProvider instanceof AndroidLocationProvider) {
                    // TODO: remove artificial noise
                    //location.setLatitude(location.getLatitude() + Math.random() * 0.0002);
                    //location.setLongitude(location.getLongitude() + Math.random() * 0.0002);
                }
            }
        };
    }

    @Override
    protected BeaconUpdateListener createBeaconUpdateListener() {
        return new BeaconUpdateListener() {
            @Override
            public void onBeaconUpdated(Beacon beacon) {
                beaconMap.setBeacons(getBeacons());
            }
        };
    }

    @CallSuper
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View inflatedView = super.onCreateView(inflater, container, savedInstanceState);
        beaconMap = inflatedView.findViewById(R.id.beaconMap);
        beaconMap.setBeacons(getBeacons());

        BitmapFactory.Options options = new BitmapFactory.Options();
        options.inScaled = false;
        Bitmap backgroundImage = BitmapFactory.decodeResource(getResources(), R.drawable.floorplan_office, options);

        Location firstReferenceLocation = new Location(10.019895668819222, 76.35076547277124);
        Location secondReferenceLocation = new Location(10.019932007511654, 76.35075204170859);

        Point firstReferencePoint = new Point(284, 753);
        Point secondReferencePoint = new Point(114, 311);

        BeaconMapBackground beaconMapBackground = BeaconMapBackground.Builder.from(backgroundImage)
                .withFirstReferenceLocation(firstReferenceLocation, firstReferencePoint)
                .withSecondReferenceLocation(secondReferenceLocation, secondReferencePoint)
                .build();

        beaconMap.setMapBackground(beaconMapBackground);

        return inflatedView;
    }

}
