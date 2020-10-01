package com.nexenio.bleindoorpositioningdemo.bluetooth;


import java.util.ArrayList;

public class BeaconStore {

    private static BeaconStore mBeaconStore;
    private static ArrayList<BeaconLoc> sBeaconArrayList;

    private BeaconStore() {
        //ToDo here
        sBeaconArrayList = new ArrayList<>();
    }

    public static BeaconStore getInstance() {
        if (mBeaconStore == null) {
            mBeaconStore = new BeaconStore();
        }
        return mBeaconStore;
    }
    public static ArrayList<BeaconLoc> getItemDataArrayList() {
        return sBeaconArrayList;
    }
    public static void addBeaconArrayList(BeaconLoc pBeaconLoc){
        sBeaconArrayList.add(pBeaconLoc);
    }

    public static void setBeaconArrayList(ArrayList<BeaconLoc> pBeaconLoc) {
        sBeaconArrayList = pBeaconLoc;
    }

    public static BeaconLoc getCurrentItem(int pId) {
            for (BeaconLoc vBeaconLoc: sBeaconArrayList) {
                if (vBeaconLoc.getId() == pId) {
                    return vBeaconLoc;
                }
            }
            return null;
    }
}
