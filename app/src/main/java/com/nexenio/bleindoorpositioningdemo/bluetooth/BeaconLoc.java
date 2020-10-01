package com.nexenio.bleindoorpositioningdemo.bluetooth;

public class BeaconLoc {

  private int id;
  private int ky;
  private int kx;
  private int minor;
  private int major;
  private String uuid;
  private String name;

    public BeaconLoc(int pId, int pKy, int pKx, int pMinor, int pMajor, String pUuid, String pName) {
        id = pId;
        ky = pKy;
        kx = pKx;
        minor = pMinor;
        major = pMajor;
        uuid = pUuid;
        name = pName;
    }

    public int getId() {
        return id;
    }

    public void setId(int pId) {
        id = pId;
    }

    public int getKy() {
        return ky;
    }

    public void setKy(int pKy) {
        ky = pKy;
    }

    public int getKx() {
        return kx;
    }

    public void setKx(int pKx) {
        kx = pKx;
    }

    public int getMinor() {
        return minor;
    }

    public void setMinor(int pMinor) {
        minor = pMinor;
    }

    public int getMajor() {
        return major;
    }

    public void setMajor(int pMajor) {
        major = pMajor;
    }

    public String getUuid() {
        return uuid;
    }

    public void setUuid(String pUuid) {
        uuid = pUuid;
    }

    public String getName() {
        return name;
    }

    public void setName(String pName) {
        name = pName;
    }
}
