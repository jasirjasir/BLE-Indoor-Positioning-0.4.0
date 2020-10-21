package com.nexenio.bleindoorpositioningdemo;

public interface SensorDataListener {
    public void onSensorChange(boolean updatePosition, float[] DRPose);
}
