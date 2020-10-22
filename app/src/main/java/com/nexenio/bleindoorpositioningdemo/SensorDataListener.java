package com.nexenio.bleindoorpositioningdemo;

public interface SensorDataListener {
    public void onSensorChange(float[] DRPose, int stepCount);
}
