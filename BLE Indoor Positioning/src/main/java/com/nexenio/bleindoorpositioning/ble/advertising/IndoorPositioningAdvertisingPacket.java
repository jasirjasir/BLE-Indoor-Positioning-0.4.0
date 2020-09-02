package com.nexenio.bleindoorpositioning.ble.advertising;

import java.util.UUID;

/**
 * For advertising packets as specified in Apples <a href="http://www.blueupbeacons.com/docs/dev/Proximity%20Beacon%20Specification%20R1.pdf">Proximity
 * Beacon Specification</a>.
 */

public class IndoorPositioningAdvertisingPacket extends IBeaconAdvertisingPacket {

    public final static UUID INDOOR_POSITIONING_UUID = UUID.fromString("F7826DA6-4FA2-4E98-8024-BC5B71E0893E");

    public IndoorPositioningAdvertisingPacket(byte[] data) {
        super(data);
    }

    public static boolean meetsSpecification(byte[] data) {
        return dataMatchesUuid(data, INDOOR_POSITIONING_UUID) && IBeaconAdvertisingPacket.meetsSpecification(data);
    }

}
