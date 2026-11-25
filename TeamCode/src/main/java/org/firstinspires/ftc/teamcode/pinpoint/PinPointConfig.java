package org.firstinspires.ftc.teamcode.pinpoint;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.ErrorDetectionType;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public final class PinPointConfig {
    public static final String PINPOINT = "pinpoint";
    public static final double TILE_IN = 24.0;
    public static final double CENTER_FROM_LEFT_EDGE_IN = 7.755905511811024;
    public static final double CENTER_FROM_REAR_EDGE_IN = 10.42232283464567;
    public static final double X_POD_RIGHT_IN = 84.0 / 25.4;
    public static final double Y_POD_FORWARD_IN = 160.875 / 25.4;
    // X must count up forward and Y must count up left.
    public static final EncoderDirection X_POD_DIRECTION = EncoderDirection.REVERSED;
    public static final EncoderDirection Y_POD_DIRECTION = EncoderDirection.REVERSED;
    static int configurePinpoint(GoBildaPinpointDriver device) {
        int version = device.getDeviceVersion();
        if (version < 1) throw new IllegalStateException("Pinpoint firmware version unavailable");
        device.setErrorDetectionType(version >= 3 ? ErrorDetectionType.CRC : ErrorDetectionType.LOCAL_TEST);
        device.setOffsets(-X_POD_RIGHT_IN, Y_POD_FORWARD_IN, DistanceUnit.INCH);
        device.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        device.setEncoderResolution(GoBildaOdometryPods.goBILDA_4_BAR_POD);
        return version;
    }
}
