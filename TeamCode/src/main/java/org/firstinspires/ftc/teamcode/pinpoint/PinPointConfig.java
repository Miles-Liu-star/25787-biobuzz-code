package org.firstinspires.ftc.teamcode.pinpoint;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.ErrorDetectionType;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.GoBildaOdometryPods;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public final class PinPointConfig {
    public static final String PINPOINT = "pinpoint";
    public static final double TILE_MM = 609.6;
    public static final double CENTER_FROM_LEFT_EDGE_MM = 0;//起始时机器人中心到场地左边缘的距离
    public static final double CENTER_FROM_REAR_EDGE_MM = 0;//起始时机器人中心到车尾的距离
    public static final double X_POD_RIGHT_MM = 84;
    public static final double Y_POD_FORWARD_MM = 160.875;
    // X must count up forward and Y must count up left.
    public static final EncoderDirection X_POD_DIRECTION = EncoderDirection.FORWARD;
    public static final EncoderDirection Y_POD_DIRECTION = EncoderDirection.FORWARD;
    static int configurePinpoint(GoBildaPinpointDriver device) {
        int version = device.getDeviceVersion();
        if (version < 1) throw new IllegalStateException("Pinpoint firmware version unavailable");
        device.setErrorDetectionType(version >= 3 ? ErrorDetectionType.CRC : ErrorDetectionType.LOCAL_TEST);
        device.setOffsets(-X_POD_RIGHT_MM, Y_POD_FORWARD_MM, DistanceUnit.MM);
        device.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        device.setEncoderResolution(GoBildaOdometryPods.goBILDA_4_BAR_POD);
        return version;
    }
}
