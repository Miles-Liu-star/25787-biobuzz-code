package org.firstinspires.ftc.teamcode.pinpoint;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Small XY-facing API over the Pinpoint pose reader used by BioBuzzTeleOp.
 * Coordinates are field-relative: X is forward and Y is left, in millimeters.
 */
public final class PinpointXYReader {
    private static final double MAX_POSITION_AGE_SECONDS = 0.25;
    private final PinpointPose pose;

    public PinpointXYReader(HardwareMap hardwareMap) {
        pose = new PinpointPose(hardwareMap);
    }

    /** Configure Pinpoint and reset its position and IMU. Call while the robot is still. */
    public boolean start() {
        return pose.startImu();
    }

    /** Poll Pinpoint. Call repeatedly from the OpMode loop. */
    public void update() {
        pose.update();
    }

    /** Set the field coordinate and heading at the robot's current location. */
    public boolean setFieldOrigin(double xForwardMm, double yLeftMm, double headingRadians) {
        return pose.anchor(xForwardMm, -yLeftMm, headingRadians);
    }

    /** Most recently accepted field X coordinate, positive forward, in millimeters. */
    public double getXmm() {
        return pose.forward;
    }

    /** Most recently accepted field Y coordinate, positive left, in millimeters. */
    public double getYmm() {
        return pose.left;
    }

    /** Most recently accepted field-relative heading, in radians. */
    public double getHeading() {
        return pose.getHeading();
    }

    /** Return the most recently accepted field coordinates as [X, Y], in millimeters. */
    public double[] getPositionMm() {
        return new double[] { getXmm(), getYmm() };
    }

    /** True while the last accepted XY sample is recent and the field origin is valid. */
    public boolean isPositionValid() {
        return pose.recentlyLocated(MAX_POSITION_AGE_SECONDS);
    }

    public String getStatus() {
        return pose.status;
    }

    public int getFirmwareVersion() {
        return pose.firmwareVersion;
    }

    public int getBadReadCount() {
        return pose.badReads;
    }

    public boolean needsReanchor() {
        return pose.requiresReanchor();
    }
}
