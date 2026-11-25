package org.firstinspires.ftc.teamcode.pinpoint;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Small XY-facing API over the Pinpoint pose reader used by BioBuzzTeleOp.
 * Coordinates are field-relative: X is forward and Y is left, in inches.
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

    /** Set the field coordinate in inches and heading in radians at the current location. */
    public boolean setFieldOrigin(double xForwardInches, double yLeftInches, double headingRadians) {
        return pose.anchor(xForwardInches, -yLeftInches, headingRadians);
    }

    /** Most recently accepted field X coordinate, positive forward, in inches. */
    public double getXInches() {
        return pose.forward;
    }

    /** Most recently accepted field Y coordinate, positive left, in inches. */
    public double getYInches() {
        return pose.left;
    }

    /** Most recently accepted field-relative heading, in radians. */
    public double getHeading() {
        return pose.getHeading();
    }

    /** Return the most recently accepted field coordinates as [X, Y], in inches. */
    public double[] getPositionInches() {
        return new double[] { getXInches(), getYInches() };
    }

    /** True while the last accepted XY sample is recent and the field origin is valid. */
    public boolean isPositionValid() {
        return pose.recentlyLocated(MAX_POSITION_AGE_SECONDS);
    }

    /** True while a recent, anchored heading is available for field-centric driving. */
    public boolean isHeadingUsable() {
        return pose.headingUsable() && Double.isFinite(pose.getHeading());
    }

    /** Automatic steering requires an accepted position sample and finite heading/rate. */
    public boolean isPoseValidForControl() {
        return pose.positionValid && isPositionValid() && isHeadingUsable()
                && Double.isFinite(pose.yawRate);
    }

    /** Pinpoint angular velocity in radians per second, positive toward the left. */
    public double getYawRate() {
        return pose.yawRate;
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
