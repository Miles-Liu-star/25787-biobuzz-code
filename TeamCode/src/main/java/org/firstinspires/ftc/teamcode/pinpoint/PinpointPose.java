package org.firstinspires.ftc.teamcode.pinpoint;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.Register;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/** The field pose is a rigid transform of the Pinpoint's measured pose. */
public final class PinpointPose {
    private static final double HEADING_GRACE = 0.12;
    private static final double POSITION_GRACE = 0.12;
    private static final double READ_RETRY_SECONDS = 0.05;
    private static final double MAX_READ_RETRY_SECONDS = 0.50;
    private static final double REANCHOR_AFTER_READ_GAP = 0.25;
    private static final double BULK_SCOPE_RETRY_SECONDS = 2;
    private static final double MAX_YAW_RATE = 18;
    private static final double HEADING_SLACK = Math.toRadians(1.5);
    private static final double MAX_POSITION_RATE_INCHES_PER_SECOND = 5000.0 / 25.4;
    private static final double POSITION_SLACK_INCHES = 45.0 / 25.4;
    private static final String POSITION_REANCHOR = "Pinpoint 位置需要重新对齐，回起点按 Y";
    private final GoBildaPinpointDriver device;
    private boolean started, anchored, positionAnchored, needsAnchor, positionNeedsAnchor, podsValid;
    private double sensorYaw, headingShift, localX, localY, originX, originY, fieldForward, fieldLeft;
    private double previousTime, previousX, previousY, lastHeadingTime = Double.NEGATIVE_INFINITY;
    private double lastHeadingSampleTime;
    private double lastYawRate, observedYaw = Double.NaN, observedYawRate = Double.NaN;
    private boolean headingRejected, positionRejected;
    private double nextRead, readRetryDelay = READ_RETRY_SECONDS, nextScopeRestore;
    private int rawX, rawY;
    public boolean headingValid, positionValid;
    public double forward, left, heading, yawRate;
    public String status = "等待 Pinpoint";
    public String lastFault = "";
    public int faults;
    public int badReads, recoveredReads, badReadStreak;
    public int headingJumps, recoveredHeadingJumps;
    public double lastJumpAngle, lastJumpSeconds, lastJumpRate;
    public int positionJumps;
    public double lastPositionJumpInches, lastPositionJumpSeconds;
    public int firmwareVersion;

    public PinpointPose(HardwareMap map) {
        device = map.tryGet(GoBildaPinpointDriver.class, PinPointConfig.PINPOINT);
    }

    public boolean startImu() {
        if (device == null) return false;
        started = headingValid = podsValid = positionValid = false;
        try {
            configure();
            device.resetPosAndIMU();
            started = true;
            anchored = positionAnchored = needsAnchor = positionNeedsAnchor = false;
            headingShift = 0;
            lastHeadingTime = Double.NEGATIVE_INFINITY;
            lastYawRate = yawRate = 0;
            headingRejected = positionRejected = false;
            clearReadBackoff();
            return true;
        } catch (RuntimeException failure) {
            status = "Pinpoint 初始化失败";
            return false;
        }
    }

    private void configure() {
        firmwareVersion = PinPointConfig.configurePinpoint(device);
        if (firmwareVersion >= 3) restoreBulkScope();
    }

    private void restoreBulkScope() {
        device.setBulkReadScope(Register.DEVICE_STATUS, Register.X_ENCODER_VALUE,
                Register.Y_ENCODER_VALUE, Register.X_POSITION, Register.Y_POSITION, Register.H_ORIENTATION,
                Register.H_VELOCITY);
    }

    public void update() {
        headingValid = podsValid = positionValid = false;
        observedYaw = observedYawRate = Double.NaN;
        yawRate = 0;
        if (device == null) {
            status = "找不到 " + PinPointConfig.PINPOINT;
            return;
        }
        if (!started) return;
        double readStart = Maths.time();
        if (readStart < nextRead) return;

        try {
            // A Pinpoint restart can discard the custom bulk layout. Probe it infrequently.
            if (firmwareVersion >= 3 && badReadStreak >= 4 && readStart >= nextScopeRestore) {
                nextScopeRestore = readStart + BULK_SCOPE_RETRY_SECONDS;
                restoreBulkScope();
                return;
            }
            device.update();
            DeviceStatus state = device.getDeviceStatus();
            double now = Maths.time();
            if (state == DeviceStatus.FAULT_BAD_READ) {
                failRead(state.name(), now);
                return;
            }
            if (badReadStreak > 0) {
                requireAnchorAfterReadGap(now);
                recoveredReads++;
            }
            clearReadBackoff();

            if (anchored && (state == DeviceStatus.CALIBRATING || state == DeviceStatus.NOT_READY
                    || state == DeviceStatus.FAULT_IMU_RUNAWAY)) needsAnchor = true;
            boolean ready = state == DeviceStatus.READY;
            if (!ready && state != DeviceStatus.FAULT_X_POD_NOT_DETECTED
                    && state != DeviceStatus.FAULT_Y_POD_NOT_DETECTED
                    && state != DeviceStatus.FAULT_NO_PODS_DETECTED) {
                status = state.name();
                if (anchored || state == DeviceStatus.FAULT_IMU_RUNAWAY) reportFault(status);
                return;
            }
            if (ready) {
                rawX = device.getEncoderX();
                rawY = device.getEncoderY();
            }
            double sampleTime = readStart + 0.5 * (now - readStart);
            updateHeading(sampleTime, now);
            if (!headingValid) return;
            status = needsAnchor ? "Pinpoint 需要重新对齐，回起点按 Y" : state.name();
            if (ready) updatePosition(sampleTime);
            else reportFault(state.name());
        } catch (RuntimeException unavailable) {
            headingValid = podsValid = positionValid = false;
            failRead("Pinpoint 读取失败 " + unavailable.getClass().getSimpleName(), Maths.time());
        }
    }

    private void failRead(String reason, double now) {
        if (badReadStreak == 0) RobotLog.ww("BioBuzzPinpoint", "%s", reason);
        badReads++;
        badReadStreak++;
        nextRead = now + readRetryDelay;
        readRetryDelay = Math.min(MAX_READ_RETRY_SECONDS, 2 * readRetryDelay);
        requireAnchorAfterReadGap(now);
        reportFault(reason);
    }

    private void clearReadBackoff() {
        badReadStreak = 0;
        nextRead = nextScopeRestore = Double.NEGATIVE_INFINITY;
        readRetryDelay = READ_RETRY_SECONDS;
    }

    private void requireAnchorAfterReadGap(double now) {
        if (anchored && now - lastHeadingTime >= REANCHOR_AFTER_READ_GAP) needsAnchor = true;
    }

    private void updateHeading(double sampleTime, double now) {
        double nextYaw = device.getHeading(UnnormalizedAngleUnit.RADIANS);
        double rate = device.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        observedYaw = Maths.wrap(nextYaw);
        observedYawRate = rate;
        double elapsed = sampleTime - lastHeadingSampleTime;
        double delta = nextYaw - sensorYaw;
        boolean ratesValid = Maths.finite(nextYaw, rate) && Math.abs(rate) <= MAX_YAW_RATE;
        // Both the continuous heading change and the rate integral span the same two samples.
        double expected = 0.5 * (lastYawRate + rate) * elapsed;
        double slack = HEADING_SLACK + 0.35 * Math.abs(rate - lastYawRate) * elapsed + 0.10 * Math.abs(expected);
        boolean inconsistent = !needsAnchor && Double.isFinite(lastHeadingTime)
                && Math.abs(delta - expected) > slack;
        if (!ratesValid || inconsistent) {
            headingJumps++;
            lastJumpAngle = delta;
            lastJumpSeconds = elapsed;
            lastJumpRate = rate;
            if (!headingRejected) {
                RobotLog.ww("BioBuzzPinpoint", "Heading rejected: raw=%+.2f last=%+.2f delta=%+.2f gyro=%+.2f dt=%.0f ms",
                        Math.toDegrees(observedYaw), Math.toDegrees(Maths.wrap(sensorYaw)), Math.toDegrees(delta),
                        Math.toDegrees(rate), 1000 * elapsed);
            }
            headingRejected = true;
            if (now - lastHeadingTime >= HEADING_GRACE) {
                needsAnchor = anchored;
                if (!anchored) lastHeadingTime = Double.NEGATIVE_INFINITY;
            }
            reportFault("Pinpoint 航向跳变");
            return;
        }
        if (headingRejected) recoveredHeadingJumps++;
        headingRejected = false;
        sensorYaw = nextYaw;
        lastHeadingSampleTime = sampleTime;
        lastHeadingTime = now;
        lastYawRate = rate;
        headingValid = true;
        if (!needsAnchor) {
            heading = Maths.wrap(sensorYaw + headingShift);
            yawRate = rate;
        }
    }

    private void updatePosition(double sampleTime) {
        localX = device.getPosX(DistanceUnit.INCH);
        localY = device.getPosY(DistanceUnit.INCH);
        podsValid = Maths.finite(localX, localY);
        if (!hasPositionAnchor()) {
            if (positionNeedsAnchor && !needsAnchor) status = POSITION_REANCHOR;
            else if (!podsValid) reportFault("Pinpoint 位置非有限值");
            return;
        }
        // Keep measured yaw available independently of the position check.
        double elapsed = sampleTime - previousTime;
        // A valid recovery frame includes motion since the last accepted position.
        double window = Math.min(elapsed, REANCHOR_AFTER_READ_GAP);
        double distance = Math.hypot(localX - previousX, localY - previousY);
        boolean jumped = podsValid && distance > MAX_POSITION_RATE_INCHES_PER_SECOND * window
                + POSITION_SLACK_INCHES;
        if (!podsValid || jumped) {
            if (jumped) {
                positionJumps++;
                lastPositionJumpInches = distance;
                lastPositionJumpSeconds = elapsed;
            }
            positionRejected = true;
            if (elapsed >= POSITION_GRACE) {
                positionNeedsAnchor = true;
                reportFault(POSITION_REANCHOR);
            } else status = jumped ? "Pinpoint 位置跳变" : "Pinpoint 位置非有限值";
            return;
        }
        positionRejected = false;
        double x = localX - originX, y = localY - originY;
        double c = Math.cos(headingShift), s = Math.sin(headingShift);
        forward = fieldForward + c * x - s * y;
        left = fieldLeft + s * x + c * y;
        copySample(sampleTime);
        positionValid = true;
    }

    public boolean anchor(double forwardInches, double rightInches, double fieldHeading) {
        if (!headingValid || Maths.time() - lastHeadingTime > HEADING_GRACE) return false;
        if (requiresReanchor() && !podsValid) return false;
        if (positionAnchored && !requiresReanchor() && !positionValid) return false;
        if (needsAnchor) {
            try {
                configure();
            } catch (RuntimeException failure) {
                return false;
            }
            update();
            if (!headingValid || !podsValid) return false;
        }
        forward = fieldForward = forwardInches;
        left = fieldLeft = -rightInches;
        originX = localX;
        originY = localY;
        headingShift = Maths.wrap(fieldHeading - sensorYaw);
        heading = Maths.wrap(fieldHeading);
        yawRate = lastYawRate;
        copySample(lastHeadingSampleTime);
        anchored = true;
        needsAnchor = positionNeedsAnchor = false;
        positionRejected = false;
        positionAnchored = positionValid = podsValid;
        if (positionValid) status = DeviceStatus.READY.name();
        return true;
    }

    public boolean isAnchored() { return anchored && !needsAnchor; }
    public boolean hasPositionAnchor() { return positionAnchored && !requiresReanchor(); }
    public boolean requiresReanchor() { return needsAnchor || positionNeedsAnchor; }
    /** Most recently accepted field-relative heading, in radians. */
    public double getHeading() { return heading; }
    public int encoderX() { return rawX; }
    public int encoderY() { return rawY; }
    public double sensorYaw() { return Maths.wrap(sensorYaw); }
    public double observedYaw() { return observedYaw; }
    public double observedYawRate() { return observedYawRate; }
    public boolean headingUsable() { return isAnchored() && Maths.time() - lastHeadingTime <= HEADING_GRACE; }
    public boolean recentlyLocated(double maxAge) {
        return hasPositionAnchor() && !positionRejected && Maths.time() - previousTime <= maxAge;
    }

    private void reportFault(String reason) {
        status = lastFault = reason;
        faults++;
    }

    private void copySample(double now) {
        previousX = localX;
        previousY = localY;
        previousTime = now;
    }
}
