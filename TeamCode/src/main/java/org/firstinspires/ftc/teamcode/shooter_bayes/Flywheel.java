package org.firstinspires.ftc.teamcode.shooter_bayes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * 双飞轮闭环。
 *
 * 与参考实现一致：
 * - 用编码器位置差分 + 一阶低通得到速度，而不是依赖 RUN_USING_ENCODER；
 * - 前馈来自实测的“功率—转速”直线模型；
 * - PID 的微分项使用实测速度，目标阶跃不会产生微分冲击；
 * - 输出有功率爬升限制。
 */
public final class Flywheel {
    public static final double[] DEFAULT_GAINS = {0.40, 0.26, 0.012};

    private final DcMotorEx encoderMotor, followerMotor;
    private final Servo blocker;
    private final double[] model;
    public final double ticksPerRev;
    private Pid pid;
    private int previousTicks;
    private boolean hasSpeedSample = true;
    private double previousVelocity, lastMotion;
    private boolean fault;
    public double speed, target, output, stableSeconds;

    public Flywheel(HardwareMap map, ShooterCalibrationStore store) {
        this(map, store, false);
    }

    /**
     * @param fallbackToMotorTicks 为 true 时，若 04 尚未标定 RPM 比例，则退回电机自带 ticks/rev，
     *                             让正式 TeleOp 仍能闭环；04 自测请传 false。
     */
    public Flywheel(HardwareMap map, ShooterCalibrationStore store, boolean fallbackToMotorTicks) {
        encoderMotor = map.get(DcMotorEx.class, FlywheelConfig.FLYWHEEL_ENCODER_MOTOR);
        followerMotor = map.get(DcMotorEx.class, FlywheelConfig.FLYWHEEL_FOLLOWER_MOTOR);
        configure(encoderMotor, FlywheelConfig.FLYWHEEL_ENCODER_DIRECTION);
        configure(followerMotor, FlywheelConfig.FLYWHEEL_FOLLOWER_DIRECTION);
        blocker = FlywheelConfig.BLOCKER_SERVO == null
                ? null : map.tryGet(Servo.class, FlywheelConfig.BLOCKER_SERVO);
        feed(false);
        previousTicks = ticks();

        double[] fullModel = store.read(ShooterCalibrationStore.FLYWHEEL_FULL_POWER_MODEL, 0, 0, 0, 0);
        model = fullModel[3] >= 40 && fullModel[0] >= 20 && fullModel[2] > 0
                ? fullModel
                : store.read(ShooterCalibrationStore.FLYWHEEL_MODEL, 0, 0, 0);

        double storedScale = store.read(ShooterCalibrationStore.FLYWHEEL_SCALE,
                FlywheelConfig.FLYWHEEL_TICKS_PER_WHEEL_REV)[0];
        ticksPerRev = storedScale > 0
                ? storedScale
                : (fallbackToMotorTicks ? motorTicksPerRev() : 0);

        setGains(store.read(ShooterCalibrationStore.FLYWHEEL_PID, DEFAULT_GAINS));
    }

    private void configure(DcMotorEx motor, DcMotorSimple.Direction direction) {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(direction);
    }

    private double motorTicksPerRev() {
        try {
            double ticks = encoderMotor.getMotorType().getTicksPerRev();
            return ticks > 0 ? ticks : 0;
        } catch (RuntimeException ignored) {
            return 0;
        }
    }

    public int ticks() {
        return encoderMotor.getCurrentPosition();
    }

    public double reference() {
        return model[0];
    }

    public double fullPowerSpeed() {
        return model.length == 4 ? model[3] : 0;
    }

    public double estimatedPowerForSpeed(double speed) {
        return model[1] + model[2] * speed / reference();
    }

    public double estimatedSpeedAtPower(double power) {
        return calibrated() ? reference() * Math.max(0, power - model[1]) / model[2] : 0;
    }

    public boolean calibrated() {
        return reference() >= 20 && model[2] > 0;
    }

    public boolean rpmCalibrated() {
        return calibrated() && ticksPerRev > 0;
    }

    public boolean faulted() {
        return fault;
    }

    public boolean ready() {
        return !fault && target > 0 && stableSeconds >= 0.25;
    }

    public boolean hasBlocker() {
        return blocker != null;
    }

    public void setGains(double[] gains) {
        pid = new Pid(gains);
    }

    /** Discard the old encoder baseline before resuming after a sampling gap. */
    public void resetSpeedSample() {
        hasSpeedSample = false;
        speed = previousVelocity = 0;
    }

    public void sample(double dt) {
        try {
            int tick = ticks();
            int delta = hasSpeedSample ? tick - previousTicks : 0;
            if (delta != 0) lastMotion = Maths.time();
            previousTicks = tick;
            hasSpeedSample = true;
            previousVelocity = speed;
            speed += dt / (0.075 + dt) * (Math.abs(delta / dt) - speed);
            if (output > 0.22 && Maths.time() - lastMotion > 0.85) fault = true;
        } catch (RuntimeException failure) {
            fault = true;
        }
        if (fault) stop();
    }

    public void requestRpm(double rpm, double dt) {
        if (!rpmCalibrated()) {
            stop();
            return;
        }
        regulate(rpm * ticksPerRev / 60, dt);
    }

    public void regulate(double ticksPerSecond, double dt) {
        if (fault || !calibrated() || ticksPerSecond <= 0) {
            stop();
            return;
        }
        if (Math.abs(target - ticksPerSecond) > target * FlywheelConfig.FLYWHEEL_SPEED_TOLERANCE * 0.5) {
            stableSeconds = 0;
        }
        target = ticksPerSecond;
        double error = (target - speed) / reference();
        double acceleration = (speed - previousVelocity) / (dt * reference());
        double ff = estimatedPowerForSpeed(target);
        double min = Math.max(0, output - 4.0 * dt);
        double max = Math.min(FlywheelConfig.FLYWHEEL_MAX_POWER,
                output + FlywheelConfig.FLYWHEEL_RAMP * dt);
        power(pid.update(error, acceleration, ff, dt, min, max), dt);
        stableSeconds = Math.abs(target - speed) <= target * FlywheelConfig.FLYWHEEL_SPEED_TOLERANCE
                ? stableSeconds + dt : 0;
    }

    public void power(double desired, double dt) {
        double next = fault ? 0 : Maths.clip(desired, 0, FlywheelConfig.FLYWHEEL_MAX_POWER);
        next = Math.min(next, output + FlywheelConfig.FLYWHEEL_RAMP * dt);
        if (output <= 0.22 && next > 0.22) lastMotion = Maths.time();
        output = next;
        encoderMotor.setPower(next);
        followerMotor.setPower(next);
    }

    public boolean feed(boolean fire) {
        boolean released = blocker != null && fire && ready();
        if (blocker != null) {
            blocker.setPosition(released ? FlywheelConfig.SB_FIRE : FlywheelConfig.SB_IDLE);
        }
        return released;
    }

    public void stop() {
        feed(false);
        encoderMotor.setPower(0);
        followerMotor.setPower(0);
        target = output = stableSeconds = 0;
        pid.reset();
    }
}
