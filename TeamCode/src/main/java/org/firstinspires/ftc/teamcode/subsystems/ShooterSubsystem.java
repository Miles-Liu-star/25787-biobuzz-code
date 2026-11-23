package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shooter_bayes.Flywheel;
import org.firstinspires.ftc.teamcode.shooter_bayes.FlywheelConfig;
import org.firstinspires.ftc.teamcode.shooter_bayes.ShooterCalibrationStore;

public class ShooterSubsystem {
    private DcMotorEx shooter0 = null;
    private DcMotorEx shooter1 = null;
    private Telemetry telemetry;
    /** 标定完成后使用贝叶斯搜索生成的闭环控制器；未标定时为空并走旧 PIDF。 */
    private Flywheel calibratedFlywheel;

    private double targetRPM = 3600;
    private static final double SHOOTER_KP = 0.0004;
    private static final double SHOOTER_KI = 0.000005;
    private static final double SHOOTER_KD = 0.000002;
    private static final double SHOOTER_KF = 0.0000;
    private static final double SHOOTER_INTEGRAL_LIMIT = 5000.0;

    private final ElapsedTime shooterPidTimer = new ElapsedTime();
    private double shooterIntegral = 0.0;
    private double previousShooterRPM = 0.0;
    private boolean shooterPidInitialized = false;
    private boolean running = false;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        shooter0 = hardwareMap.get(DcMotorEx.class, "00");
        shooter1 = hardwareMap.get(DcMotorEx.class, "01");

        shooter0.setDirection(FlywheelConfig.FLYWHEEL_ENCODER_DIRECTION);
        shooter1.setDirection(FlywheelConfig.FLYWHEEL_FOLLOWER_DIRECTION);

        shooter0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterPidTimer.reset();

        // 读取 04 自测飞轮保存的模型和 PID。没有完整标定结果时保留原控制逻辑。
        ShooterCalibrationStore store = new ShooterCalibrationStore(hardwareMap.appContext);
        Flywheel candidate = new Flywheel(hardwareMap, store, false);
        if (candidate.rpmCalibrated()) {
            calibratedFlywheel = candidate;
        } else {
            candidate.stop();
            shooter0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /** Run one speed-control tick, or stop once when the shooter is disabled. */
    public void update(boolean enabled) {
        telemetry.addData("Shooter target RPM", "%.0f", targetRPM);
        if (!enabled || targetRPM <= 0.0) {
            if (running) stop();
            telemetry.addData("Shooter", "Off");
            return;
        }

        double dt = running ? shooterPidTimer.seconds() : 0.02;
        if (!running && calibratedFlywheel != null) {
            calibratedFlywheel.resetSpeedSample();
        }
        running = true;
        shooterPidTimer.reset();

        if (calibratedFlywheel != null) {
            dt = Range.clip(dt, 0.005, 0.05);
            if (!calibratedFlywheel.faulted()) {
                calibratedFlywheel.sample(dt);
                if (!calibratedFlywheel.faulted()) {
                    calibratedFlywheel.requestRpm(targetRPM, dt);
                }
            }
            telemetry.addData("Shooter", calibratedFlywheel.faulted() ? "Flywheel fault" : "Calibrated PID");
            telemetry.addData("Shooter flywheel RPM", "%.0f",
                    calibratedFlywheel.speed * 60.0 / calibratedFlywheel.ticksPerRev);
            telemetry.addData("Shooter power", "%.3f", calibratedFlywheel.output);
        } else {
            updateLegacyPid(dt > 0.0 ? dt : 0.02);
        }
    }

    private void updateLegacyPid(double dt) {
        double ticksPerRev = shooter0.getMotorType().getTicksPerRev();
        if (!Double.isFinite(ticksPerRev) || ticksPerRev <= 0.0) {
            stop();
            telemetry.addData("Shooter PID error", "Invalid encoder ticks per revolution");
            return;
        }

        double currentRPM = Math.abs(shooter0.getVelocity()) * 60.0 / ticksPerRev;
        if (!Double.isFinite(currentRPM)) {
            stop();
            telemetry.addData("Shooter PID error", "Invalid encoder velocity");
            return;
        }
        double error = targetRPM - currentRPM;

        double derivative = 0.0;
        if (shooterPidInitialized) {
            derivative = -(currentRPM - previousShooterRPM) / dt;
        }
        previousShooterRPM = currentRPM;
        shooterPidInitialized = true;

        shooterIntegral = Range.clip(
                shooterIntegral + error * dt,
                -SHOOTER_INTEGRAL_LIMIT,
                SHOOTER_INTEGRAL_LIMIT);

        double feedForward = SHOOTER_KF * targetRPM;
        double power = feedForward
                + SHOOTER_KP * error
                + SHOOTER_KI * shooterIntegral
                + SHOOTER_KD * derivative;
        power = Range.clip(power, 0.0, 1.0);

        shooter0.setPower(power);
        shooter1.setPower(power);
        telemetry.addData("Shooter", "Legacy PID");
        telemetry.addData("Shooter motor RPM", "%.0f", currentRPM);
        telemetry.addData("Shooter power", "%.3f", power);
    }

    public void increaseTargetRPM(double amount) {
        targetRPM += amount;
    }

    public void decreaseTargetRPM(double amount) {
        targetRPM = Math.max(0, targetRPM - amount);
    }

    public void stop() {
        if (calibratedFlywheel != null) {
            calibratedFlywheel.stop();
        } else {
            shooter0.setPower(0);
            shooter1.setPower(0);
        }
        running = false;
        shooterIntegral = 0.0;
        shooterPidInitialized = false;
        shooterPidTimer.reset();
    }
}
