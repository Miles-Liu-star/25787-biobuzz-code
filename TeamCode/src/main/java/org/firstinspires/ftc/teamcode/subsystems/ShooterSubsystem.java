package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem {
    private DcMotorEx shooter0 = null;
    private DcMotorEx shooter1 = null;
    private Telemetry telemetry;

    private double targetRPM = 1000;
    private static final double SHOOTER_KP = 0.0004;
    private static final double SHOOTER_KI = 0.00001;
    private static final double SHOOTER_KD = 0.00000;
    private static final double SHOOTER_KF = 0.0000;
    private static final double SHOOTER_INTEGRAL_LIMIT = 5000.0;

    private final ElapsedTime shooterPidTimer = new ElapsedTime();
    private double shooterIntegral = 0.0;
    private double previousShooterRPM = 0.0;
    private boolean shooterPidInitialized = false;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        shooter0 = hardwareMap.get(DcMotorEx.class, "00");
        shooter1 = hardwareMap.get(DcMotorEx.class, "01");

        shooter0.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterPidTimer.reset();
    }

    public double FPD() {
        if (targetRPM <= 0) {
            shooterIntegral = 0.0;
            shooterPidInitialized = false;
            shooterPidTimer.reset();
            return 0.0;
        }

        double ticksPerRev = shooter0.getMotorType().getTicksPerRev();
        if (ticksPerRev <= 0.0) {
            telemetry.addData("Shooter PID error", "Invalid encoder ticks per revolution");
            return 0.0;
        }

        double currentRPM = Math.abs(shooter0.getVelocity()) * 60.0 / ticksPerRev;
        double error = targetRPM - currentRPM;
        double dt = shooterPidTimer.seconds();
        shooterPidTimer.reset();
        if (dt <= 0.0) {
            dt = 0.02;
        }

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

        // telemetry.addData("Shooter actual RPM", "%.0f", currentRPM);
        // telemetry.addData("Shooter power", "%.3f", power);
        return power;
    }

    public void setShooterPower(double power) {
        shooter0.setPower(power);
        shooter1.setPower(power);
    }

    public void resetShooterPID() {
        shooterIntegral = 0.0;
        shooterPidInitialized = false;
        shooterPidTimer.reset();
    }

    public void increaseTargetRPM(double amount) {
        targetRPM += amount;
    }

    public void decreaseTargetRPM(double amount) {
        targetRPM = Math.max(0, targetRPM - amount);
    }

    public void stop() {
        shooter0.setPower(0);
        shooter1.setPower(0);
    }
}