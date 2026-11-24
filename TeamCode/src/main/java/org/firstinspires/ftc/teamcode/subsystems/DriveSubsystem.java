package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem {
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private final Telemetry telemetry;

    private static final double DEADBAND = 0.05;
    private static final double ACCEL_SLEW_RATE = 5;
    private static final double DECEL_SLEW_RATE = 5.0;
    private double lastDrivePower = 0.0;
    private double lastStrafePower = 0.0;
    private double lastTurnPower = 0.0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        LeftFront = hardwareMap.get(DcMotor.class, "0");
        LeftBack = hardwareMap.get(DcMotor.class, "1");
        RightFront = hardwareMap.get(DcMotor.class, "2");
        RightBack = hardwareMap.get(DcMotor.class, "3");

        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        loopTimer.reset();
    }

    private double applyDeadband(double value) {
        return Math.abs(value) > DEADBAND ? value : 0.0;
    }

    public void update(Gamepad gamepad1, double heading, boolean is_use_aim, double AimTurn) {
        double rawY = -gamepad1.left_stick_y;
        double rawX = gamepad1.left_stick_x;
        double rawTurn = gamepad1.right_stick_x;

        double y = applyDeadband(rawY);
        double x = applyDeadband(rawX);
        double turn = is_use_aim? AimTurn : applyDeadband(rawTurn);

        y = Math.pow(y, 3);
        x = Math.pow(x, 3);
        turn = Math.pow(turn, 3);

        double botHeading = heading;
        double headingOffset = -botHeading;
        double rotatedX = x * Math.cos(headingOffset) - y * Math.sin(headingOffset);
        double rotatedY = x * Math.sin(headingOffset) + y * Math.cos(headingOffset);
        double targetDrive = rotatedY;
        double targetStrafe = rotatedX;
        double targetTurn = turn;

        double driveSlewRate = Math.abs(targetDrive) > Math.abs(lastDrivePower) ? ACCEL_SLEW_RATE : DECEL_SLEW_RATE;
        double strafeSlewRate = Math.abs(targetStrafe) > Math.abs(lastStrafePower) ? ACCEL_SLEW_RATE : DECEL_SLEW_RATE;
        double turnSlewRate = Math.abs(targetTurn) > Math.abs(lastTurnPower) ? ACCEL_SLEW_RATE : DECEL_SLEW_RATE;

        double elapsedTime = loopTimer.seconds();
        loopTimer.reset();

        double driveMaxChange = driveSlewRate * elapsedTime;
        double strafeMaxChange = strafeSlewRate * elapsedTime;
        double turnMaxChange = turnSlewRate * elapsedTime;

        double currentDrive = lastDrivePower
                + Range.clip(targetDrive - lastDrivePower, -driveMaxChange, driveMaxChange);
        double currentStrafe = lastStrafePower
                + Range.clip(targetStrafe - lastStrafePower, -strafeMaxChange, strafeMaxChange);
        double currentTurn = lastTurnPower
                + Range.clip(targetTurn - lastTurnPower, -turnMaxChange, turnMaxChange);

        lastDrivePower = currentDrive;
        lastStrafePower = currentStrafe;
        lastTurnPower = currentTurn;

        double frontLeftPower = currentDrive + currentStrafe + currentTurn;
        double backLeftPower = currentDrive - currentStrafe + currentTurn;
        double frontRightPower = currentDrive - currentStrafe - currentTurn;
        double backRightPower = currentDrive + currentStrafe - currentTurn;

        double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        telemetry.addData("frontLeftPower", "%5.1f", frontLeftPower);
        telemetry.addData("frontRightPower", "%5.1f", frontRightPower);
        telemetry.addData("backLeftPower", "%5.1f", backLeftPower);
        telemetry.addData("backRightPower", "%5.1f", backRightPower);

        LeftFront.setPower(frontLeftPower);
        LeftBack.setPower(backLeftPower);
        RightFront.setPower(frontRightPower);
        RightBack.setPower(backRightPower);
    }

    public void stop() {
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);
    }
}
