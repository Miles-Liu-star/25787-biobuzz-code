
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem {

    private DcMotor LeftFront;
    private DcMotor LeftBack;
    private DcMotor RightFront;
    private DcMotor RightBack;

    private IMU imu;
    private Telemetry telemetry;

    private static final double DEADBAND = 0.05;
    private static final double ACCEL_SLEW_RATE = 5.0;
    private static final double DECEL_SLEW_RATE = 5.0;

    private double lastDrivePower = 0.0;
    private double lastStrafePower = 0.0;
    private double lastTurnPower = 0.0;

    private final ElapsedTime loopTimer = new ElapsedTime();

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // 初始化四个底盘电机
        LeftFront = hardwareMap.get(DcMotor.class, "0");
        LeftBack = hardwareMap.get(DcMotor.class, "1");
        RightFront = hardwareMap.get(DcMotor.class, "2");
        RightBack = hardwareMap.get(DcMotor.class, "3");

        // 初始化电机方向
        LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // 设置零功率模式
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 初始化 IMU
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        loopTimer.reset();
    }

    // 摇杆死区
    private double applyDeadband(double value) {
        return Math.abs(value) > DEADBAND ? value : 0.0;
    }

    // 更新底盘驱动
    public void update(Gamepad gamepad) {

        double rawY = -gamepad.left_stick_y;
        double rawX = gamepad.left_stick_x;
        double rawTurn = gamepad.right_stick_x;

        double y = applyDeadband(rawY);
        double x = applyDeadband(rawX);
        double turn = applyDeadband(rawTurn);

        // 三次方曲线：精细控制低速，大幅输入时仍有较大输出
        y = Math.pow(y, 3);
        x = Math.pow(x, 3);
        turn = Math.pow(turn, 3);

        // Y 键重置 IMU 航向
        if (gamepad.y) {
            imu.resetYaw();
        }

        // 机器人航向补偿，实现相对于场地的方向控制
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double headingOffset = -botHeading;
        double rotatedX = x * Math.cos(headingOffset) - y * Math.sin(headingOffset);
        double rotatedY = x * Math.sin(headingOffset) + y * Math.cos(headingOffset);
        double targetDrive = rotatedY;
        double targetStrafe = rotatedX;
        double targetTurn = turn;

        // 加速、减速变化率限制
        double driveSlewRate = Math.abs(targetDrive) > Math.abs(lastDrivePower) ? ACCEL_SLEW_RATE : DECEL_SLEW_RATE;
        double strafeSlewRate = Math.abs(targetStrafe) > Math.abs(lastStrafePower) ? ACCEL_SLEW_RATE : DECEL_SLEW_RATE;
        double turnSlewRate = Math.abs(targetTurn) > Math.abs(lastTurnPower) ? ACCEL_SLEW_RATE : DECEL_SLEW_RATE;

        double elapsedTime = loopTimer.seconds();
        loopTimer.reset();

        double driveMaxChange = driveSlewRate * elapsedTime;
        double strafeMaxChange = strafeSlewRate * elapsedTime;
        double turnMaxChange = turnSlewRate * elapsedTime;

        double currentDrive = lastDrivePower + Range.clip(targetDrive - lastDrivePower, -driveMaxChange, driveMaxChange);
        double currentStrafe = lastStrafePower + Range.clip(targetStrafe - lastStrafePower, -strafeMaxChange, strafeMaxChange);
        double currentTurn = lastTurnPower + Range.clip(targetTurn - lastTurnPower, -turnMaxChange, turnMaxChange);

        lastDrivePower = currentDrive;
        lastStrafePower = currentStrafe;
        lastTurnPower = currentTurn;

        // Mecanum 四轮运动
        double frontLeftPower = currentDrive + currentStrafe + currentTurn;
        double backLeftPower = currentDrive - currentStrafe + currentTurn;
        double frontRightPower = currentDrive - currentStrafe - currentTurn;
        double backRightPower = currentDrive + currentStrafe - currentTurn;

        // 归一化
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                                   Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        // 输出电机功率
        LeftFront.setPower(frontLeftPower);
        LeftBack.setPower(backLeftPower);
        RightFront.setPower(frontRightPower);
        RightBack.setPower(backRightPower);

        // 遥测
        telemetry.addData("frontLeftPower", "%.3f", frontLeftPower);
        telemetry.addData("frontRightPower", "%.3f", frontRightPower);
        telemetry.addData("backLeftPower", "%.3f", backLeftPower);
        telemetry.addData("backRightPower", "%.3f", backRightPower);
        telemetry.addData("Heading", "%.1f",
                Math.toDegrees(botHeading));
    }

    // 停止底盘
    public void stop() {
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        lastDrivePower = 0.0;
        lastStrafePower = 0.0;
        lastTurnPower = 0.0;

        loopTimer.reset();
    }
}