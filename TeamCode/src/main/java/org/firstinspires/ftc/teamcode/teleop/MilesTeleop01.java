package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PitchSubsystem;

/*
* 常用Terminal指令
 & "D:\AndroidStudio-SDK\platform-tools\adb.exe" connect 192.168.43.1:5555
 & "D:\AndroidStudio-SDK\platform-tools\adb.exe" disconnect 192.168.43.1:5555
 & "D:\AndroidStudio-SDK\platform-tools\adb.exe" devices
*/
@TeleOp(name = "无头手动", group = "Advanced")
public class MilesTeleop01 extends OpMode {
    private DcMotor intake = null;
    private DcMotor middleTurn = null;
    private PitchSubsystem pitch = null;
    private Servo switch_model_big = null;
    private Servo switch_model_small = null;
    private IMU imu = null;
    private DriveSubsystem driveSubsystem = null;
    private ShooterSubsystem shooterSubsystem = null;
    private boolean was_press_dpadup = false;
    private boolean was_press_dpaddown = false;
    private boolean ball_model = false;// true 为开 false 为关
    private boolean was_press_switch = false;
    private boolean is_open_shooter = false;
    private boolean was_press_shooter = false;
    private boolean is_open_intake = false;
    private boolean was_press_intake = false;
    private boolean is_open_midturn = false;
    private boolean was_press_midturn = false;

    private final ElapsedTime yawPidTimer = new ElapsedTime();
    private ElapsedTime shooterTimer = new ElapsedTime();

    private double yawIntegral = 0;
    private double previousYawError = 0;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "03");
        middleTurn = hardwareMap.get(DcMotor.class, "02");
        switch_model_big = hardwareMap.get(Servo.class, "s1");
        switch_model_small = hardwareMap.get(Servo.class, "s2");

        middleTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry, imu);
        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        pitch = new PitchSubsystem(hardwareMap, telemetry);

        yawPidTimer.reset();
        shooterTimer.reset();

        telemetry.addData("version", "01");
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            imu.resetYaw();
        }

        if(gamepad1.right_bumper && !was_press_intake)
        {
            is_open_intake = !is_open_intake;
        }
        was_press_intake = gamepad1.right_bumper;
        if (is_open_intake) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

        if(gamepad1.left_bumper && !was_press_midturn)
        {
            is_open_midturn = !is_open_midturn;
        }
        was_press_midturn = gamepad1.left_bumper;
        if(is_open_midturn)
        {
            middleTurn.setPower(1);
        }
        else
        {
            middleTurn.setPower(0);
        }


        if(gamepad1.b && !was_press_shooter)
        {
            is_open_shooter = !is_open_shooter;
        }
        was_press_shooter = gamepad1.b;
        if (is_open_shooter) {
            double shooterPower = shooterSubsystem.FPD();
            shooterSubsystem.setShooterPower(shooterPower);
        } else {
            shooterSubsystem.setShooterPower(0);
            shooterSubsystem.resetShooterPID();
        }

        if (gamepad1.dpad_up && !was_press_dpadup) {
            shooterSubsystem.increaseTargetRPM(25);
            was_press_dpadup = true;
        }
        if (gamepad1.dpad_down && !was_press_dpaddown) {
            shooterSubsystem.decreaseTargetRPM(25);
            was_press_dpaddown = true;
        }
        was_press_dpadup = gamepad1.dpad_up;
        was_press_dpaddown = gamepad1.dpad_down;

        if (gamepad1.x && !was_press_switch) {
            ball_model = !ball_model;
            switch_model_big.setPosition(ball_model ? 1.0 : 0.0);
            switch_model_small.setPosition(ball_model ? 0.3 : 0.2);
        }
        was_press_switch = gamepad1.x;

        driveSubsystem.update(gamepad1);
        pitch.update();
    }

    @Override
    public void stop() {
        if (driveSubsystem != null)
            driveSubsystem.stop();
        if (shooterSubsystem != null)
            shooterSubsystem.stop();
        if (intake != null)
            intake.setPower(0);
        if (middleTurn != null)
            middleTurn.setPower(0);
    }
}