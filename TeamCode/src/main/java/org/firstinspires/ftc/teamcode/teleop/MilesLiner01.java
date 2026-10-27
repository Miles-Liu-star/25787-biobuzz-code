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

@TeleOp(name = "Headless", group = "Advanced")
public class MilesLiner01 extends OpMode {
    private DcMotor intake = null;
    private DcMotor middleTurn = null;
    private Servo pitch = null;
    private Servo switch_model = null;
    private IMU imu = null;
    private DriveSubsystem driveSubsystem = null;
    private ShooterSubsystem shooterSubsystem = null;
    private boolean was_press_dpadup = false;
    private boolean was_press_dpaddown = false;

    private boolean pitch_model = false;// true为高 false为低
    private boolean was_press_pitch = false;
    private boolean ball_model = false;// true 为大球 false 为小球
    private boolean was_press_switch = false;

    private final ElapsedTime yawPidTimer = new ElapsedTime();

    private double yawIntegral = 0;
    private double previousYawError = 0;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "03");
        middleTurn = hardwareMap.get(DcMotor.class, "02");
        pitch = hardwareMap.get(Servo.class, "s5");
        switch_model = hardwareMap.get(Servo.class, "s00");

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

        yawPidTimer.reset();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            imu.resetYaw();
        }

        if (gamepad1.right_bumper) {
            intake.setPower(1);
        } else if (gamepad1.left_bumper) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        if (gamepad1.right_trigger > 0.5) {
            double shooterPower = shooterSubsystem.FPD();
            shooterSubsystem.setShooterPower(shooterPower);
        } else {
            shooterSubsystem.setShooterPower(0);
            shooterSubsystem.resetShooterPID();
        }

        if (gamepad1.dpad_up && !was_press_dpadup) {
            shooterSubsystem.increaseTargetRPM(50);
            was_press_dpadup = true;
        }
        if (gamepad1.dpad_down && !was_press_dpaddown) {
            shooterSubsystem.decreaseTargetRPM(50);
            was_press_dpaddown = true;
        }
        was_press_dpadup = gamepad1.dpad_up;
        was_press_dpaddown = gamepad1.dpad_down;


        if (gamepad1.a && !was_press_pitch) {
            pitch_model = !pitch_model;
            pitch.setPosition(pitch_model ? 1.0 : 0.0);
            was_press_pitch = true;
        }
        was_press_pitch = gamepad1.a;

        if (gamepad1.x && !was_press_switch) {
            ball_model = !ball_model;
            switch_model.setPosition(ball_model ? 1.0 : 0.0);
            was_press_switch = true;
        }
        was_press_switch = gamepad1.x;

        middleTurn.setPower(-gamepad1.left_trigger);
        driveSubsystem.update(gamepad1);
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