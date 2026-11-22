package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pinpoint.PinpointXYReader;
import org.firstinspires.ftc.teamcode.pinpoint.PinPointConfig;
import org.firstinspires.ftc.teamcode.subsystems.AimSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PitchSubsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
    private Servo shovel_lr = null;//不确定，需要改！！！！
    private IMU imu = null;
    private DriveSubsystem driveSubsystem = null;
    private ShooterSubsystem shooterSubsystem = null;
    private PinpointXYReader pinpoint = null;
    private AimSubsystem autoAim = null;
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
    private boolean shovel_model = true;// true 为高 false 为低
    private boolean was_press_shovel = false;
    private boolean anchored = false;
    private boolean pinpointStarted = false;
    private boolean wasPressY = false;

    private final ElapsedTime yawPidTimer = new ElapsedTime();
    private ElapsedTime shooterTimer = new ElapsedTime();
    private final ElapsedTime pinpointStartTimer = new ElapsedTime();
    private final double startX = PinPointConfig.CENTER_FROM_REAR_EDGE_MM;
    private final double startY = -PinPointConfig.CENTER_FROM_LEFT_EDGE_MM;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "02");
        middleTurn = hardwareMap.get(DcMotor.class, "03");
        switch_model_big = hardwareMap.get(Servo.class, "s01");
        switch_model_small = hardwareMap.get(Servo.class, "s1");
        shovel_lr = hardwareMap.get(Servo.class, "s00");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        middleTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        yawPidTimer.reset();
        shooterTimer.reset();

        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        pinpoint = new PinpointXYReader(hardwareMap);
        autoAim = new AimSubsystem();

        pinpointStarted = pinpoint.start();
        pinpointStartTimer.reset();
        pitch = new PitchSubsystem(hardwareMap, telemetry);

        telemetry.addData("version", "0115");
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        updatePinpoint();
        double yawRateRadiansPerSecond = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;

        if (gamepad1.y && !wasPressY) {
            anchored = pinpoint.setFieldOrigin(startX, startY, 0);
            imu.resetYaw();
        }
        wasPressY = gamepad1.y;

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
            middleTurn.setPower(-0.9);
        }
        else
        {
            middleTurn.setPower(0);
        }

        if(gamepad1.a && !was_press_shovel)
        {
            shovel_model = !shovel_model;
        }
        was_press_shovel = gamepad1.a;
        if(shovel_model)
        {
            shovel_lr.setPosition(0);
        }
        else
        {
            shovel_lr.setPosition(0.49);
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
            switch_model_big.setPosition(ball_model ? 0.19 : 0.0);
            switch_model_small.setPosition(ball_model ? 0.0 : 0.09);
        }
        was_press_switch = gamepad1.x;

        driveSubsystem.update(gamepad1, pinpoint.getHeading());
        pitch.update();

        telemetry.update();
    }

    private void updatePinpoint() {
        if (!pinpointStarted && pinpointStartTimer.seconds() >= 1.0) {
            pinpointStarted = pinpoint.start();
            pinpointStartTimer.reset();
        }

        pinpoint.update();
        if (pinpoint.needsReanchor()) anchored = false;

        // Automatically establish the configured start pose once Pinpoint is ready.
        // If localization later requires re-anchoring, the driver must press Y at the start point.
        if (pinpointStarted && !anchored && !pinpoint.needsReanchor()) {
            anchored = pinpoint.setFieldOrigin(startX, startY, 0);
        }
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
