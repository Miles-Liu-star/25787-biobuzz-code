package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PitchSubsystem;
import org.firstinspires.ftc.teamcode.shooter_bayes.FlywheelConfig;

/*
* 常用Terminal指令
 & "D:\AndroidStudio-SDK\platform-tools\adb.exe" connect 192.168.43.1:5555
 & "D:\AndroidStudio-SDK\platform-tools\adb.exe" disconnect 192.168.43.1:5555
 & "D:\AndroidStudio-SDK\platform-tools\adb.exe" devices
*/
@TeleOp(name = "shooter测试", group = "Advanced")
public class shooterTest extends OpMode {

    private DcMotorEx shooter1 = null;
    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "00");

        shooter1.setDirection(FlywheelConfig.FLYWHEEL_ENCODER_DIRECTION);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        shooter1.setPower(0.5);
        telemetry.addData("currentVelocity:", shooter1.getVelocity());
    }

    @Override
    public void stop() {
        if(shooter1 != null)
            shooter1.setPower(0);
    }
}
