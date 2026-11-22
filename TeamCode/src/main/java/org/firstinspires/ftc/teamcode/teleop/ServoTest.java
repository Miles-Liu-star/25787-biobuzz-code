package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest")
public class ServoTest extends LinearOpMode {

    private Servo servo;

    @Override
    public void runOpMode() {
        telemetry.addData("version:","1");

        servo = hardwareMap.get(Servo.class, "s0");

        waitForStart();

        servo.setPosition(0);

        while (opModeIsActive()) {
            servo.setPosition(0.5);
            telemetry.addData("position:",servo.getPosition());
            telemetry.update();
        }
    }
}