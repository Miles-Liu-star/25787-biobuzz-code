package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Set;

@TeleOp(name = "MotorDetectTest", group = "Test")
public class DCmotorTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry.addLine("Detected Motors:");

        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {

            Set<String> names = hardwareMap.getNamesOf(motor);

            telemetry.addData("Motor Name", names);
        }

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}