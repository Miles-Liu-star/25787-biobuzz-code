package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PitchSubsystem {
    private final Telemetry telemetry;
    private Servo pitch_big = null;
    private Servo pitch_small_left = null;
    private Servo pitch_small_right = null;

    public PitchSubsystem(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.telemetry = telemetry;
        pitch_big = hardwareMap.get(Servo.class, "s0");
        pitch_small_left = hardwareMap.get(Servo.class, "s2");
        pitch_small_right = hardwareMap.get(Servo.class, "s3");
    }

    public void update()
    {
        pitch_big.setPosition(0);
        pitch_small_left.setPosition(0.5);
        pitch_small_right.setPosition(0.5);
    }
}
