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
        pitch_big = hardwareMap.get(Servo.class, "s5");
        pitch_small_left = hardwareMap.get(Servo.class, "s00");
        pitch_small_right = hardwareMap.get(Servo.class, "s0");
    }

    public void update()
    {
    }
}
