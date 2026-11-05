package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SwitchTest", group = "Test")
public class test extends OpMode {

    private Servo switch_model_big = null;
    private Servo switch_model_small = null;

    // 初始舵机位置
    private double switchPosition = 0.5;

    // 按键状态记录（用于实现单次按压触发）
    private boolean was_press_dpadup = false;
    private boolean was_press_dpaddown = false;

    @Override
    public void init() {

        switch_model_big = hardwareMap.get(Servo.class, "s1");
        switch_model_small = hardwareMap.get(Servo.class, "s2");

        // 初始化位置
        switch_model_big.setPosition(switchPosition);
        //switch_model_small.setPosition(switchPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Switch Position", switchPosition);
        telemetry.update();
    }

    @Override
    public void loop() {

        // D-pad Up：增加 0.05
        if (gamepad1.dpad_up && !was_press_dpadup) {
            switchPosition = Math.min(switchPosition+0.01, 1.0);
        }

        // D-pad Down：减少 0.05
        if (gamepad1.dpad_down && !was_press_dpaddown) {
            switchPosition = Math.max(switchPosition-0.01, 0.0);
        }

        if (gamepad1.a)
        {
            switch_model_big.setPosition(1);
        }
        else
        {
            switch_model_big.setPosition(0);
        }

        // 更新按键状态
        was_press_dpadup = gamepad1.dpad_up;
        was_press_dpaddown = gamepad1.dpad_down;

        // 更新 Servo 位置
        switch_model_big.setPosition(switchPosition);
        //switch_model_small.setPosition(switchPosition);

        // 实时显示数据
        telemetry.addData("Switch Position", "%.2f", switchPosition);
        telemetry.addData("D-pad Up", "Increase 0.05");
        telemetry.addData("D-pad Down", "Decrease 0.05");
        telemetry.update();
    }
}