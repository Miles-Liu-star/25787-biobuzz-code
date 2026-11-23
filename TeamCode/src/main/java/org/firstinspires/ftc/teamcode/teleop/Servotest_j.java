package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "手柄舵机调试器 (servo-1)", group = "Test")
public class Servotest_j extends LinearOpMode {

    private static final String SERVO_NAME = "s2";

    // 步进精度定义
    private static final double FINE_STEP = 0.005;   // 十字键上下：微调 (0.5%)
    private static final double COARSE_STEP = 0.05;  // 十字键左右：粗调 (5.0%)

    @Override
    public void runOpMode() {
        ServoImplEx servo;

        try {
            servo = hardwareMap.get(ServoImplEx.class, SERVO_NAME);
        } catch (Exception e) {
            telemetry.addData("【错误】找不到舵机", SERVO_NAME);
            telemetry.addLine("请确认 RC 配置中的名称是否完全为: " + SERVO_NAME);
            telemetry.update();
            while (!isStopRequested()) { idle(); }
            return;
        }

        double targetPos = 0.5; // 默认中位开局
        boolean pwmEnabled = true;

        // 边沿检测变量（按一次动一步，防止长按瞬间跑飞）
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        boolean lastDpadLeft = false;
        boolean lastDpadRight = false;
        boolean lastY = false;

        // INIT 阶段默认不发暴力极限，先居中
        servo.setPosition(targetPos);

        telemetry.addLine(">>> 舵机 " + SERVO_NAME + " 已就绪 <<<");
        telemetry.addLine("点击 [START] 开始调试");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. 十字键按键边缘触发步进
            if (gamepad1.dpad_up && !lastDpadUp) {
                targetPos += FINE_STEP;
            } else if (gamepad1.dpad_down && !lastDpadDown) {
                targetPos -= FINE_STEP;
            }

            if (gamepad1.dpad_right && !lastDpadRight) {
                targetPos += COARSE_STEP;
            } else if (gamepad1.dpad_left && !lastDpadLeft) {
                targetPos -= COARSE_STEP;
            }

            // 2. 经典预设快捷键（定点跳转）
            if (gamepad1.a) targetPos = 0.0; // 极小值
            if (gamepad1.x) targetPos = 0.5; // 中位
            if (gamepad1.b) targetPos = 1.0; // 极大值

            // 限制范围在 0.0 ~ 1.0 之间
            targetPos = Range.clip(targetPos, 0.0, 1.0);

            // 3. Y 键切换 PWM 输出（断电卸力模式，防止堵转烧舵机）
            if (gamepad1.y && !lastY) {
                pwmEnabled = !pwmEnabled;
                if (pwmEnabled) {
                    servo.setPwmEnable();
                } else {
                    servo.setPwmDisable();
                }
            }

            // 4. 只有在 PWM 使能时才输出位置
            if (pwmEnabled) {
                servo.setPosition(targetPos);
            }

            // 更新按键状态
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;
            lastDpadLeft = gamepad1.dpad_left;
            lastDpadRight = gamepad1.dpad_right;
            lastY = gamepad1.y;

            // 5. 屏幕实时显示
            telemetry.addLine("======= 舵机调试面板: " + SERVO_NAME + " =======");
            telemetry.addData("【当前输出数值 (Position)】", ">>> %.4f <<<", targetPos);
            telemetry.addData("PWM 供电状态", pwmEnabled ? "锁定加力 (ON)" : "【卸力自由转动 (OFF)】");
            telemetry.addLine("----------------------------------");
            telemetry.addData("参考等效 180° 角度", "%.1f°", targetPos * 180.0);
            telemetry.addData("参考等效 270° 角度", "%.1f°", targetPos * 270.0);
            telemetry.addData("参考等效 300° 角度", "%.1f°", targetPos * 300.0);
            telemetry.addLine("----------------------------------");
            telemetry.addLine("操作提示:");
            telemetry.addLine("• 十字键 ↑/↓ : ±0.005 微调");
            telemetry.addLine("• 十字键 ←/→ : ±0.05 粗调");
            telemetry.addLine("• 按 A/X/B   : 直达 0.0 / 0.5(中位) / 1.0");
            telemetry.addLine("• 按 Y 键    : 切换供电/卸力 (听到滋滋声堵转立刻按 Y)");
            telemetry.update();
        }
    }
}
