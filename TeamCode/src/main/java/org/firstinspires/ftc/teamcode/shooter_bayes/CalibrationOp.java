package org.firstinspires.ftc.teamcode.shooter_bayes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * 自测 OpMode 基类：负责确认提示、计时循环、保存参数和显示结果。
 * 由 04 飞轮自测使用。
 */
abstract class CalibrationOp extends LinearOpMode {
    ShooterCalibrationStore store;
    private boolean motionAborted;
    private double nextDisplay;

    @Override
    public final void runOpMode() throws InterruptedException {
        store = new ShooterCalibrationStore(hardwareMap.appContext);
        telemetry.addLine(intro());
        telemetry.addLine("请在 Driver Station 点击 PLAY 开始。手柄 A 确认，B 跳过/停止当前动作。");
        telemetry.update();
        waitForStart();
        if (!opModeIsActive()) return;
        try {
            calibrate();
        } catch (RuntimeException failure) {
            telemetry.log().add("自测中断：%s", failure.getMessage());
        }
        while (opModeIsActive()) {
            telemetry.addLine("本轮结束。请查看日志，再按 STOP。");
            showResults();
            telemetry.update();
            sleep(100);
        }
    }

    abstract String intro();

    abstract void calibrate() throws InterruptedException;

    void showResults() {}

    boolean confirm(String instructions) throws InterruptedException {
        return confirm(instructions, () -> {});
    }

    boolean confirm(String instructions, Runnable display) throws InterruptedException {
        while (opModeIsActive() && (gamepad1.a || gamepad1.b)) sleep(20);
        while (opModeIsActive()) {
            telemetry.addLine(instructions);
            telemetry.addLine("A 确认 / B 跳过");
            display.run();
            telemetry.update();
            if (gamepad1.b) return false;
            if (gamepad1.a) {
                motionAborted = false;
                return true;
            }
            sleep(25);
        }
        return false;
    }

    interface Tick {
        boolean step(double seconds, double dt);
    }

    boolean runFor(double duration, Tick task) throws InterruptedException {
        double start = Maths.time(), previous = start - 0.02;
        while (opModeIsActive() && !motionAborted()) {
            double now = Maths.time(), elapsed = now - start;
            if (elapsed >= duration) return true;
            if (!task.step(elapsed, Maths.clip(now - previous, 0.005, 0.25))) return false;
            previous = now;
            sleep(20);
        }
        return false;
    }

    boolean motionAborted() {
        motionAborted |= gamepad1.b;
        return motionAborted;
    }

    boolean save(String key, double... values) {
        boolean saved = store.save(key, values);
        telemetry.log().add("%s %s", key, saved ? "已保存" : "保存失败");
        return saved;
    }

    void show(String task, String format, Object... values) {
        double now = Maths.time();
        if (now < nextDisplay) return;
        nextDisplay = now + 0.10;
        telemetry.addLine(task);
        telemetry.addData("测量", format, values);
        telemetry.addLine("B 停止当前动作");
        telemetry.update();
    }
}
