package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pinpoint.PinpointXYReader;
import org.firstinspires.ftc.teamcode.pinpoint.PinPointConfig;
import org.firstinspires.ftc.teamcode.aim_bayes.AimCalibrationStore;
import org.firstinspires.ftc.teamcode.subsystems.AimSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PitchSubsystem;

/*
* 常用Terminal指令
 & "D:\AndroidStudio-SDK\platform-tools\adb.exe" connect 192.168.43.1:5555
 & "D:\AndroidStudio-SDK\platform-tools\adb.exe" disconnect 192.168.43.1:5555
 & "D:\AndroidStudio-SDK\platform-tools\adb.exe" devices
*/
@TeleOp(name = "无头手动", group = "Advanced")
public class MilesTeleop01 extends OpMode {
    private static final double AIM_TARGET_X_MM = 1400;
    private static final double AIM_TARGET_Y_MM = 1300;
    private static final double MANUAL_TURN_OVERRIDE = 0.12;

    private DcMotor intake = null;
    private DcMotor middleTurn = null;
    private PitchSubsystem pitch = null;
    private Servo switch_model_big = null;
    private Servo switch_model_small = null;
    private Servo shovel_lr = null;//不确定，需要改！！！！
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
    private boolean aimActive;
    private boolean wasAimTriggerPressed;
    private boolean initialPoseAnchored;
    private double[] aimGains;
    private boolean pinpointStarted = false;
    private boolean wasPressY = false;

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

        middleTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        pinpoint = new PinpointXYReader(hardwareMap);
        autoAim = new AimSubsystem();
        aimGains = new AimCalibrationStore(hardwareMap.appContext).readGains();
        if (aimGains != null) autoAim.setGains(aimGains[0], aimGains[1], aimGains[2]);

        pinpointStarted = pinpoint.start();
        pinpointStartTimer.reset();
        pitch = new PitchSubsystem(hardwareMap, telemetry);

        autoAim.setTargetPosition(AIM_TARGET_X_MM, AIM_TARGET_Y_MM);

        telemetry.addData("version", "0115");
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        if (!pinpointStarted && pinpointStartTimer.seconds() >= 1.0) {
            pinpointStarted = pinpoint.start();
            pinpointStartTimer.reset();
        }
        pinpoint.update();
        if (pinpointStarted && !initialPoseAnchored && !pinpoint.needsReanchor()) {
            initialPoseAnchored = anchorAtStart();
        }
        String blocked = aimBlockedReason();
        showAimStatus(blocked == null ? "就绪，右扳机开启" : blocked);
        telemetry.addLine("机器人放在配置起点，INIT 期间保持静止");
        telemetry.update();
    }

    @Override
    public void start() {
        wasAimTriggerPressed = gamepad1.right_trigger_pressed;
        wasPressY = gamepad1.y;
    }

    @Override
    public void loop() {
        pinpoint.update();
        updateDrive();

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

        if (gamepad1.dpad_up && !was_press_dpadup) {
            shooterSubsystem.increaseTargetRPM(15);
            was_press_dpadup = true;
        }
        if (gamepad1.dpad_down && !was_press_dpaddown) {
            shooterSubsystem.decreaseTargetRPM(15);
            was_press_dpaddown = true;
        }
        was_press_dpadup = gamepad1.dpad_up;
        was_press_dpaddown = gamepad1.dpad_down;
        shooterSubsystem.update(is_open_shooter);

        if (gamepad1.x && !was_press_switch) {
            ball_model = !ball_model;
            switch_model_big.setPosition(ball_model ? 0.19 : 0.0);
            switch_model_small.setPosition(ball_model ? 0.0 : 0.09);
        }
        was_press_switch = gamepad1.x;

        pitch.update();
        telemetry.update();
    }

    private void updateDrive() {
        boolean reanchorPressed = gamepad1.y && !wasPressY;
        wasPressY = gamepad1.y;
        String reanchorResult = null;
        if (reanchorPressed) {
            if (!pinpointStarted) pinpointStarted = pinpoint.start();
            reanchorResult = anchorAtStart() ? "已重新锚定" : "锚定失败，等定位就绪后再按 Y";
        }

        boolean triggerPressed = gamepad1.right_trigger_pressed;
        boolean toggleAim = triggerPressed && !wasAimTriggerPressed;
        wasAimTriggerPressed = triggerPressed;

        String blocked = aimBlockedReason();
        boolean manualOverride = Math.abs(gamepad1.right_stick_x) > MANUAL_TURN_OVERRIDE;
        boolean nextAimActive = aimActive;
        if (reanchorPressed || blocked != null || manualOverride) {
            nextAimActive = false;
        } else if (toggleAim) {
            nextAimActive = !aimActive;
        }
        if (nextAimActive != aimActive) autoAim.resetController();
        aimActive = nextAimActive;

        boolean fieldCentric = pinpoint.isHeadingUsable();
        double heading = fieldCentric ? pinpoint.getHeading() : 0.0;
        double turn = 0.0;
        if (aimActive) {
            turn = autoAim.calculateTurn(pinpoint.getXmm(), pinpoint.getYmm(), heading, pinpoint.getYawRate());
            telemetry.addData("目标航向 / 误差", "%+.1f° / %+.1f°",
                    Math.toDegrees(autoAim.getTargetHeadingRadians()),
                    Math.toDegrees(autoAim.getHeadingErrorRadians()));
        }
        driveSubsystem.update(gamepad1, heading, aimActive, turn);

        String status = blocked != null ? blocked : aimActive ? "已开启" : "已关闭，右扳机开启";
        if (blocked == null && manualOverride) status = "右摇杆手动转向";
        showAimStatus(status);
        if (reanchorResult != null) telemetry.addData("Pinpoint 锚定", reanchorResult);
        telemetry.addData("驾驶坐标", fieldCentric ? "场地" : "车身");
        telemetry.addData("自瞄 turn（左为正）", "%+.3f", turn);
    }

    private boolean anchorAtStart() {
        return pinpointStarted && pinpoint.setFieldOrigin(startX, startY, 0)
                && pinpoint.isPoseValidForControl();
    }

    private String aimBlockedReason() {
        if (aimGains == null) return "需要运行「自测自动瞄准 PID」";
        if (pinpoint.needsReanchor()) return "定位失效，回起点按 Y";
        if (!pinpoint.isPoseValidForControl()) return "等待有效定位，必要时回起点按 Y";
        return null;
    }

    private void showAimStatus(String status) {
        telemetry.addData("自瞄", status);
        telemetry.addData("Pinpoint", "%s / 定位%s", pinpoint.getStatus(),
                pinpoint.isPoseValidForControl() ? "有效" : "无效");
        telemetry.addData("坐标 X / Y / 航向", "%+.0f / %+.0f mm / %+.1f°",
                pinpoint.getXmm(), pinpoint.getYmm(), Math.toDegrees(pinpoint.getHeading()));
        telemetry.addData("目标 X / Y", "%.0f / %.0f mm", AIM_TARGET_X_MM, AIM_TARGET_Y_MM);
        if (aimGains != null) {
            telemetry.addData("自瞄 P / I / D", "%.4f / %.4f / %.4f", aimGains[0], aimGains[1], aimGains[2]);
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
