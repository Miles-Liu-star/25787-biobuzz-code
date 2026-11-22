package org.firstinspires.ftc.teamcode.pinpoint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/** Live diagnostic that displays the field-relative XY returned by PinpointXYReader. */
@TeleOp(name = "Pinpoint XY 坐标读取", group = "BIOBUZZ 诊断")
public final class PinpointXYTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PinpointXYReader pinpoint = new PinpointXYReader(hardwareMap);
        double startX = PinPointConfig.CENTER_FROM_REAR_EDGE_MM;
        double startY = -PinPointConfig.CENTER_FROM_LEFT_EDGE_MM;
        boolean lastY = false;
        boolean imuStarted = false;
        double nextStartAttempt = 0;
        DriveSubsystem driveSubsystem = null;
        IMU imu = null;

        while (opModeInInit()) {
            if (!imuStarted && Maths.time() >= nextStartAttempt) {
                imuStarted = pinpoint.start();
                nextStartAttempt = Maths.time() + 1.0;
            }
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();
            driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
            pinpoint.update();
            telemetry.addLine("机器人保持静止等待 Pinpoint 校准；START 后按项目默认起始坐标进行锚定。");
            telemetry.addData("Pinpoint", pinpoint.getStatus());
            telemetry.addData("固件 / 坏读", "V%d / %d", pinpoint.getFirmwareVersion(), pinpoint.getBadReadCount());
            telemetry.update();
            sleep(25);
        }
        if (isStopRequested()) return;

        // Match BioBuzzTeleOp's default start location and field-axis convention.
        pinpoint.update();
        boolean anchored = pinpoint.setFieldOrigin(startX, startY, 0);
        while (opModeIsActive()) {
            if (!imuStarted && Maths.time() >= nextStartAttempt) {
                imuStarted = pinpoint.start();
                nextStartAttempt = Maths.time() + 1.0;
            }
            pinpoint.update();
            driveSubsystem.update(gamepad1, pinpoint.getHeading());
            boolean y = gamepad1.y;
            if (y && !lastY) {
                anchored = pinpoint.setFieldOrigin(startX, startY, 0);
                imu.resetYaw();
            }
            lastY = y;

            if (!anchored && !pinpoint.needsReanchor()) {
                anchored = pinpoint.setFieldOrigin(startX, startY, 0);
            }

            telemetry.addData("场地 X (向前)", "%+.1f mm", pinpoint.getXmm());
            telemetry.addData("场地 Y (向左)", "%+.1f mm", pinpoint.getYmm());
            telemetry.addData("场地 X / Y", "%+.2f / %+.2f 格",
                    pinpoint.getXmm() / PinPointConfig.TILE_MM,
                    pinpoint.getYmm() / PinPointConfig.TILE_MM);
            telemetry.addData("定位有效", pinpoint.isPositionValid());
            telemetry.addData("Pinpoint", pinpoint.getStatus());
            telemetry.addData("固件 / 坏读", "V%d / %d", pinpoint.getFirmwareVersion(), pinpoint.getBadReadCount());
            if (pinpoint.needsReanchor()) telemetry.addLine("定位中断：回到起始点后按 Y 重新锚定");
            telemetry.update();
            sleep(15);
        }
    }
}
