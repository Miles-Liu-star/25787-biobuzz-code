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
        double startX = PinPointConfig.CENTER_FROM_REAR_EDGE_IN;
        double startY = PinPointConfig.CENTER_FROM_LEFT_EDGE_IN;
        boolean lastY = false;
        boolean imuStarted = false;
        double nextStartAttempt = 0;
        IMU imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();
        DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);

        while (opModeInInit()) {
            if (!imuStarted && Maths.time() >= nextStartAttempt) {
                imuStarted = pinpoint.start();
                nextStartAttempt = Maths.time() + 1.0;
            }
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
        double headingOffset = anchored
                ? Maths.wrap(pinpoint.getHeading() - readHubYaw(imu)) : 0.0;
        while (opModeIsActive()) {
            if (!imuStarted && Maths.time() >= nextStartAttempt) {
                imuStarted = pinpoint.start();
                nextStartAttempt = Maths.time() + 1.0;
            }
            pinpoint.update();
            double controlHeading = Maths.wrap(readHubYaw(imu) + headingOffset);
            driveSubsystem.update(gamepad1, controlHeading, false, 0);
            boolean y = gamepad1.y;
            if (y && !lastY) {
                anchored = pinpoint.setFieldOrigin(startX, startY, 0);
                if (anchored) {
                    imu.resetYaw();
                    headingOffset = Maths.wrap(pinpoint.getHeading() - readHubYaw(imu));
                }
            }
            lastY = y;

            if (!anchored && !pinpoint.needsReanchor()) {
                anchored = pinpoint.setFieldOrigin(startX, startY, 0);
                if (anchored) {
                    headingOffset = Maths.wrap(pinpoint.getHeading() - readHubYaw(imu));
                }
            }

            telemetry.addData("场地 X (向前)", "%+.2f in", pinpoint.getXInches());
            telemetry.addData("场地 Y (向左)", "%+.2f in", pinpoint.getYInches());
            telemetry.addData("场地 X / Y", "%+.2f / %+.2f 格",
                    pinpoint.getXInches() / PinPointConfig.TILE_IN,
                    pinpoint.getYInches() / PinPointConfig.TILE_IN);
            telemetry.addData("定位有效", pinpoint.isPositionValid());
            telemetry.addData("Hub IMU 控制航向", "%+.1f°", Math.toDegrees(controlHeading));
            telemetry.addData("Pinpoint", pinpoint.getStatus());
            telemetry.addData("固件 / 坏读", "V%d / %d", pinpoint.getFirmwareVersion(), pinpoint.getBadReadCount());
            if (pinpoint.needsReanchor()) telemetry.addLine("定位中断：回到起始点后按 Y 重新锚定");
            telemetry.update();
            sleep(15);
        }
    }

    private static double readHubYaw(IMU imu) {
        return imu.getRobotYawPitchRollAngles().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);
    }
}
