//package org.firstinspires.ftc.teamcode.PIDTest;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.teamcode.pedroRathing.PID;
//
//// @Config 注解让FTC Dashboard可以识别并修改这个类中的 public static 变量
//@Config
//@TeleOp(name = "Motor PIDF Tuner (Dashboard)", group = "Tuning")
//public class PIDTest extends LinearOpMode{
//
//    // 定义电机对象
//    private DcMotorEx motor;
//    private DcMotorEx motor2;
//
//    // FtcDashboard 实例
//    private FtcDashboard dashboard;
//
//    // *****************************************************************************************
//    //                                  用户需要修改/调试的常量
//    // *****************************************************************************************
//
//    // 在这里输入你从电机规格中查到的每转编码器刻度数
//    public static final double MOTOR_TICK_COUNT = 28; // 例如, goBILDA 5203系列 19.2:1 电机
//
//
//    // Dashboard writes this field from its WebSocket thread.
//    public static volatile double TARGET_RPM = 2000;
//
//    // PIDF 系数 - 现在可以通过Dashboard实时修改！
//    // P, I, D, F 必须是独立的 public static 变量才能被Dashboard识别
//    public static double P = 0.006, I = 0, D = 0.0000;
//    public static double F = 0.0004;
//
//
//    // *****************************************************************************************
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // 初始化 FTC Dashboard
//        dashboard = FtcDashboard.getInstance();
//
//        // 从硬件映射中获取电机
//        motor = hardwareMap.get(DcMotorEx.class, "00");
//        motor2 = hardwareMap.get(DcMotorEx.class, "01");
//        motor.setDirection(DcMotor.Direction.REVERSE);
//        motor2.setDirection(DcMotor.Direction.FORWARD);
//        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        while (opModeInInit()) {
//            double targetRpm = TARGET_RPM;
//            TelemetryPacket packet = new TelemetryPacket();
//            packet.put("目标 RPM", targetRpm);
//            dashboard.sendTelemetryPacket(packet);
//            telemetry.addData("目标 RPM", targetRpm);
//            telemetry.addLine("Dashboard → PIDTest → TARGET_RPM，修改后点 Save Changes");
//            telemetry.update();
//            sleep(100);
//        }
//
//        if (opModeIsActive()) {
//
//            while (opModeIsActive()) {
//                double targetRpm = TARGET_RPM;
//                // 从Dashboard实时更新PIDF系数
////                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
////                motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//
//                // 获取当前电机的速度（单位：刻度/秒）
//                double currentVelocityTicks = motor.getVelocity();
//
//                // 将当前速度从 刻度/秒 转换为 RPM
//                double currentRPM = (currentVelocityTicks / MOTOR_TICK_COUNT) * 60;
//
//                PID pid = new PID(P,0,D);
//                // 将目标RPM转换为每秒的编码器刻度数和功率
//                double targetTicksPerSecond = targetRpm * MOTOR_TICK_COUNT / 60.0;
//                double feedForward = F * targetTicksPerSecond;
////
//                double pidPower = pid.update(targetTicksPerSecond-currentVelocityTicks);
//                double targetPower = Range.clip(feedForward + pidPower, -1.0, 1.0);
//                // 设置电机的目标功率
//                motor.setPower(targetPower);
//                motor2.setPower(targetPower);
//
//
//
//                // 创建一个数据包，用于发送到Dashboard
//                TelemetryPacket packet = new TelemetryPacket();
//
//                // 将你需要绘制图表的数据放入数据包
//                // packet.put(key, value)
//                packet.put("目标 RPM", targetRpm);
//                packet.put("当前 RPM", currentRPM);
//                packet.put("误差 (RPM)", targetRpm - currentRPM);
//
//                // 发送数据包到Dashboard
//                dashboard.sendTelemetryPacket(packet);
//
//                // 同时在手机屏幕上显示简单数据
//                telemetry.addData("目标 RPM", targetRpm);
//                telemetry.addData("当前 RPM", "%.2f", currentRPM);
//                telemetry.update();
//            }
//        }
//
//        // 停止电机
//        motor.setPower(0);
//        motor2.setPower(0);
//    }
//}
