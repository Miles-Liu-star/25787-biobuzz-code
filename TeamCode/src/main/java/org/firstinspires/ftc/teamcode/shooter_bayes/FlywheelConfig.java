package org.firstinspires.ftc.teamcode.shooter_bayes;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * 飞轮相关常量。设备名必须与 Driver Station -> Robot Configuration 中的名称一致。
 *
 * 你的机器人：00 接编码器反馈，01 为另一侧飞轮电机。
 */
public final class FlywheelConfig {
    /** 带编码器的飞轮电机（反馈来源）。 */
    public static final String FLYWHEEL_ENCODER_MOTOR = "00";
    /** 第二个独立射手电机：只跟随 00 输出，不参与测速和 PID 反馈。 */
    public static final String FLYWHEEL_FOLLOWER_MOTOR = "01";

    public static final DcMotorSimple.Direction FLYWHEEL_ENCODER_DIRECTION =
            DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FLYWHEEL_FOLLOWER_DIRECTION =
            DcMotorSimple.Direction.FORWARD;

    /**
     * 可选挡弹舵机。你的车目前没有配置 SB，保持 null 即可；
     * 以后加了挡弹机构，把这里改成舵机名（例如 "s3"）就能启用 feed()。
     */
    public static final String BLOCKER_SERVO = null;
    public static final double SB_IDLE = 0.25;
    public static final double SB_FIRE = 0.60;

    /** 飞轮最大输出功率。 */
    public static final double FLYWHEEL_MAX_POWER = 1.0;
    /** 每秒允许增加的输出功率（功率爬升限制）。 */
    public static final double FLYWHEEL_RAMP = 2.5;

    /** 04 自测时使用的低速目标（RPM）。 */
    public static final double FLYWHEEL_IDLE_RPM = 1200;
    /** 04 自测时使用的射击目标（RPM）。 */
    public static final double FLYWHEEL_SHOOT_RPM = 1800;

    /** 判定“到位”的转速容差，0.02 表示 ±2%。 */
    public static final double FLYWHEEL_SPEED_TOLERANCE = 0.02;

    /** TeleOp 默认目标转速（RPM）。 */
    public static final double DEFAULT_TARGET_RPM = 1800;

    /**
     * 每飞轮转一圈的编码器计数。0 表示尚未标定：
     * - 04 自测会先带你标定它；
     * - 正式 TeleOp 在未标定时会退回电机自带的 ticks/rev；
     * - 但只有跑完 04 生成功率模型后，闭环才会真正输出。
     */
    public static final double FLYWHEEL_TICKS_PER_WHEEL_REV = 0;

    private FlywheelConfig() {}
}
