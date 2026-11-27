package org.firstinspires.ftc.teamcode.automatic;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "红方自动 八段", group = "Red")
public class RedAuto1 extends LinearOpMode
{
    /* 防卡阈值需在本车上验证。 */
    private
    static final double AUTO_LIMIT = 29.5;
    private
    static final double PARK_RESERVE = 4.0;
    private
    static final double[] LEG_LIMIT = {5, 8, 3, 3, 6, 3, 6, 4};
    private
    static final double STALL_WINDOW = 1.2;
    private
    static final double MIN_MOVE_IN = 0.40;
    private
    static final double MIN_TURN_DEG = 3.0;
    private
    static final double POSITION_TOL_IN = 2.0;
    private
    static final double SHOT_HEADING_TOL = 5.0;
    private
    static final double TURN_LIMIT = 3.0;
    private
    static final double SETTLE_SEC = 0.20;
    private
    static final double SERIES_LIMIT_A = 5.0;
    private
    static final double OVERCURRENT_SEC = 0.40;

    /* 首轮和后续射球转速相同，俯仰分别设置。 */
    private
    static final double FIRST_SHOT_RPM = 3650;
    private
    static final double FOLLOWUP_SHOT_RPM = 3650;
    private
    static final double FIRST_SHOT_PITCH = 0.50;
    private
    static final double FOLLOWUP_SHOT_PITCH = 0.41;
    private
    double targetRpm = FIRST_SHOT_RPM;
    private
    static final double RPM_TOL = 80;
    private
    static final double TICKS_PER_REV = 28;
    private
    static final double SPINUP_LIMIT = 1.5;
    private
    static final double GATE_DELAY = 0.10;
    private
    static final double FEED_SEC = 2.0;

    private
    final PoseFactory pf = PoseFactory.degrees();
    private
    final Pose start = pf.of(58.77, 9.949, -90);
    private
    final Pose p1 = pf.of(8.558, 9.1458, -90);
    private
    final Pose c1 = pf.of(5.7019, 25.0337, 0);
    private
    final Pose p2 = pf.of(49.0542, 117.7491, 234.2);
    private
    final Pose c2 = pf.of(24.0053, 93.2037, 0);
    private
    final Pose s3 = pf.of(49.0542, 117.7491, -265.8);
    private
    final Pose p3 = pf.of(47.0386, 132.2078, 90);
    private
    final Pose p4 = pf.of(49.2303, 117.7379, -265.8);
    private
    final Pose s5 = pf.of(49.2303, 117.7379, 94.2);
    private
    final Pose p5 = pf.of(26.0462, 46.6055, -180);
    private
    final Pose c5 = pf.of(21.1146, 93.96, 0);
    private
    final Pose p6 = pf.of(9.4364, 46.5721, -179.8556);
    private
    final Pose s7 = pf.of(9.4364, 46.5721, -180);
    private
    final Pose p7 = pf.of(49.1038, 117.4402, 94.2);
    private
    final Pose c71 = pf.of(28.8863, 45.1362, 0);
    private
    final Pose c72 = pf.of(29.102, 115.2073, 0);
    private
    final Pose p8 = pf.of(10, 89.4445, -0.0133);
    private
    final Pose c8 = pf.of(29.2477, 89.44, 0);

    private
    final DcMotor[] wheels = new DcMotor[4];
    private
    final ElapsedTime timer = new ElapsedTime();
    private
    Follower follower;
    private
    DcMotorEx fly0, fly1, intake, transfer;
    private
    Servo small, big, pitchA, pitchB, shovelA, shovelB;
    private
    boolean spin, seriesOn;
    private
    double rpm, intakeA, transferA, highSince = -1, nextReport;
    private
    String phase = "初始化";

    private
    static final class Halt extends RuntimeException
    {
        Halt(String reason) { super(reason); }
    }

    @Override public void
    runOpMode()
    {
        String result = "未开始";
        try
        {
            Scheduler.reset();
            for (int i = 0; i < wheels.length; i++)
            {
                wheels[i] = hardwareMap.get(DcMotor.class, Integer.toString(i));
            }
            follower = Constants.create(hardwareMap);
            follower.setPose(start);
            initMechanisms();

            while (opModeInInit())
            {
                follower.update();
                stopDrive();
                Pose actual = follower.pose();
                if (valid(actual))
                {
                    telemetry.addData("起始定位 X / Y / 航向", "%.2f / %.2f / %.1f°",
                            actual.x(), actual.y(), Math.toDegrees(actual.heading()));
                    telemetry.addData("起点位置 / 航向误差", "%.2f 英寸 / %.1f°",
                            distance(actual, start), headingError(actual, start));
                }
                else
                {
                    telemetry.addData("起始定位", "无效");
                }
                telemetry.addLine("起点先射；Path2 结束后原地转向并射球");
                telemetry.addLine("确认车头朝 -90°，铲板升起，两闸关闭");
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested())
                return;
            timer.reset();
            result = runRoute() ? "Path8 停车完成" : "Path8 未到停车点，自动结束";
        }
        catch (Halt e)
        {
            result = phase + "：" + e.getMessage();
        }
        catch (RuntimeException e)
        {
            result = phase + "：" + e.getClass().getSimpleName() + " " + e.getMessage();
        }
        finally
        {
            try
            {
                Scheduler.reset();
            }
            finally
            {
                stopDrive();
                stopMechanisms();
            }
        }

        while (opModeIsActive())
        {
            telemetry.addData("最终状态", result);
            telemetry.addData("飞轮转速", "%.0f RPM", rpm);
            telemetry.addData("最近 02 / 03 电流", "%.2f / %.2f A", intakeA, transferA);
            telemetry.update();
            sleep(100);
        }
    }

    private
    void initMechanisms()
    {
        fly0 = motor("00", DcMotor.ZeroPowerBehavior.FLOAT);
        fly0.setDirection(DcMotor.Direction.FORWARD);
        fly1 = motor("01", DcMotor.ZeroPowerBehavior.FLOAT);
        intake = motor("02", DcMotor.ZeroPowerBehavior.BRAKE);
        transfer = motor("03", DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(DcMotor.Direction.REVERSE);

        small = hardwareMap.get(Servo.class, "s01");
        big = hardwareMap.get(Servo.class, "s1");
        pitchA = hardwareMap.get(Servo.class, "s2");
        pitchB = hardwareMap.get(Servo.class, "s3");
        shovelA = hardwareMap.get(Servo.class, "s00");
        shovelB = hardwareMap.get(Servo.class, "s02");

        setSeries(false);
        gates(false);
        pitchA.setPosition(FIRST_SHOT_PITCH);
        pitchB.setPosition(FIRST_SHOT_PITCH);
        shovel(false);
    }

    private
    DcMotorEx motor(String name, DcMotor.ZeroPowerBehavior zero)
    {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setPower(0);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(zero);
        return m;
    }

    private
    boolean runRoute()
    {
        /* 开局先尝试射球；姿态超差或预热超时则关闸，继续 Path1。 */
        targetRpm = FIRST_SHOT_RPM;
        shoot("起点预装球", start, true);
        targetRpm = FOLLOWUP_SHOT_RPM;
        pitchA.setPosition(FOLLOWUP_SHOT_PITCH);
        pitchB.setPosition(FOLLOWUP_SHOT_PITCH);

        collect(1, p1);
        spin = true;
        try
        {
            if (leg(2, p2) && turnAtP2())
            {
                shoot("Path2转向后射球", s3);
            }
        }
        finally
        {
            follower.stop();
            stopDrive();
            stopMechanisms();
        }

        moveShovel(true);
        collect(3, p3);
        spin = true;
        boolean reachedP4 = leg(4, p4);
        if (!reachedP4)
            stopMechanisms();
        moveShovel(false);
        if (reachedP4)
            shoot("Path4回射球区", p4);

        leg(5, p5);
        moveShovel(true);
        collect(6, p6);
        moveShovel(false);
        spin = true;
        boolean reachedP7 = leg(7, p7);

        if (reachedP7 && timer.seconds() + SPINUP_LIMIT + GATE_DELAY + FEED_SEC + PARK_RESERVE + 0.15 < AUTO_LIMIT)
        {
            shoot("Path7回射球区", p7);
        }
        else
        {
            stopMechanisms();
        }
        return leg(8, p8);
    }

    private
    Path path(int n)
    {
        switch (n)
        {
            case 1:
                return Paths.curve(start, c1, p1).constant(p1);
            case 2:
                return Paths.curve(p1, c2, p2).linear(p1, p2);
            case 3:
                return Paths.line(s3, p3).linear(s3, p3);
            case 4:
                return Paths.line(p3, p4).linear(p3, p4);
            case 5:
                return Paths.curve(s5, c5, p5).linear(s5, p5);
            case 6:
                return Paths.line(p5, p6).tangent();
            case 7:
                return Paths.curve(s7, c71, c72, p7).linear(s7, p7);
            case 8:
                return Paths.curve(p7, c8, p8).reverseTangent();
            default:
                throw new IllegalArgumentException("不存在 Path" + n);
        }
    }

    private
    boolean leg(int n, Pose target)
    {
        phase = "Path" + n;
        checkRun();
        boolean[] done = {false};
        boolean reached = false;
        ElapsedTime elapsed = new ElapsedTime();
        try
        {
            Scheduler.schedule(sequential(
                    follow(follower, path(n)), instant(()->done[0] = true)));
            Progress guard = new Progress();
            while (!done[0])
            {
                tick();
                Scheduler.execute();
                if (done[0])
                    break;
                if (elapsed.seconds() >= LEG_LIMIT[n - 1])
                {
                    reportLegSkip(n, "本段超时");
                    return false;
                }
                guard.check();
            }
            if (distance(follower.pose(), target) > POSITION_TOL_IN)
            {
                reportLegSkip(n, "跟随结束但未到终点");
                return false;
            }
            reached = true;
            return true;
        }
        finally
        {
            try
            {
                Scheduler.reset();
            }
            finally
            {
                if (!reached)
                    follower.stop();
                stopDrive();
            }
        }
    }

    private
    void reportLegSkip(int n, String reason)
    {
        String next = n < LEG_LIMIT.length ? "继续 Path" + (n + 1) : "自动结束";
        telemetry.log().add("Path" + n + "：" + reason + "，" + next);
    }

    private
    boolean turnAtP2()
    {
        phase = "Path2终点原地转向";
        checkRun();
        /* 位置保持不变，目标航向为 -265.8°，等价于 94.2°。 */
        follower.hold(s3);
        ElapsedTime elapsed = new ElapsedTime();
        double stableSince = -1;
        Progress guard = new Progress();
        while (true)
        {
            tick();
            double now = timer.seconds();
            if (atPose(s3))
            {
                if (stableSince < 0)
                    stableSince = now;
                if (now - stableSince >= SETTLE_SEC)
                    return true;
            }
            else
            {
                stableSince = -1;
            }
            if (elapsed.seconds() >= TURN_LIMIT)
            {
                telemetry.log().add("Path2 转向超时：跳过射球，继续 Path3");
                return false;
            }
            if (stableSince < 0)
                guard.check();
        }
    }

    private
    final class Progress
    {
        private
        Pose anchor = follower.pose();
        private
        double since = timer.seconds();

        void check()
        {
            Pose actual = follower.pose();
            if (distance(actual, anchor) >= MIN_MOVE_IN || headingError(actual, anchor) >= MIN_TURN_DEG)
            {
                anchor = actual;
                since = timer.seconds();
            }
            else if (timer.seconds() - since >= STALL_WINDOW)
            {
                throw new Halt("底盘无有效位移或转角，检查车轮、障碍和定位");
            }
        }
    }

    private void
    shoot(String name, Pose target)
    {
        shoot(name, target, false);
    }

    private
    void shoot(String name, Pose target, boolean skipOnPreShotPoseError)
    {
        phase = name + "：等待飞轮达速";
        checkRun();
        spin = true;
        try
        {
            ElapsedTime ready = new ElapsedTime();
            double stableSince = -1;
            while (true)
            {
                tick();
                if (!atPose(target))
                {
                    Pose actual = follower.pose();
                    String reason = String.format(
                            "姿态偏离：位置误差 %.2f 英寸，航向误差 %.1f 度，禁止送料",
                            distance(actual, target), headingError(actual, target));
                    if (skipOnPreShotPoseError)
                    {
                        telemetry.log().add(name + "：" + reason + "；跳过首射，继续 Path1");
                        return;
                    }
                    throw new Halt(reason);
                }
                double now = timer.seconds();
                if (Math.abs(rpm - targetRpm) <= RPM_TOL)
                {
                    if (stableSince < 0)
                        stableSince = now;
                    if (now - stableSince >= 0.08)
                        break;
                }
                else
                {
                    stableSince = -1;
                }
                if (ready.seconds() >= SPINUP_LIMIT)
                {
                    telemetry.log().add(String.format(
                            "%s：预热超时（%.0f / %.0f RPM），跳过射球",
                            name, rpm, targetRpm));
                    return;
                }
            }

            phase = name + "：开闸";
            gates(true);
            waitAtPose(GATE_DELAY, target);

            phase = name + "：送料射球";
            setSeries(true);
            waitAtPose(FEED_SEC, target);
        }
        finally
        {
            stopMechanisms();
        }
    }

    private
    void waitAtPose(double seconds, Pose target)
    {
        ElapsedTime t = new ElapsedTime();
        double lowSince = -1;
        while (t.seconds() < seconds)
        {
            tick();
            if (!atPose(target))
                throw new Halt("射球中姿态偏离");
            if (seriesOn && rpm < targetRpm * 0.45)
            {
                if (lowSince < 0)
                    lowSince = timer.seconds();
                if (timer.seconds() - lowSince >= 0.35)
                {
                    throw new Halt("送料时飞轮持续严重掉速");
                }
            }
            else
            {
                lowSince = -1;
            }
        }
    }

    private
    void collect(int n, Pose target)
    {
        checkRun();
        gates(false);
        setSeries(true);
        try
        {
            if (leg(n, target))
            {
                if (n == 3)
                {
                    phase = "Path3末端补吸：底盘保持";
                    try
                    {
                        follower.hold(target);
                        waitTick(0.6);
                    }
                    finally
                    {
                        try
                        {
                            follower.stop();
                        }
                        finally
                        {
                            stopDrive();
                        }
                    }
                }
                else
                {
                    phase = "Path" + n + "末端补吸";
                    waitTick(0.6);
                }
            }
        }
        finally
        {
            setSeries(false);
        }
    }

    private
    void moveShovel(boolean down)
    {
        checkRun();
        phase = down ? "放铲板" : "升铲板";
        shovel(down);
        waitTick(0.5);
    }

    private
    void waitTick(double seconds)
    {
        ElapsedTime t = new ElapsedTime();
        while (t.seconds() < seconds)
            tick();
    }

    private
    void checkRun()
    {
        if (!opModeIsActive() || isStopRequested())
            throw new Halt("收到 STOP");
        if (timer.seconds() >= AUTO_LIMIT)
            throw new Halt("自动总时限已到");
    }

    private
    void tick()
    {
        checkRun();
        follower.update();
        Pose actual = follower.pose();
        if (!valid(actual))
            throw new Halt("定位读数无效");

        double raw = -fly0.getVelocity() * 60.0 / TICKS_PER_REV;
        if (!Double.isFinite(raw))
            throw new Halt("00 飞轮编码器读数无效");
        rpm += 0.4 * (raw - rpm);
        double power = spin
                ? Range.clip(0.00020 * targetRpm + 0.0050 * (targetRpm - rpm),
                -0.1, 1.0)
                : 0;
        fly0.setPower(-power);
        fly1.setPower(power);

        if (seriesOn)
        {
            intakeA = intake.getCurrent(CurrentUnit.AMPS);
            transferA = transfer.getCurrent(CurrentUnit.AMPS);
            if (!Double.isFinite(intakeA) || !Double.isFinite(transferA))
            {
                throw new Halt("02/03 电流读数无效");
            }
            if (Math.max(intakeA, transferA) >= SERIES_LIMIT_A)
            {
                if (highSince < 0)
                    highSince = timer.seconds();
                if (timer.seconds() - highSince >= OVERCURRENT_SEC)
                {
                    throw new Halt(intakeA >= transferA
                            ? "02 吸球持续过流"
                            : "03 通道持续过流");
                }
            }
            else
            {
                highSince = -1;
            }
        }

        if (timer.seconds() >= nextReport)
        {
            nextReport = timer.seconds() + 0.10;
            telemetry.addData("状态", phase);
            telemetry.addData("时间", "%.1f s", timer.seconds());
            telemetry.addData("X / Y / 航向", "%.2f / %.2f / %.1f°",
                    actual.x(), actual.y(), Math.toDegrees(actual.heading()));
            telemetry.addData("飞轮", "%.0f RPM", rpm);
            telemetry.addData("最近 02 / 03 电流", "%.2f / %.2f A", intakeA, transferA);
            telemetry.update();
        }
        idle();
    }

    private
    boolean atPose(Pose target)
    {
        Pose actual = follower.pose();
        return valid(actual) && distance(actual, target) <= POSITION_TOL_IN && headingError(actual, target) <= SHOT_HEADING_TOL;
    }

    private
    static boolean valid(Pose p)
    {
        return p != null && Double.isFinite(p.x()) && Double.isFinite(p.y()) && Double.isFinite(p.heading());
    }

    private
    static double distance(Pose a, Pose b)
    {
        return Math.hypot(a.x() - b.x(), a.y() - b.y());
    }

    private
    static double headingError(Pose a, Pose b)
    {
        return Math.abs(Math.IEEEremainder(
                Math.toDegrees(a.heading() - b.heading()), 360));
    }

    private
    void shovel(boolean down)
    {
        shovelA.setPosition(down ? 0.48 : 0.00);
        shovelB.setPosition(down ? 0.52 : 1.00);
    }

    private
    void gates(boolean open)
    {
        if (small != null)
            small.setPosition(open ? 0.19 : 0.00);
        if (big != null)
            big.setPosition(open ? 0.00 : 0.09);
    }

    private
    void setSeries(boolean on)
    {
        seriesOn = on;
        highSince = -1;
        if (intake != null)
            intake.setPower(on ? 1 : 0);
        if (transfer != null)
            transfer.setPower(on ? 1 : 0);
    }

    private
    void stopDrive()
    {
        for (DcMotor wheel : wheels)
        {
            if (wheel != null)
                wheel.setPower(0);
        }
    }

    private
    void stopMechanisms()
    {
        spin = false;
        setSeries(false);
        if (fly0 != null)
            fly0.setPower(0);
        if (fly1 != null)
            fly1.setPower(0);
        gates(false);
    }
}
