package org.firstinspires.ftc.teamcode.automatic;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/* 包名以 Android Studio 里 teamcode 下的实际文件夹名为准：若标红，把 pedroRathing 改成 pedroPathing */
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class RedAuto1 extends LinearOpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    /* 超时时间（秒）：只在 path3、7、8、9、10 设置，其余路径不设 */
    public static double PATH3_TIMEOUT = 8.0;
    public static double PATH7_TIMEOUT = 3.0;
    public static double PATH8_TIMEOUT = 3.0;
    public static double PATH9_TIMEOUT = 5.0;
    public static double PATH10_TIMEOUT = 4.0;

    /* path3 结束后，离 point3 超过该距离（英寸）就先校正回 point3；校正最多花 SETTLE_TIMEOUT 秒 */
    public static double SETTLE_DISTANCE = 1.5;
    public static double SETTLE_TIMEOUT = 1.5;

    private int timeoutCount = 0;
    private String lastTimeoutPath = "无";

    /* path3 结束瞬间，实际位置相对 point3 的偏差，用来判断到底偏了多少、偏向哪边 */
    private double path3EndDx = 0;
    private double path3EndDy = 0;
    private double path3EndHeadingDeg = 0;

    private final Pose start = poseFactory.of(55.9992, 9.0379, -90);
    private final Pose path1Target = poseFactory.of(56.0098, 14.4482, -90.1121);

    private final Pose point2 = poseFactory.of(8.558, 9.1458, -90);
    private final Pose point2Control1 = poseFactory.of(16.1563, 20.4927, 0);

    /* -250 与 110 是同一方向，统一写成 110 */
    private final Pose point3 = poseFactory.of(49.0542, 121.0074, 110);
    private final Pose point3Control1 = poseFactory.of(20.0759, 44.9294, 0);
    private final Pose point3Control2 = poseFactory.of(20.5513, 93.5285, 0);
    private final Pose point3Segment1Heading = poseFactory.of(49.0542, 121.0074, -90);
    private final Pose point3Segment3Start = poseFactory.of(49.0542, 121.0074, -113.9127);
    private final Pose point3Segment3End = poseFactory.of(49.0542, 100.0074, 110);

    private final Pose point4 = poseFactory.of(47.0386, 132.2078, 90);

    private final Pose point5 = poseFactory.of(49.2088, 120.6891, 109.3732);
    private final Pose point5Segment1Heading = poseFactory.of(49.2088, 120.6891, 90);

    private final Pose point6 = poseFactory.of(22.6759, 46.6055, 91.8884);
    private final Pose point6Control1 = poseFactory.of(21.1146, 93.96, 0);
    private final Pose point6Segment1Heading = poseFactory.of(22.6759, 46.6055, 90);

    private final Pose point7 = poseFactory.of(9.4364, 46.5721, -179.8556);

    private final Pose point8 = poseFactory.of(27.7752, 46.4057, 179.4801);

    private final Pose point9 = poseFactory.of(49.08, 120.9256, 110);
    private final Pose point9Control1 = poseFactory.of(29.3975, 105.9785, 0);
    private final Pose point9Segment2Start = poseFactory.of(49.08, 120.9256, -105.0982);
    private final Pose point9Segment2End = poseFactory.of(49.08, 120.9256, 110);

    private final Pose point10 = poseFactory.of(10, 89.4445, -0.0133);
    private final Pose point10Control1 = poseFactory.of(29.2477, 89.44, 0);

    /* 带超时的跟随：超过 timeoutSec 仍未走完就结束这一段，进入下一段 */
    private Command followTimed(Path path, double timeoutSec, String name) {
        /* race：跟随和计时器同时跑，谁先结束就结束整个 race。
           正常走完时计时器被取消，不会计入超时；计时器先到才记一次超时 */
        return race(
                follow(follower, path),
                sequential(
                        waitMs(timeoutSec * 1000.0),
                        instant(() -> {
                            timeoutCount++;
                            lastTimeoutPath = name;
                        })
                )
        );
    }

    /* 记录 path3 结束瞬间的实际位置偏差，只做遥测，不影响动作 */
    private Command recordPath3End() {
        return instant(() -> {
            Pose cur = follower.pose();
            if (cur == null) return;
            path3EndDx = cur.x() - point3.x();
            path3EndDy = cur.y() - point3.y();
            path3EndHeadingDeg = Math.toDegrees(cur.heading());
        });
    }

    /* path3 结束后无论是否超时，只要离 point3 超过 SETTLE_DISTANCE，就从当前位置直线校正回 point3。
       校正最多 SETTLE_TIMEOUT 秒，防止卡在原地 */
    private Command settleAtPoint3() {
        return lazy(() -> {
            Pose cur = follower.pose();
            if (cur == null) return instant(() -> { });
            double dist = Math.hypot(cur.x() - point3.x(), cur.y() - point3.y());
            if (dist < SETTLE_DISTANCE) return instant(() -> { });
            return race(
                    follow(follower, Paths.line(cur, point3).constant(point3)),
                    waitMs(SETTLE_TIMEOUT * 1000.0)
            );
        });
    }

    /* path4 从机器人实际位置出发，直线连到 point4。
       这样即使到 point3 时有横向偏差，也会在去花朵的路上被拉回 point4 的正前方 */
    private Command path4FromCurrent() {
        return lazy(() -> {
            Pose cur = follower.pose();
            if (cur == null) return follow(follower, path4());
            return follow(follower, Paths.line(cur, point4).constant(point4));
        });
    }

    /* 自动路径序列 */
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                followTimed(path3(), PATH3_TIMEOUT, "path3"),
                recordPath3End(),
                settleAtPoint3(),
                path4FromCurrent(),
                follow(follower, path5()),
                follow(follower, path6()),
                followTimed(path7(), PATH7_TIMEOUT, "path7"),
                followTimed(path8(), PATH8_TIMEOUT, "path8"),
                followTimed(path9(), PATH9_TIMEOUT, "path9"),
                followTimed(path10(), PATH10_TIMEOUT, "path10")
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        /* init 阶段持续刷新定位，消除起步抽搐 */
        while (opModeInInit()) {
            follower.update();
            Pose initPose = follower.pose();
            if (initPose != null) {
                telemetry.addData("x", initPose.x());
                telemetry.addData("y", initPose.y());
                telemetry.addData("heading(deg)", Math.toDegrees(initPose.heading()));
            } else {
                telemetry.addLine("定位丢失");
            }
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            Pose livePose = follower.pose();
            if (livePose != null) {
                telemetry.addData("x", livePose.x());
                telemetry.addData("y", livePose.y());
                telemetry.addData("heading(deg)", Math.toDegrees(livePose.heading()));
            } else {
                telemetry.addLine("定位丢失");
            }

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }
            telemetry.addData("超时次数", timeoutCount);
            telemetry.addData("最近超时的路径", lastTimeoutPath);
            /* dx 为正表示 path3 结束时偏在 point3 右侧，dy 为正表示偏在 point3 上方 */
            telemetry.addData("path3结束 dx(英寸)", path3EndDx);
            telemetry.addData("path3结束 dy(英寸)", path3EndDy);
            telemetry.addData("path3结束 朝向(度)", path3EndHeadingDeg);
            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, path1Target).reverseTangent();
    }

    public Path path2() {
        return Paths.curve(path1Target, point2Control1, point2).constant(point2);
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3Control2, point3)
                .heading(Interpolator.piecewise()
                        .until(0.1373, Interpolator.constant(point3Segment1Heading))
                        .until(0.7344, Interpolator.tangent.reverse())
                        .until(1, Interpolator.linear(point3Segment3Start, point3Segment3End)));
    }

    public Path path4() {
        return Paths.line(point3, point4).constant(point4);
    }

    /* 终点姿态线性插值，不使用 facingPoint，跑完不会原地自转 */
    public Path path5() {
        return Paths.line(point4, point5)
                .heading(Interpolator.piecewise()
                        .until(0.1657, Interpolator.constant(point5Segment1Heading))
                        .until(1, Interpolator.linear(point4, point5)));
    }

    public Path path6() {
        return Paths.curve(point5, point6Control1, point6)
                .heading(Interpolator.piecewise()
                        .until(0.0515, Interpolator.constant(point6Segment1Heading))
                        .until(1, Interpolator.tangent.reverse()));
    }

    public Path path7() {
        return Paths.line(point6, point7).reverseTangent();
    }

    public Path path8() {
        return Paths.line(point7, point8).reverseTangent();
    }

    /* path8 到 path9 用 point8 到 point9Segment2Start 的线性过渡，避免 180 度跳变 */
    public Path path9() {
        return Paths.curve(point8, point9Control1, point9)
                .heading(Interpolator.piecewise()
                        .until(0.5949, Interpolator.linear(point8, point9Segment2Start))
                        .until(1, Interpolator.linear(point9Segment2Start, point9Segment2End)));
    }

    public Path path10() {
        return Paths.curve(point9, point10Control1, point10).reverseTangent();
    }
}