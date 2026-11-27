package org.firstinspires.ftc.teamcode.subsystems;

/**
 * 2D 散点打表 + 高斯核（softmax 距离加权）查表。
 *
 * 权重: w_i = exp( -(d_i - dmin)^2 / (2σ^2) )，d_i > maxRadius 的点权重为 0
 * 减去 dmin 只是数值稳定，归一化后等价于 exp(-d_i^2/2σ^2) 的 softmax。
 *
 * 调参:
 *   σ  ≈ 打表点间距的 0.5~1 倍（8 英寸一格 → σ 取 6~8）
 *   maxRadius = 2~3 倍 σ，或 Double.MAX_VALUE 表示不限制
 */
public class table {
    private final double sigma = 6;
    private final double maxRadius = 15;
    public static class Point {
        public final double x, y;
        public final double pitchSmall, pitchBig, shooterRPM;

        public Point(double x, double y, double pitchSmall, double pitchBig, double shooterRPM) {
            this.x = x;
            this.y = y;
            this.pitchSmall = pitchSmall;
            this.pitchBig = pitchBig;
            this.shooterRPM = shooterRPM;
        }
    }

    /* ==================== 打表数据（在这里填） ==================== */
    // 每行: new Point(x, y, pitchSmall, pitchBig, shooterRPM)
    //   x, y        : 打表时机器人的 Pinpoint 坐标（英寸）
    //   pitchSmall  : 小球俯仰 Servo 位置 0~1
    //   pitchBig    : 大球俯仰 Servo 位置 0~1
    //   shooterRPM  : 目标转速
    private static final Point[] POINTS = {
    };

    /* ==================== 查表结果 ==================== */
    public static class Result {
        public double x, y;                          // 加权后的坐标（可用于自瞄方向）
        public double pitchSmall, pitchBig, shooterRPM;
        public double bestDist;                      // 到最近打表点的距离（供 telemetry）
        public boolean valid;                        // false = 表为空
        public boolean fallback;                     // true = 圆内无点，退化为最近点
    }

    private final Point[] points;

    public table() {
        this.points = POINTS;
    }

    /** 允许外部传入自定义点集（比如从 SharedPreferences 加载） */
    public table(Point[] customPoints) {
        this.points = customPoints;
    }

    public int size() {
        return points.length;
    }

    /**
     * 高斯核加权平均查表。
     *
     * @param px,py     机器人当前坐标（英寸）
     * @return 加权后的坐标和参数；valid=false 表示表为空，调用方走兜底
     */
    public Result weighted(double px, double py) {
        Result r = new Result();
        int n = points.length;
        if (n == 0) {
            r.valid = false;
            return r;
        }

        double[] d = new double[n];
        double dmin = Double.MAX_VALUE;
        int inCircle = 0;
        for (int i = 0; i < n; i++) {
            Point p = points[i];
            d[i] = Math.hypot(px - p.x, py - p.y);
            if (d[i] <= maxRadius) {
                inCircle++;
                if (d[i] < dmin) dmin = d[i];
            }
        }

        if (inCircle == 0) {
            int best = 0;
            double bestD = Double.MAX_VALUE;
            for (int i = 0; i < n; i++) {
                if (d[i] < bestD) {
                    bestD = d[i];
                    best = i;
                }
            }
            Point p = points[best];
            r.x = p.x;
            r.y = p.y;
            r.pitchSmall = p.pitchSmall;
            r.pitchBig = p.pitchBig;
            r.shooterRPM = p.shooterRPM;
            r.bestDist = bestD;
            r.valid = true;
            r.fallback = true;
            return r;
        }

        double[] w = new double[n];
        double sum = 0.0;
        double twoSigmaSq = 2.0 * sigma * sigma;
        for (int i = 0; i < n; i++) {
            if (d[i] > maxRadius) {
                w[i] = 0.0;
                continue;
            }
            double dd = d[i] - dmin;
            w[i] = Math.exp(-(dd * dd) / twoSigmaSq);
            sum += w[i];
        }
        if (sum < 1e-12) {
            r.valid = false; // 理论上不会到，兜底
            return r;
        }

        double sx = 0, sy = 0, ss = 0, sb = 0, sr = 0;
        for (int i = 0; i < n; i++) {
            if (w[i] == 0.0) continue;
            w[i] /= sum;
            Point p = points[i];
            sx += w[i] * p.x;
            sy += w[i] * p.y;
            ss += w[i] * p.pitchSmall;
            sb += w[i] * p.pitchBig;
            sr += w[i] * p.shooterRPM;
        }
        r.x = sx;
        r.y = sy;
        r.pitchSmall = ss;
        r.pitchBig = sb;
        r.shooterRPM = sr;
        r.bestDist = dmin;
        r.valid = true;
        r.fallback = false;
        return r;
    }
}