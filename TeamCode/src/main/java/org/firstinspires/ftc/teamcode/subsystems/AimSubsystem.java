package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Calculates the turn command needed to aim at a field-coordinate target.
 *
 * <p>This class does not read sensors or control motors. The OpMode supplies
 * the latest robot pose and angular velocity, then passes the returned turn
 * command to the drive subsystem.</p>
 *
 * <p>Coordinates use the Pinpoint convention: X is forward and Y is left,
 * both in millimeters. Heading is in radians, with positive angles turning
 * toward the left.</p>
 */
public final class AimSubsystem {
    // The calibration OpMode supplies a search seed and saves measured gains.
    public static final double DEFAULT_K_P = 0.0;
    public static final double DEFAULT_K_I = 0.0;
    public static final double DEFAULT_K_D = 0.0;

    private static final double MAX_TURN = 1.0;
    private static final double MAX_INTEGRAL = 1.0;
    private static final double MAX_INTEGRATION_STEP_SECONDS = 0.1;

    private double targetXmm;
    private double targetYmm;
    private boolean targetSet;
    private double targetHeadingRadians;
    private double headingErrorRadians;
    private double integralError;
    private long lastUpdateNanos;
    private double kP = DEFAULT_K_P;
    private double kI = DEFAULT_K_I;
    private double kD = DEFAULT_K_D;

    /** Replace the controller gains and clear accumulated state. Gains must be finite and nonnegative. */
    public void setGains(double p, double i, double d) {
        if (!Double.isFinite(p) || !Double.isFinite(i) || !Double.isFinite(d)
                || p < 0 || i < 0 || d < 0) {
            throw new IllegalArgumentException("PID gains must be finite and nonnegative");
        }
        kP = p;
        kI = i;
        kD = d;
        resetController();
    }

    /** Return a copy of the active [P, I, D] gains. */
    public double[] getGains() {
        return new double[]{kP, kI, kD};
    }

    /** Set or replace the target field coordinate, in millimeters. */
    public void setTargetPosition(double targetXmm, double targetYmm) {
        if (this.targetXmm != targetXmm || this.targetYmm != targetYmm) {
            resetController();
        }
        this.targetXmm = targetXmm;
        this.targetYmm = targetYmm;
        targetSet = Double.isFinite(targetXmm) && Double.isFinite(targetYmm);
    }

    /**
     * Calculate the normalized turn command for the current robot state.
     * @param robotXmm current field X coordinate in millimeters
     * @param robotYmm current field Y coordinate in millimeters
     * @param headingRadians current robot heading in radians
     * @param angularVelocityRadiansPerSecond current angular velocity in radians/second
     * @return turn command clamped to [-1, 1], or zero if the input/target is invalid
     */
    public double calculateTurn(double robotXmm, double robotYmm,
                                double headingRadians,
                                double angularVelocityRadiansPerSecond) {
        if (!targetSet || !Double.isFinite(robotXmm) || !Double.isFinite(robotYmm)
                || !Double.isFinite(headingRadians)
                || !Double.isFinite(angularVelocityRadiansPerSecond)) {
            targetHeadingRadians = 0.0;
            headingErrorRadians = 0.0;
            resetController();
            return 0.0;
        }

        double deltaX = targetXmm - robotXmm;
        double deltaY = targetYmm - robotYmm;
        if (deltaX == 0.0 && deltaY == 0.0) {
            targetHeadingRadians = headingRadians;
            headingErrorRadians = 0.0;
            resetController();
            return 0.0;
        }
        targetHeadingRadians = Math.atan2(deltaY, deltaX);
        headingErrorRadians = wrapRadians(targetHeadingRadians - headingRadians);

        long nowNanos = System.nanoTime();
        double deltaSeconds = lastUpdateNanos == 0L
                ? 0.0 : (nowNanos - lastUpdateNanos) / 1_000_000_000.0;
        lastUpdateNanos = nowNanos;
        deltaSeconds = Math.max(0.0, Math.min(deltaSeconds, MAX_INTEGRATION_STEP_SECONDS));

        double candidateIntegral = clamp(
                integralError + headingErrorRadians * deltaSeconds,
                -MAX_INTEGRAL, MAX_INTEGRAL);
        double turn = calculateUnclampedTurn(candidateIntegral, angularVelocityRadiansPerSecond);

        // Do not accumulate more error while the output is saturated in the same direction.
        boolean pushesFurtherIntoSaturation = (turn > MAX_TURN && headingErrorRadians > 0.0)
                || (turn < -MAX_TURN && headingErrorRadians < 0.0);
        if (!pushesFurtherIntoSaturation) {
            integralError = candidateIntegral;
        }

        turn = calculateUnclampedTurn(integralError, angularVelocityRadiansPerSecond);
        return clamp(turn, -MAX_TURN, MAX_TURN);
    }

    /** Clear accumulated controller state, for example when auto-aim is disabled. */
    public void resetController() {
        integralError = 0.0;
        lastUpdateNanos = 0L;
    }

    /** Most recently calculated target bearing, in radians. */
    public double getTargetHeadingRadians() {
        return targetHeadingRadians;
    }

    /** Most recently calculated shortest-path heading error, in radians. */
    public double getHeadingErrorRadians() {
        return headingErrorRadians;
    }

    private static double wrapRadians(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private double calculateUnclampedTurn(double integral, double angularVelocityRadiansPerSecond) {
        // Measured angular velocity supplies derivative damping.
        return kP * headingErrorRadians
                + kI * integral
                - kD * angularVelocityRadiansPerSecond;
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}
