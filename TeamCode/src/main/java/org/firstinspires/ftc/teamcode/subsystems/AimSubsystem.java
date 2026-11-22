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
    // Tune these controller gains on the robot. They are intentionally unset.
    private static final double K_P = 0.0;
    private static final double K_D = 0.0;

    private static final double MAX_TURN = 1.0;

    private double targetXmm;
    private double targetYmm;
    private boolean targetSet;
    private double targetHeadingRadians;
    private double headingErrorRadians;

    /** Set or replace the target field coordinate, in millimeters. */
    public void setTargetPosition(double targetXmm, double targetYmm) {
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
            return 0.0;
        }

        double deltaX = targetXmm - robotXmm;
        double deltaY = targetYmm - robotYmm;
        if (deltaX == 0.0 && deltaY == 0.0) {
            targetHeadingRadians = headingRadians;
            headingErrorRadians = 0.0;
            return 0.0;
        }
        targetHeadingRadians = Math.atan2(deltaY, deltaX);
        headingErrorRadians = wrapRadians(targetHeadingRadians - headingRadians);

        // Derivative damping uses measured angular velocity supplied by the OpMode.
        double turn = K_P * headingErrorRadians
                - K_D * angularVelocityRadiansPerSecond;
        return clamp(turn, -MAX_TURN, MAX_TURN);
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

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}
