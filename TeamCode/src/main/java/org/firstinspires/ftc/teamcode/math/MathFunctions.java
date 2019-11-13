package org.firstinspires.ftc.teamcode.math;

public class MathFunctions {
    public static double degreesToRadians (double degrees) {
        return Math.PI / 180 * degrees;
    }

    public static double radiansToDegrees (double radians) {
        return 180 / Math.PI * radians;
    }

    /*
     * Rotate a point counterclockwise by theta degrees
     * Returns double[] {newX, newY}
     */
    public static double[] rotatePointCounterClockwise (double x, double y, double thetaDegrees) {
        double theta = degreesToRadians(thetaDegrees);
        double newX = x * Math.cos(theta) - y * Math.sin(theta);
        double newY = x * Math.sin(theta) + y * Math.cos(theta);
        return new double[] {newX, newY};
    }

    public static double absMax(double... doubles) {
        double max = 0;
        for (double d : doubles) {
            max = Math.max(max, Math.abs(d));
        }
        return max;
    }
}
