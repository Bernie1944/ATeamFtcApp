package org.firstinspires.ftc.teamcode.util;

// Collection of static functions to help working with degrees
public final class Degrees {
    public static String toString(double n) {
        return String.format("%10.1f°", n);
    }

    public static double fromRadians(double radians) {
        return Math.toDegrees(radians);
    }

    public static double toRadians(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double sin(double degrees) {
        return Math.sin(toRadians(degrees));
    }

    public static double cos(double degrees) {
        return Math.cos(toRadians(degrees));
    }

    public static double tan(double degrees) {
        return Math.tan(toRadians(degrees));
    }

    public static double asin(double oppositeOverHypotenuse) {
        return fromRadians(Math.asin(oppositeOverHypotenuse));
    }

    public static double acos(double adjacentOverHypotenuse) {
        return fromRadians(Math.acos(adjacentOverHypotenuse));
    }

    public static double atan(double oppositeOverAdjacent) {
        return fromRadians(Math.atan(oppositeOverAdjacent));
    }

    public static double atan2(double opposite, double adjacent) {
        return fromRadians(Math.atan2(opposite, adjacent));
    }

    // Unwraps degrees so that they are between [-180, 180]
    public static double normalize(double degrees) {
        return atan2(sin(degrees), cos(degrees));
    }

    // Keep class from being instantiated
    private Degrees() {}
}
