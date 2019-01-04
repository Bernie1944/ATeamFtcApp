package org.firstinspires.ftc.teamcode.util;

// Collection of static functions to help working with inches
public final class Inches {
    public static String toString(double n) {
        return String.format("%10.2f\"", n);
    }

    public static String toString(Vector2 v) {
        return String.format("(%10.2f\", %10.2f\") (%10.2f\" @ %6.1f°)", v.getX(), v.getY(), v.getMagnitude(), v.getRotation());
    }

    public static String toString(Vector3 v) {
        return String.format("(%10.2f\", %10.2f\", %10.2f\") (%10.2f\")", v.getX(), v.getY(), v.getZ(), v.getMagnitude());
    }

    public static double fromCentimeters(double centimeters) {
        return centimeters / 2.54;
    }
    public static Vector2 fromCentimeters(Vector2 centimeters) {
        return centimeters.div(2.54);
    }
    public static Vector3 fromCentimeters(Vector3 centimeters) {
        return centimeters.div(2.54);
    }

    public static double toCentimeters(double inches) {
        return inches * 2.54;
    }
    public static Vector2 toCentimeters(Vector2 inches) {
        return inches.mul(2.54);
    }
    public static Vector3 toCentimeters(Vector3 inches) {
        return inches.mul(2.54);
    }

    // Keep class from being instantiated
    private Inches() {}
}
