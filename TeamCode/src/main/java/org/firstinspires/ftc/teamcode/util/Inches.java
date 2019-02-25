package org.firstinspires.ftc.teamcode.util;

// Collection of static functions to help working with inches
public final class Inches {
    public static String toString(double n) {
        return String.format("%10.2f\"", n);
    }

    public static String toString(Vector2 v) {
        return String.format("(%10.2f\", %10.2f\") (%10.2f\" @ %6.1f°)", v.getX(), v.getY(), v.getMagnitude(), v.getRotation());
    }

    // Keep class from being instantiated
    private Inches() {}
}
