package org.firstinspires.ftc.teamcode.util;

// Represents a 2 dimensional vector with x and y axises
// Once instantiated, the vector is immutable
// Therefore, it is safe to assign the vector to multiple variables without the possibility of changes to one variable affecting them all
// In addition, this makes a vector variable that is final truly a constant
public class Vector2 {
    public static final Vector2 ZERO = new Vector2(0.0, 0.0);
    public static final Vector2 LEFT = new Vector2(-1.0, 0.0);
    public static final Vector2 RIGHT = new Vector2(1.0, 0.0);
    public static final Vector2 BACKWARD = new Vector2(0.0, -1.0);
    public static final Vector2 FORWARD = new Vector2(0.0, 1.0);
    public static final Vector2 DOWN = new Vector2(0.0, -1.0);
    public static final Vector2 UP = new Vector2(0.0, 1.0);

    private final double x;
    private final double y;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public Vector2 withX(double x) {
        return new Vector2(x, y);
    }

    public Vector2 addX(double n) {
        return withX(x + n);
    }

    public Vector2 subX(double n) {
        return withX(x - n);
    }

    public Vector2 mulX(double n) {
        return withX(x * n);
    }

    public Vector2 divX(double n) {
        return withX(x / n);
    }

    public double getY() {
        return y;
    }

    public Vector2 withY(double y) {
        return new Vector2(x, y);
    }

    public Vector2 addY(double n) {
        return withY(y + n);
    }

    public Vector2 subY(double n) {
        return withY(y - n);
    }

    public Vector2 mulY(double n) {
        return withY(y * n);
    }

    public Vector2 divY(double n) {
        return withY(y / n);
    }

    public double getMagnitude() {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    // Returns vector with specified magnitude, leaving rotation the same
    public Vector2 withMagnitude(double magnitude) {
        return new Vector2(Degrees.cos(getRotation()) * magnitude, Degrees.sin(getRotation()) * magnitude);
    }

    public Vector2 withConstrainedMagnitude(double maxMagnitude) {
        if (getMagnitude() > maxMagnitude) {
            return withMagnitude(maxMagnitude);
        } else {
            return this;
        }
    }

    public Vector2 withNormalizedMagnitude() {
        return withMagnitude(1.0);
    }

    // Returns vector with added magnitude, keeping or reversing the direction
    public Vector2 addMagnitude(double magnitude) {
        return withMagnitude(getMagnitude() + magnitude);
    }

    // Returns vector with subtracted magnitude, keeping or reversing the direction
    public Vector2 subMagnitude(double magnitude) {
        return withMagnitude(getMagnitude() - magnitude);
    }

    public double getDistanceFrom(Vector2 other) {
        return sub(other).getMagnitude();
    }

    // Gets rotation of vector between [-180, 180] starting at positive x axis and going counterclockwise in degrees
    public double getRotation() {
        return Degrees.atan2(y, x);
    }

    // Returns vector with specified rotation starting at right going counterclockwise in degrees, leaving magnitude the same
    public Vector2 withRotation(double rotation) {
        return new Vector2(Degrees.cos(rotation) * getMagnitude(), Degrees.sin(rotation) * getMagnitude());
    }

    // Gets rotation of vector between [-180, 180] starting at referenceVector and going counterclockwise in degrees
    public double getRelativeRotation(Vector2 referenceVector) {
        return Degrees.normalize(getRotation() - referenceVector.getRotation());
    }

    // Returns vector with specified rotation starting at referenceVector and going counterclockwise in degrees, leaving magnitude the same
    public Vector2 withRelativeRotation(Vector2 referenceVector, double rotationFromReference) {
        return withRotation(referenceVector.getRotation() + rotationFromReference);
    }

    // Returns vector with added rotation going counterclockwise in degrees, leaving magnitude the same
    public Vector2 addRotation(double rotation) {
        return withRotation(getRotation() + rotation);
    }

    // Returns vector with subtracted rotation going clockwise in degrees, leaving magnitude the same
    public Vector2 subRotation(double rotation) {
        return withRotation(getRotation() - rotation);
    }

    public double getRotationFrom(Vector2 other) {
        return sub(other).getRotation();
    }

    public Vector2 add(Vector2 other) {
        return new Vector2(x + other.x, y + other.y);
    }

    public Vector2 sub(Vector2 other) {
        return new Vector2(x - other.x, y - other.y);
    }

    public Vector2 mul(double n) {
        return new Vector2(x * n, y * n);
    }

    public Vector2 div(double n) {
        return new Vector2(x / n, y / n);
    }

    @Override
    public boolean equals(Object object) {
        return (object instanceof Vector2) && (x == ((Vector2) object).x) && (y == ((Vector2) object).y);
    }

    @Override
    public String toString() {
        return String.format("(%f, %f) (%f @ %6.1f°)", getX(), getY(), getMagnitude(), getRotation());
    }
}