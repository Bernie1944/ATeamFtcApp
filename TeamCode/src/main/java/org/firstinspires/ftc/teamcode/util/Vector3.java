package org.firstinspires.ftc.teamcode.util;

// Represents a 3 dimensional vector with x, y, and z axises
// Once instantiated, the vector is immutable
// Therefore, it is safe to assign the vector to multiple variables without the possibility of changes to one variable affecting them all
// In addition, this makes a vector variable that is final truly a constant
public class Vector3 {
    public static final Vector3 ZERO = new Vector3(0.0, 0.0, 0.0);
    public static final Vector3 LEFT = new Vector3(-1.0, 0.0, 0.0);
    public static final Vector3 RIGHT = new Vector3(1.0, 0.0, 0.0);
    public static final Vector3 BACKWARD = new Vector3(0.0, -1.0, 0.0);
    public static final Vector3 FORWARD = new Vector3(0.0, 1.0, 0.0);
    public static final Vector3 DOWN = new Vector3(0.0, 0.0, -1.0);
    public static final Vector3 UP = new Vector3(0.0, 0.0,1.0);

    private final double x;
    private final double y;
    private final double z;

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double getX() {
        return x;
    }

    public Vector3 withX(double x) {
        return new Vector3(x, y, z);
    }

    public Vector3 addX(double n) {
        return withX(x + n);
    }

    public Vector3 subX(double n) {
        return withX(x - n);
    }

    public Vector3 mulX(double n) {
        return withX(x * n);
    }

    public Vector3 divX(double n) {
        return withX(x / n);
    }

    public double getY() {
        return y;
    }

    public Vector3 withY(double y) {
        return new Vector3(x, y, z);
    }

    public Vector3 addY(double n) {
        return withY(y + n);
    }

    public Vector3 subY(double n) {
        return withY(y - n);
    }

    public Vector3 mulY(double n) {
        return withY(y * n);
    }

    public Vector3 divY(double n) {
        return withY(y / n);
    }

    public double getZ() {
        return z;
    }

    public Vector3 withZ(double z) {
        return new Vector3(x, y, z);
    }

    public Vector3 addZ(double n) {
        return withZ(z + n);
    }

    public Vector3 subZ(double n) {
        return withZ(z - n);
    }

    public Vector3 mulZ(double n) {
        return withZ(z * n);
    }

    public Vector3 divZ(double n) {
        return withZ(z / n);
    }

    public Vector2 getXY() {
        return new Vector2(x, y);
    }

    public Vector3 withXY(Vector2 xY) {
        return new Vector3(xY.getX(), xY.getY(), z);
    }

    public Vector3 addXY(Vector2 xY) {
        return withXY(getXY().add(xY));
    }

    public Vector3 subXY(Vector2 xY) {
        return withXY(getXY().sub(xY));
    }

    public Vector3 mulXY(double n) {
        return withXY(getXY().mul(n));
    }

    public Vector3 divXY(double n) {
        return withXY(getXY().div(n));
    }

    public Vector2 getXZ() {
        return new Vector2(x, z);
    }

    public Vector3 withXZ(Vector2 vector2) {
        return new Vector3(vector2.getX(), y, vector2.getY());
    }

    public Vector3 addXZ(Vector2 xZ) {
        return withXZ(getXZ().add(xZ));
    }

    public Vector3 subXZ(Vector2 xZ) {
        return withXZ(getXZ().sub(xZ));
    }

    public Vector3 mulXZ(double n) {
        return withXZ(getXZ().mul(n));
    }

    public Vector3 divXZ(double n) {
        return withXZ(getXZ().div(n));
    }

    public Vector2 getYZ() {
        return new Vector2(y, z);
    }

    public Vector3 withYZ(Vector2 vector2) {
        return new Vector3(x, vector2.getX(), vector2.getY());
    }

    public Vector3 addYZ(Vector2 yZ) {
        return withYZ(getYZ().add(yZ));
    }

    public Vector3 subYZ(Vector2 yZ) {
        return withYZ(getYZ().sub(yZ));
    }

    public Vector3 mulYZ(double n) {
        return withYZ(getYZ().mul(n));
    }

    public Vector3 divYZ(double n) {
        return withYZ(getYZ().div(n));
    }

    public double getMagnitude() {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }

    // Returns vector with specified magnitude, leaving rotation the same
    public Vector3 withMagnitude(double magnitude) {
        return mul(magnitude / getMagnitude());
    }

    public Vector3 withConstrainedMagnitude(double maxMagnitude) {
        if (getMagnitude() > maxMagnitude) {
            return withMagnitude(maxMagnitude);
        } else {
            return this;
        }
    }

    public Vector3 withNormalizedMagnitude() {
        return withMagnitude(1.0);
    }

    // Returns vector with added magnitude, keeping or reversing the direction
    public Vector3 addMagnitude(double magnitude) {
        return withMagnitude(getMagnitude() + magnitude);
    }

    // Returns vector with subtracted magnitude, keeping or reversing the direction
    public Vector3 subMagnitude(double magnitude) {
        return withMagnitude(getMagnitude() - magnitude);
    }

    public Vector3 add(Vector3 other) {
        return new Vector3(x + other.x, y + other.y, z + other.z);
    }

    public Vector3 sub(Vector3 other) {
        return new Vector3(x - other.x, y - other.y, z - other.z);
    }

    public Vector3 mul(double n) {
        return new Vector3(x * n, y * n, z * n);
    }

    public Vector3 div(double n) {
        return new Vector3(x / n, y / n, z / n);
    }

    @Override
    public String toString() {
        return String.format("(%f, %f, %f) (%f)", getX(), getY(), getZ(), getMagnitude());
    }
}