package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.Degrees;
import org.firstinspires.ftc.teamcode.util.Inches;
import org.firstinspires.ftc.teamcode.util.Vector2;

// Combines Drive with a gyro to track robots orientation relative to the playing field
public class Nav extends Component {
    private static final String GYRO_NAME = "NavGyro";

    // Controls drive wheels
    // Access to drive is not exposed to avoid confusion between methods such as Drive.setTargetVelocities(), where target velocity is relative to the robot's rotation,
    // and Nav.setTargetVelocities(), where target velocity is relative to the playing field
    // In addition, Nav.getAngularVelocity() is more accurate than Drive.getAngularVelocity() because Nav uses a gyro
    public final Drive drive;

    // For accurately determining robot rotation
    private final ModernRoboticsI2cGyro gyro;

    // In degrees
    private double rotationOffsetFromGyro;

    // See getter method
    private static Vector2 position = Vector2.ZERO;
    private static double rotation = 0.0;
    private double angularVelocity;

    // initialPosition is in inches relative to center of playing field with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Nav(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);
        drive = new Drive(telemetry, hardwareMap);
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, GYRO_NAME);

        // Set rotationOffsetFromGyro to how far initialRotation is from gyro's rotation reading
        rotationOffsetFromGyro = rotation - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        angularVelocity = gyro.getAngularVelocity(AngleUnit.DEGREES).zRotationRate;
    }

    // In inches relative to center of playing field with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Vector2 getPosition() {
        return position;
    }

    // In inches relative to center of playing field with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public void setPosition(Vector2 position) {
        Nav.position = position;
    }

    // In inches per second with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Vector2 getVelocity() {
        return drive.getVelocity().addRotation(getRotation());
    }

    // In degrees between [-180, 180] starting with robot facing positive x-axis (facing right from the the driving team's perspective) going counterclockwise
    public double getRotation() {
        return rotation;
    }

    // In degrees starting with robot facing positive x-axis (facing right from the the driving team's perspective) going counterclockwise
    // If rotation is outside [-180, 180] it will be looped over into this range
    public void setRotation(double rotation) {
        rotationOffsetFromGyro += rotation - Nav.rotation;
        Nav.rotation = rotation;
    }

    // In degrees per second with positive going counterclockwise
    public double getAngularVelocity() {
        return angularVelocity;
    }

    // True if drive has set target velocity and angular velocity and is not running to target position and rotation
    public boolean areTargetVelocitiesSet() {
        return drive.areTargetVelocitiesSet();
    }

    // In inches per second with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Vector2 getTargetVelocity() {
        return drive.getTargetVelocity().addRotation(getRotation());
    }

    // In degrees per second with positive counterclockwise
    public double getTargetAngularVelocity() {
        return drive.getTargetAngularVelocity();
    }

    // targetVelocity is in inches per second with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    // targetAngularVelocity is in degrees per second with positive counterclockwise
    // If targetVelocity and targetAngularVelocity are not obtainable, both of these values will be scaled proportionally and targetVelocity's direction will be kept the same
    public void setTargetVelocities(Vector2 targetVelocity, double targetAngularVelocity) {
        drive.setTargetVelocities(targetVelocity.subRotation(getRotation()), targetAngularVelocity);
    }

    // Called through Component.update()
    @Override
    void updateImpl() {
        drive.update();

        position = position.add(getVelocity().mul(deltaTime));

        rotation = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + rotationOffsetFromGyro;
        angularVelocity = gyro.getAngularVelocity(AngleUnit.DEGREES).zRotationRate;
    }

    // Returns text describing state
    @Override
    public String toString() {
        return "position : " + Inches.toString(getPosition()) + "\n" +
                "rotation : " + Degrees.toString(getRotation()) + "\n" +
                "targetVelocitiesSet : " + Boolean.toString(areTargetVelocitiesSet()) + "\n" +
                "targetVelocity : " + Inches.toString(getTargetVelocity()) + "\n" +
                "velocity : " + Inches.toString(getVelocity()) + "\n" +
                "targetAngularVelocity : " + Degrees.toString(getTargetAngularVelocity()) + "\n" +
                "angularVelocity : " + Degrees.toString(getAngularVelocity());
    }

    // Returns text verbosely describing state
    @Override
    public String toStringVerbose() {
        return toString() + "\n" +
                "rotationOffsetFromGyro : " + Degrees.toString(rotationOffsetFromGyro) + "\n" +
                "drive {\n" +
                drive.toStringVerbose() + "\n" +
                "}";
    }
}