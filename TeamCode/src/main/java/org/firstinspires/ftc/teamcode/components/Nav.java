package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.Degrees;
import org.firstinspires.ftc.teamcode.util.Inches;
import org.firstinspires.ftc.teamcode.util.Vector2;

// Combines Drive with a Gyro to track robots orientation relative to the playing field
public class Nav extends Component {
    private static final String GYRO_NAME = "NavGyro";

    // Controls drive wheels
    // Access to drive is not exposed to avoid confusion between methods such as Drive.setTargetVelocity(), where velocities are relative to the robot's rotation,
    // and Nav.setTargetVelocity(), where velocities are relative to the playing field
    // Therefore, instead of using "drive.setTargetVelocity(driveTargetVelocity)", use "nav.setTargetVelocity(driveTargetVelocity.addRotation(gyro.getRotation()))"
    // Also access to drive is not exposed to avoid confusion between methods such as Drive.getAngularVelocity(), where rotation velocity is calculated using wheels velocities,
    // and Gyro.getAngularVelocity(), which is more accurate because it is calculated using gyro velocity
    // (Drive.getAngularVelocity() and Drive.getAngularAcceleration() still have a use because they are used to determine when Gyro.instantCalibrate() should be called by this class)
    public final Drive drive;

    // For accurately keeping track of robot rotation
    private final Gyro gyro;

    // See getter method
    private static Vector2 position = Vector2.ZERO;

    // initialPosition is in inches relative to center of playing field with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Nav(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);
        this.drive = new Drive(telemetry, hardwareMap);
        this.gyro = new Gyro(telemetry, hardwareMap, GYRO_NAME);
    }

    // In inches relative to center of playing field with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Vector2 getPosition() {
        return position;
    }

    // In inches relative to center of playing field with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public static void setPosition(Vector2 position) {
        Nav.position = position;
    }

    // In inches per second with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Vector2 getVelocity() {
        return drive.getVelocity().addRotation(gyro.getRotation());
    }

    // In degrees starting with robot facing positive x-axis (facing right from the the driving team's perspective) going counterclockwise
    // This does not loop over, meaning it can be outside a range of 360 degrees
    public double getRotation() {
        return gyro.getRotation();
    }

    // In degrees starting with robot facing positive x-axis (facing right from the the driving team's perspective) going counterclockwise
    // rotation does not have to be looped over, meaning it can be outside a range of 360 degrees
    public void setRotation(double rotation) {
        gyro.setRotation(rotation);
    }

    // In degrees per second with positive going counterclockwise
    public double getAngularVelocity() {
        return gyro.getAngularVelocity();
    }

    // True if drive has set target velocity and angular velocity and is not running to target position and rotation
    public boolean areTargetVelocitiesSet() {
        return drive.areTargetVelocitiesSet();
    }

    // In inches per second with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Vector2 getTargetVelocity() {
        return drive.getTargetVelocity().addRotation(gyro.getRotation());
    }

    // In degrees per second with positive counterclockwise
    public double getTargetAngularVelocity() {
        return drive.getTargetAngularVelocity();
    }

    // targetVelocity is in inches per second with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    // targetAngularVelocity is in degrees per second with positive counterclockwise
    // If targetVelocity and targetAngularVelocity are not obtainable, both of these values will be scaled proportionally and targetVelocity's direction will be kept the same
    public void setTargetVelocities(Vector2 targetVelocity, double targetAngularVelocity) {
        drive.setTargetVelocities(targetVelocity.subRotation(gyro.getRotation()), targetAngularVelocity);
    }

    // True if drive has set target position and rotation and is not running with target velocity and angular velocity
    public boolean isTargetOrientationSet() {
        return drive.isTargetRelativeOrientationSet();
    }

    // In inches relative to center of playing field with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Vector2 getTargetPosition() {
        return drive.getTargetRelativePosition().addRotation(getRotation()).add(getPosition());
    }

    // In degrees starting with robot facing positive x-axis (facing right from the the driving team's perspective) going counterclockwise
    // This does not loop over, meaning it can be outside a range of 360 degrees
    public double getTargetRotation() {
        return drive.getTargetRelativeRotation() + getRotation();
    }

    // targetRotation does not loop over, meaning it can be outside a range of 360 degrees
    // If this is not taken into consideration, it can cause the robot to spin around multiple times before it gets to targetRotation
    public void setTargetOrientation(Vector2 targetPosition, double targetRotation, double maxVelocity, double maxAngularVelocity) {
        drive.setTargetRelativeOrientation(targetPosition.sub(getPosition()).subRotation(getRotation()), targetRotation - getRotation(),
                maxVelocity, maxAngularVelocity);
    }

    // targetRotation does not loop over, meaning it can be outside a range of 360 degrees
    // Max velocities are infinity
    public void setTargetOrientation(Vector2 targetPosition, double targetRotation) {
        setTargetOrientation(targetPosition, targetRotation, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    // Called through Component.update()
    @Override
    void updateImpl() {
        drive.update();
        gyro.update();

        position = position.add(getVelocity().mul(deltaTime));
    }

    // Returns text describing state
    @Override
    public String toString() {
        return "targetOrientationSet : " + Boolean.toString(isTargetOrientationSet()) + "\n" +
                "targetPosition : " + Inches.toString(getTargetPosition()) + "\n" +
                "position : " + Inches.toString(getPosition()) + "\n" +
                "targetRotation : " + Inches.toString(getTargetRotation()) + "\n" +
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
                "drive {\n" +
                drive.toStringVerbose() + "\n" +
                "}\n" +
                "gyro {\n" +
                gyro.toStringVerbose() + "\n" +
                "}";
    }
}