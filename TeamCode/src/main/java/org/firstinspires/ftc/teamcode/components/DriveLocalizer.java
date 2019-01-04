package org.firstinspires.ftc.teamcode.components;

import java.util.Locale;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.Degrees;
import org.firstinspires.ftc.teamcode.util.Inches;
import org.firstinspires.ftc.teamcode.util.Vector2;
import org.firstinspires.ftc.teamcode.util.Vector3;

// Combines Drive with a Gyro to track robots orientation relative to the playing field
public class DriveLocalizer extends Component {
    private static final String GYRO_NAME = "DriveLocalizerGyro";

    // Controls drive wheels
    // Access to drive is not exposed to avoid confusion between methods such as Drive.setTargetVelocity(), where velocities are relative to the robot's rotation,
    // and DriveLocalizer.setTargetVelocity(), where velocities are relative to the playing field
    // Therefore, instead of using "drive.setTargetVelocity(driveTargetVelocity)", use "nav.setTargetVelocity(driveTargetVelocity.addRotation(gyro.getArmRotation()))"
    // Also access to drive is not exposed to avoid confusion between methods such as Drive.getArmRotationVelocity(), where rotation velocity is calculated using wheels velocities,
    // and Gyro.getArmRotationVelocity(), which is more accurate because it is calculated using gyro velocity
    // (Drive.getArmRotationVelocity() and Drive.getArmRotationAcceleration() still have a use because they are used to determine when Gyro.instantCalibrate() should be called by this class)
    public final Drive drive;

    // For accurately keeping track of robot rotation
    private final Gyro gyro;

    // Used in getAcceleration()
    private Vector2 previousDriveVelocity = Vector2.ZERO;

    // See getter method
    private Vector2 position;

    // initialPosition is in inches relative to center of playing field with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public DriveLocalizer(Telemetry telemetry, HardwareMap hardwareMap, Vector2 initialPosition, double initialRotation) {
        super(telemetry, hardwareMap);
        this.drive = new Drive(telemetry, hardwareMap);
        this.gyro = new Gyro(telemetry, hardwareMap, GYRO_NAME, initialRotation);
        position = initialPosition;
    }

    // In inches per second with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Vector2 getTargetVelocity() {
        return drive.getTargetVelocity().addRotation(gyro.getRotation());
    }

    // In degrees per second with positive counterclockwise
    public double getTargetRotationVelocity() {
        return drive.getTargetRotationVelocity();
    }

    // targetVelocity is in inches per second with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    // targetRotationVelocity is in degrees per second with positive counterclockwise
    // If targetVelocity and targetRotationVelocity are not obtainable, both of these values will be scaled proportionally and targetVelocity's direction will be kept the same
    public void setTargetVelocityAndTargetRotationVelocity(Vector2 targetVelocity, double targetRotationVelocity) {
        drive.setTargetVelocityAndTargetRotationVelocity(targetVelocity.subRotation(gyro.getRotation()), targetRotationVelocity);
    }

    // In inches relative to center of playing field with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Vector2 getPosition() {
        return position;
    }

    // In inches relative to center of playing field with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public void setPosition(Vector2 position) {
        this.position = position;
    }

    // In inches per second with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Vector2 getVelocity() {
        return drive.getVelocity().addRotation(gyro.getRotation());
    }

    // In inches per second per second with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Vector2 getAcceleration() {
        // Acceleration is calculated in this way because "drive.getAcceleration.addRotation(gyro.getArmRotation())" does not produce correct results
        // For example, it would produce non-zero values just because the robot is rotating even if it was moving in a straight line at a constant speed
        // In addition, if acceleration was calculated in updateImpl(), getAcceleration() would not change rotation if gyro.setRotation() were called
        return getVelocity().sub(previousDriveVelocity.addRotation(gyro.getRotation()).div(deltaTime));
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
    public double getRotationVelocity() {
        return gyro.getRotationVelocity();
    }

    // In degrees per second per second with positive going counterclockwise
    public double getRotationAcceleration() {
        return gyro.getRotationAcceleration();
    }

    // Called through Component.update()
    @Override
    void updateImpl() {
        // Used in getAcceleration()
        previousDriveVelocity = drive.getVelocity();

        drive.update();

        position = position.add(getVelocity().mul(deltaTime));

        gyro.update();

        // If robot is at a complete stop (Double.MIN_NORMAL is a very small number), instantCalibrate() gyro
        if (drive.getVelocity().getMagnitude() < Double.MIN_NORMAL && drive.getAcceleration().getMagnitude() < Double.MIN_NORMAL &&
                drive.getRotationVelocity() < Double.MIN_NORMAL && drive.getRotationAcceleration() < Double.MIN_NORMAL) {
            //gyro.instantCalibrate();
        }
    }

    // Returns text describing state
    @Override
    public String toString() {
        return "position :       " + Inches.toString(getPosition()) + "\n" +
                "targetVelocity : " + Inches.toString(getTargetVelocity()) + "\n" +
                "velocity :       " + Inches.toString(getVelocity()) + "\n" +
                "acceleration :   " + Inches.toString(getAcceleration()) + "\n" +
                "rotation :               " + Degrees.toString(getRotation()) + "\n" +
                "targetRotationVelocity : " + Degrees.toString(getTargetRotationVelocity()) + "\n" +
                "rotationVelocity :       " + Degrees.toString(getRotationVelocity()) + "\n" +
                "rotationAcceleration :   " + Degrees.toString(getRotationAcceleration());
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