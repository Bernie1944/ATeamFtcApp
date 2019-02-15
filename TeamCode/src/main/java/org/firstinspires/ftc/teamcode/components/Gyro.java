package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.util.Degrees;

// Simplifies using a Modern Robotics internalGyro to read the z-axis
public class Gyro extends Component {
    // In degrees per second of how internalGyro's angular velocity reading is off from what it should actually be
    private static final double INTERNAL_GYRO_ANGULAR_VELOCITY_ERROR = 0.0; //0.081818;

    // Ftc-provided class for using MR gyro
    private final ModernRoboticsI2cGyro internalGyro;

    // In degrees of how far rotation is from internalGyro's rotation reading
    private double rotationOffsetFromInternalGyro;

    // See getter methods
    private double rotation;
    private double angularVelocity;
    private double angularAcceleration = 0.0;

    public Gyro(Telemetry telemetry, HardwareMap hardwareMap, String deviceName, double initialRotation) {
        super(telemetry, hardwareMap);

        internalGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, deviceName);

        // Set rotationOffsetFromInternalGyro to how far initialRotation is from internalGyro's rotation reading
        rotationOffsetFromInternalGyro = initialRotation - internalGyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        rotation = initialRotation;
        angularVelocity = internalGyro.getAngularVelocity(AngleUnit.DEGREES).zRotationRate - INTERNAL_GYRO_ANGULAR_VELOCITY_ERROR;
    }

    // In degrees going counterclockwise
    // This does not loop over, meaning it can be outside a range of 360 degrees
    public double getRotation() {
        return rotation;
    }

    // In degrees going counterclockwise
    // rotation does not have to be looped over, meaning it can be outside a range of 360 degrees
    public void setRotation(double rotation) {
        rotationOffsetFromInternalGyro += rotation - this.rotation;
        this.rotation = rotation;
    }

    // In degrees per second with positive going counterclockwise
    public double getAngularVelocity() {
        return angularVelocity;
    }

    // In degrees per second per second with positive going counterclockwise
    public double getAngularAcceleration() {
        return angularAcceleration;
    }

    public void calibrate() {
        internalGyro.calibrate();

        while (internalGyro.isCalibrating()) {
            telemetry.addLine("Gyro Calibrating...");
            telemetry.addLine();
            telemetry.addLine("DO NOT MOVE!");
            telemetry.update();

            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                return;
            }
        }

        // Save calibration
        internalGyro.writeCommand(ModernRoboticsI2cGyro.Command.WRITE_EEPROM);
        internalGyro.writeCommand(ModernRoboticsI2cGyro.Command.NORMAL);

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            return;
        }
    }

    // Called through Component.update()
    @Override
    void updateImpl() {
        rotationOffsetFromInternalGyro -= INTERNAL_GYRO_ANGULAR_VELOCITY_ERROR * deltaTime;

        rotation = internalGyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + rotationOffsetFromInternalGyro;

        double previousAngularVelocity = angularVelocity;
        angularVelocity = internalGyro.getAngularVelocity(AngleUnit.DEGREES).zRotationRate - INTERNAL_GYRO_ANGULAR_VELOCITY_ERROR;

        angularAcceleration = (angularVelocity - previousAngularVelocity) / deltaTime;
    }

    // Returns text describing state
    @Override
    public String toString() {
        return "rotation : " + Degrees.toString(getRotation()) + "\n" +
                "angularVelocity : " + Degrees.toString(getAngularVelocity()) + "\n" +
                "angularAcceleration : " + Degrees.toString(getAngularAcceleration());
    }

    // Returns text verbosely describing state
    @Override
    public String toStringVerbose() {
        return toString() + "\n" +
                "rotationOffsetFromInternalGyro : " + Degrees.toString(rotationOffsetFromInternalGyro);
    }
}