package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.util.Locale;

// Simplifies controlling a DcMotor
// Note: AndyMark AM-2992 encoder cable 4-pin housing is not keyed for FTC dcMotor controllers. Make sure black cable is towards bottom of dcMotor controller.
public class Motor extends Component {
    // In motor ticks
    private static final int DEFAULT_IS_AT_POSITION_THRESHOLD_IN_TICKS = 5;

    // Ftc-provided class that controls motor
    // Does not provide an easy-to-use interface
    // For example, position is given in ticks instead of degrees
    private final DcMotor dcMotor;

    // Used when outputting state to telemetry
    // See String.format()
    // For example "%.2fin"
    private final String unitFormat;

    // Amount getPosition() returns per motor shaft revolution
    // This is < 0.0 if counterclockwise motor shaft rotation (from motor's perspective with back of motor behind and front of motor in front)
    // should produce negative velocities
    // This is > 0.0 if counterclockwise motor shaft rotation (from motor's perspective with back of motor behind and front of motor in front)
    // should produce positive velocities
    private final double gearing;

    // When the magnitude of power is set greater than nearly zero, the motors power will internally be set to a magnitude between [powerDeadzone, 1.0]
    private final double powerDeadzone;

    // The value getSlidePosition() gos to when robot is restarted
    private final double initialPosition;

    // See getter methods
    private final double minPosition;
    private final double maxPosition;
    private double position;
    private double velocity = 0.0;
    private double acceleration = 0.0;

    // name is name used to retrieve DcMotor from hardwareMap
    // unitFormat is used when outputting state to telemetry
    // See String.format()
    // For example "%.2fin"
    // gearing is the amount getPosition() returns per motor shaft revolution
    // This is < 0.0 if counterclockwise motor shaft rotation (from motor's perspective with back of motor behind and front of motor in front)
    // should produce negative velocities
    // This is > 0.0 if counterclockwise motor shaft rotation (from motor's perspective with back of motor behind and front of motor in front)
    // should produce positive velocities
    // powerDeadzone: When the magnitude of power is set greater than nearly zero, the motors power will internally be set to a magnitude between [powerDeadzone, 1.0]
    // initialPosition is the value getPosition() gos to when robot is restarted
    // minPosition is the lower limit that as getPosition() moves beyond setTargetVelocity() will set the target velocity to zero
    // This can be Double.NEGATIVE_INFINITY
    // minPosition is the upper limit that as getPosition() moves beyond setTargetVelocity() will set the target velocity to zero
    // This can be Double.POSITIVE_INFINITY
    // Will leave PID coefficients at defaults (see setPidCoefficients())
    public Motor(Telemetry telemetry, HardwareMap hardwareMap, String name, String unitFormat, double gearing, double powerDeadzone,
                 double initialPosition, double minPosition, double maxPosition
    ) {
        super(telemetry, hardwareMap, name);

        this.unitFormat = unitFormat;
        this.gearing = gearing;
        this.powerDeadzone = powerDeadzone;
        this.initialPosition = initialPosition;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;

        dcMotor = hardwareMap.dcMotor.get(name);

        // Set positive rotation to be counterclockwise when facing motor shaft
        if (dcMotor.getMotorType().getOrientation() == Rotation.CCW) {
            dcMotor.setDirection(DcMotor.Direction.FORWARD);
        } else {
            dcMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        position = (dcMotor.getCurrentPosition() / dcMotor.getMotorType().getTicksPerRev() * gearing) + initialPosition;
    }

    // name is name used to retrieve DcMotor from hardwareMap
    // unitFormat is used when outputting state to telemetry
    // See String.format()
    // For example "%.2fin"
    // gearing is the amount getPosition() returns per motor shaft revolution
    // This is < 0.0 if counterclockwise motor shaft rotation (from motor's perspective with back of motor behind and front of motor in front)
    // should produce negative velocities
    // This is > 0.0 if counterclockwise motor shaft rotation (from motor's perspective with back of motor behind and front of motor in front)
    // should produce positive velocities
    // powerDeadzone: When the magnitude of power is set greater than nearly zero, the motors power will internally be set to a magnitude between [powerDeadzone, 1.0]
    // Will leave PID coefficients at defaults (see setPidCoefficients())
    public Motor(Telemetry telemetry, HardwareMap hardwareMap, String name, String unitFormat, double gearing, double powerDeadzone) {
        this(telemetry, hardwareMap, name, unitFormat, gearing, powerDeadzone, 0.0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    // In units (specified by gearing)
    // The value getSlidePosition() gos to when robot is restarted
    public double getInitialPosition() {
        return initialPosition;
    }

    // In units (specified by gearing)
    // Lower limit that as getPosition() moves beyond setTargetVelocity() will set the target velocity to zero
    // This can be Double.NEGATIVE_INFINITY
    public double getMinPosition() {
        return minPosition;
    }

    // In units (specified by gearing)
    // Upper limit that as getPosition() moves beyond setTargetVelocity() will set the target velocity to zero
    // This can be Double.POSITIVE_INFINITY
    public double getMaxPosition() {
        return maxPosition;
    }

    // In units (specified by gearing) starting at initialPosition
    public double getPosition() {
        return position;
    }

    // Is withing tolerance of position?
    public boolean isPositionAt(double position, double tolerance) {
        return Math.abs(position - getPosition()) <= tolerance;
    }

    // Is close to position?
    public boolean isPositionAt(double position) {
        return isPositionAt(position, DEFAULT_IS_AT_POSITION_THRESHOLD_IN_TICKS / dcMotor.getMotorType().getTicksPerRev() * Math.abs(gearing));
    }

    // Is less than or withing tolerance of position?
    public boolean isPositionLessThanOrAt(double position, double tolerance) {
        return getPosition() < position || isPositionAt(position, tolerance);
    }

    // Is less than or close to position?
    public boolean isPositionLessThanOrAt(double position) {
        return getPosition() < position || isPositionAt(position);
    }

    // Is greater than or withing tolerance of position?
    public boolean isPositionGreaterThanOrAt(double position, double tolerance) {
        return getPosition() > position || isPositionAt(position, tolerance);
    }

    // Is greater than or close to position?
    public boolean isPositionGreaterThanOrAt(double position) {
        return getPosition() > position || isPositionAt(position);
    }

    // Returns true if position <= minPosition
    public boolean isPositionAtMin() {
        return isPositionAt(getMinPosition()) || getPosition() < getMinPosition();
    }

    // Returns true if position >= maxPosition
    public boolean isPositionAtMax() {
        return isPositionAt(getMaxPosition()) || getPosition() > getMaxPosition();
    }

    // In units (specified by gearing) per second
    public double getVelocity() {
        return velocity;
    }

    // In units (specified by gearing) per second per second
    public double getAcceleration() {
        return acceleration;
    }

    // True if motor has set power and is not running with target velocity or target position
    public boolean isPowerSet() {
        return dcMotor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER && dcMotor.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.FLOAT;
    }

    // In amount of power output expressed between [-1, 1] (actual voltage varies with battery voltage)
    // If not running with set power, will return 0.0
    public double getPower() {
        if (isPowerSet()) {
            double powerWithDeadzone = gearing < 0.0 ? -dcMotor.getPower() : dcMotor.getPower();

            if (powerWithDeadzone <= -powerDeadzone) {
                return Range.scale(powerWithDeadzone, -1.0, -powerDeadzone, -1.0, -0.0);
            } else if (powerWithDeadzone >= powerDeadzone) {
                return Range.scale(powerWithDeadzone, powerDeadzone, 1.0, 0.0, 1.0);
            } else {
                return 0.0;
            }
        } else {
            return 0.0;
        }
    }

    // In amount of power output expressed between [-1, 1] (actual voltage varies with battery voltage)
    // If power is outside [-1, 1], it will be clipped
    public void setPower(double power) {
        dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if (Double.isNaN(power) || (power < 0.0 && isPositionAtMin()) || (power > 0.0 && isPositionAtMax())) {
            power = 0.0;
        }

        double powerWithDeadzone;
        if (power <= -Double.MIN_NORMAL) {
            powerWithDeadzone = Range.scale(power, -1.0, -0.0, -1.0, -powerDeadzone);
        } else if (power >= Double.MIN_NORMAL) {
            powerWithDeadzone = Range.scale(power, 0.0, 1.0, powerDeadzone, 1.0);
        } else {
            powerWithDeadzone = 0.0;
        }

        dcMotor.setPower(Range.clip(gearing < 0.0 ? -powerWithDeadzone : powerWithDeadzone, -1.0, 1.0));
    }

    // True if motor is in braking mode and is not running with power, target velocity, or target position
    public boolean isBraking() {
        return dcMotor.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER && dcMotor.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE && dcMotor.getPower() == 0.0;
    }

    // This uses no battery power, and simply connects the motor wires together (the motor is essentially turned into a generator under high load),
    // making the motor harder to turn than setPower(0.0)
    // Use setTargetVelocity(0.0) if more turning resistance is needed
    public void brake() {
        dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcMotor.setPower(0.0);
    }

    // True if motor has set target velocity and is not running with power or target position
    public boolean isTargetVelocitySet() {
        return dcMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER;
    }

    // In units (specified by gearing) per second
    public double getMaxTargetSpeed() {
        return Math.abs(dcMotor.getMotorType().getMaxRPM() / 60.0 * gearing);
    }

    // In units (specified by gearing) per second
    // If not running with target velocity, will return current velocity
    public double getTargetVelocity() {
        if (isTargetVelocitySet()) {
            // dcMotor.getPower() is expressed as fraction of max supported speed
            return dcMotor.getPower() * dcMotor.getMotorType().getMaxRPM() / 60.0 * gearing;
        } else {
            return getVelocity();
        }
    }

    // In units (specified by gearing) per second
    public void setTargetVelocity(double targetVelocity) {
        dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (Double.isNaN(targetVelocity) || (targetVelocity > -Double.MIN_NORMAL && targetVelocity < Double.MIN_NORMAL) ||
                (targetVelocity < 0.0 && isPositionAtMin()) || (targetVelocity > 0.0 && isPositionAtMax())) {
            targetVelocity = 0.0;
        }

        // dcMotor power expressed as fraction of max supported speed between [-1, 1]
        dcMotor.setPower(Range.clip(targetVelocity / gearing * 60.0 / dcMotor.getMotorType().getMaxRPM(), -1.0, 1.0));
    }

    // True if motor has set target position and is not running with power or target velocity
    public boolean isTargetPositionSet() {
        return dcMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION;
    }

    // In units (specified by gearing) relative to initialPosition (which can be provided in the constructor)
    // If not running with target position, will return current position
    public double getTargetPosition() {
        if (isTargetPositionSet()) {
            return (dcMotor.getTargetPosition() / dcMotor.getMotorType().getTicksPerRev() * gearing) + initialPosition;
        } else {
            return getPosition();
        }
    }

    // In units (specified by gearing) per second
    // If not running with target position, will return absolute value of getTargetVelocity()
    public double getMaxSpeedTowardsTargetPosition() {
        if (isTargetPositionSet()) {
            // dcMotor.getPower() is expressed as fraction of max supported speed
            return dcMotor.getPower() * getMaxTargetSpeed();
        } else {
            return Math.abs(getTargetVelocity());
        }
    }

    // targetPosition is in units (specified by gearing) relative to initialPosition (which can be provided in the constructor)
    // maxSpeedTowardsTargetPosition is in units (specified by gearing) per second
    public void setTargetPosition(double targetPosition, double maxSpeedTowardsTargetPosition) {
        dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcMotor.setPower(maxSpeedTowardsTargetPosition >= Double.MIN_NORMAL ?
                Range.clip(maxSpeedTowardsTargetPosition / getMaxTargetSpeed(), 0.0, 1.0) : 0.0
        );

        targetPosition = Range.clip(targetPosition, minPosition, maxPosition);
        dcMotor.setTargetPosition((int) Math.round(
                (targetPosition - initialPosition) / gearing * dcMotor.getMotorType().getTicksPerRev()
        ));
    }

    // targetPosition is in units (specified by gearing) relative to initialPosition (which can be provided in the constructor)
    // Moves motor as fast as possible to targetPosition
    public void setTargetPosition(double targetPosition) {
        setTargetPosition(targetPosition, Double.POSITIVE_INFINITY);
    }

    public boolean isDrivingBackward() {
        return (isPowerSet() && getPower() < 0.0) ||
                (isTargetVelocitySet() && getTargetVelocity() < 0.0) ||
                (isTargetPositionSet() && getTargetPosition() < getPosition() && !isPositionAt(getTargetPosition()) && getMaxSpeedTowardsTargetPosition() > 0.0);
    }

    public boolean isDrivingForward() {
        return (isPowerSet() && getPower() > 0.0) ||
                (isTargetVelocitySet() && getTargetVelocity() > 0.0) ||
                (isTargetPositionSet() && getTargetPosition() > getPosition() && !isPositionAt(getTargetPosition()) && getMaxSpeedTowardsTargetPosition() > 0.0);
    }

    public boolean isDriving() {
        return isDrivingBackward() || isDrivingForward();
    }

    // Called through Component.update()
    @Override
    void internalUpdate() {
        double previousPosition = position;
        double previousVelocity = velocity;

        position = (dcMotor.getCurrentPosition() / dcMotor.getMotorType().getTicksPerRev() * gearing) + initialPosition;
        velocity = (position - previousPosition) / deltaTime;
        acceleration = (velocity - previousVelocity) / deltaTime;
    }

    @Override
    public String toString() {
        return createStateString("targetPositionSet", isTargetPositionSet()) +
                createStateString("targetPosition", unitFormat, getTargetPosition()) +
                createStateString("position", unitFormat, getPosition()) +
                createStateString("positionAtMin", isPositionAtMin()) +
                createStateString("positionAtMax", isPositionAtMax()) +
                createStateString("maxSpeedTowardsTargetPosition", unitFormat + "/s", getMaxSpeedTowardsTargetPosition()) +
                createStateString("targetVelocitySet", isTargetPositionSet()) +
                createStateString("targetVelocity", unitFormat + "/s", getTargetVelocity()) +
                createStateString("velocity", unitFormat + "/s", getVelocity()) +
                createStateString("acceleration", unitFormat + "/s^2", getAcceleration()) +
                createStateString("powerSet", isPowerSet()) +
                createStateString("power", "%.2f", getPower()) +
                createStateString("braking", isBraking()) +
                createStateString("driving", isDriving()) +
                createStateString("drivingBackward", isDrivingBackward()) +
                createStateString("drivingForward", isDrivingForward());
    }
}