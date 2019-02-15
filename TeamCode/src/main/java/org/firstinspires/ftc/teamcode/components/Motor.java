package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

// Simplifies controlling a DcMotor
// Note: AndyMark AM-2992 encoder cable 4-pin housing is not keyed for FTC dcMotor controllers. Make sure black cable is towards bottom of dcMotor controller.
public class Motor extends Component {
    // In motor ticks
    private static final int IS_AT_POSITION_THRESHOLD = 5;

    // Ftc-provided class that controls motor
    // Does not provide an easy-to-use interface
    // For example, position is given in ticks instead of degrees
    private final DcMotor dcMotor;

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

    // deviceName is name used to retrieve DcMotor from hardwareMap
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
    public Motor(Telemetry telemetry, HardwareMap hardwareMap, String deviceName, double gearing, double powerDeadzone,
                 double initialPosition, double minPosition, double maxPosition
    ) {
        super(telemetry, hardwareMap);

        this.gearing = gearing;
        this.powerDeadzone = powerDeadzone;
        this.initialPosition = initialPosition;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;

        dcMotor = hardwareMap.dcMotor.get(deviceName);

        // Set positive rotation to be counterclockwise when facing motor shaft
        if (dcMotor.getMotorType().getOrientation() == Rotation.CCW) {
            dcMotor.setDirection(DcMotor.Direction.FORWARD);
        } else {
            dcMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        position = (dcMotor.getCurrentPosition() / dcMotor.getMotorType().getTicksPerRev() * 360.0 * gearing) + initialPosition;
    }

    // deviceName is name used to retrieve DcMotor from hardwareMap
    // gearing is the amount getPosition() returns per motor shaft revolution
    // This is < 0.0 if counterclockwise motor shaft rotation (from motor's perspective with back of motor behind and front of motor in front)
    // should produce negative velocities
    // This is > 0.0 if counterclockwise motor shaft rotation (from motor's perspective with back of motor behind and front of motor in front)
    // should produce positive velocities
    // powerDeadzone: When the magnitude of power is set greater than nearly zero, the motors power will internally be set to a magnitude between [powerDeadzone, 1.0]
    // Will leave PID coefficients at defaults (see setPidCoefficients())
    public Motor(Telemetry telemetry, HardwareMap hardwareMap, String deviceName, double gearing, double powerDeadzone) {
        this(telemetry, hardwareMap, deviceName, gearing, powerDeadzone, 0.0, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
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
    // This does not loop over so this can be outside of the range [0, 360)
    public double getPosition() {
        return position;
    }

    public boolean isPositionAt(double position) {
        int dcMotorPosition = (int) Math.round((position - initialPosition) / gearing * dcMotor.getMotorType().getTicksPerRev());

        return Math.abs(dcMotorPosition - dcMotor.getCurrentPosition()) <= IS_AT_POSITION_THRESHOLD;
    }

    // Returns true if position <= minPosition
    public boolean isPositionAtMin() {
        return isPositionAt(minPosition) || position < minPosition;
    }

    // Returns true if position >= maxPosition
    public boolean isPositionAtMax() {
        return isPositionAt(maxPosition) || position > maxPosition;
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
    // If not running with set voltage, will return 0.0
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

        if (Double.isNaN(targetVelocity) || (targetVelocity < 0.0 && isPositionAtMin()) || (targetVelocity > 0.0 && isPositionAtMax())) {
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
        if (dcMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            return (dcMotor.getTargetPosition() / dcMotor.getMotorType().getTicksPerRev() * gearing) + initialPosition;
        } else {
            return getPosition();
        }
    }

    // targetPosition is in units (specified by gearing) relative to initialPosition (which can be provided in the constructor)
    // maxSpeed is in units (specified by gearing) per second
    public void setTargetPosition(double targetPosition, double maxSpeed) {
        dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (maxSpeed < 0.0) maxSpeed = 0.0;
        double dcMotorPowerThatLimitsSpeed = Math.abs(maxSpeed / gearing * 60.0 / dcMotor.getMotorType().getMaxRPM());
        if (Double.isNaN(dcMotorPowerThatLimitsSpeed)) dcMotorPowerThatLimitsSpeed = 0.0;
        else if (dcMotorPowerThatLimitsSpeed > 1.0) dcMotorPowerThatLimitsSpeed = 1.0;
        dcMotor.setPower(dcMotorPowerThatLimitsSpeed);

        targetPosition = Range.clip(targetPosition, minPosition, maxPosition);
        dcMotor.setTargetPosition((int) Math.round((targetPosition - initialPosition) / gearing * dcMotor.getMotorType().getTicksPerRev()));
    }

    // targetPosition is in units (specified by gearing) relative to initialPosition (which can be provided in the constructor)
    // Moves motor as fast as possible to targetPosition
    public void setTargetPosition(double targetPosition) {
        setTargetPosition(targetPosition, Double.POSITIVE_INFINITY);
    }

    // pCoefficient is the proportional coefficient used in motor controller PID loop (Ignored if not using a modern robotics motor controller)
    // iCoefficient is the integral coefficient used in motor controller PID loop (Ignored if not using a modern robotics motor controller)
    // dCoefficient is the derivative coefficient used in motor controller PID loop (Ignored if not using a modern robotics motor controller)
    public void setPidCoefficients(double pCoefficient, double iCoefficient, double dCoefficient) {
        if (dcMotor.getController() instanceof ModernRoboticsUsbDcMotorController) {
            ((ModernRoboticsUsbDcMotorController) dcMotor.getController()).setDifferentialControlLoopCoefficients(
                    dcMotor.getPortNumber(),
                    new DifferentialControlLoopCoefficients(pCoefficient, iCoefficient, dCoefficient)
            );
        }
    }

    // Proportional coefficient used in motor controller PID loop
    public double getPCoefficient() {
        if (dcMotor.getController() instanceof ModernRoboticsUsbDcMotorController) {
            return -((ModernRoboticsUsbDcMotorController) dcMotor.getController()).getDifferentialControlLoopCoefficients(dcMotor.getPortNumber()).p;
        } else {
            return 0.0;
        }
    }

    // Proportional coefficient used in motor controller PID loop
    // Takes no action if not used with a modern robotics motor controller
    public void setPCoefficient(double pCoefficient) {
        setPidCoefficients(pCoefficient, getICoefficient(), getDCoefficient());
    }

    // Integral coefficient used in motor controller PID loop
    public double getICoefficient() {
        if (dcMotor.getController() instanceof ModernRoboticsUsbDcMotorController) {
            return ((ModernRoboticsUsbDcMotorController) dcMotor.getController()).getDifferentialControlLoopCoefficients(dcMotor.getPortNumber()).i;
        } else {
            return 0.0;
        }
    }

    // Integral coefficient used in motor controller PID loop
    // Takes no action if not used with a modern robotics motor controller
    public void setICoefficient(double iCoefficient) {
        setPidCoefficients(getPCoefficient(), iCoefficient, getDCoefficient());
    }

    // Derivative coefficient used in motor controller PID loop
    public double getDCoefficient() {
        if (dcMotor.getController() instanceof ModernRoboticsUsbDcMotorController) {
            return -((ModernRoboticsUsbDcMotorController) dcMotor.getController()).getDifferentialControlLoopCoefficients(dcMotor.getPortNumber()).d;
        } else {
            return 0.0;
        }
    }

    // Derivative coefficient used in motor controller PID loop
    // Takes no action if not used with a modern robotics motor controller
    public void setDCoefficient(double dCoefficient) {
        setPidCoefficients(getPCoefficient(), getICoefficient(), dCoefficient);
    }

    // Called through Component.update()
    @Override
    void updateImpl() {
        double previousPosition = position;
        double previousVelocity = velocity;

        position = (dcMotor.getCurrentPosition() / dcMotor.getMotorType().getTicksPerRev() * gearing) + initialPosition;
        velocity = (position - previousPosition) / deltaTime;
        acceleration = (velocity - previousVelocity) / deltaTime;
    }

    // Returns text describing state
    @Override
    public String toString() {
        return "targetPositionSet : " + Boolean.toString(isTargetPositionSet()) + "\n" +
                "targetPosition : " + String.format("%10.3f", getTargetPosition()) + "\n" +
                "position : " + String.format("%10.3f", getPosition()) + "\n" +
                "positionAtMin : " + Boolean.toString(isPositionAtMin()) + "\n" +
                "positionAtMax : " + Boolean.toString(isPositionAtMax()) + "\n" +
                "targetVelocitySet : " + Boolean.toString(isTargetPositionSet()) + "\n" +
                "targetVelocity : " + String.format("%10.3f", getTargetVelocity()) + "\n" +
                "velocity : " + String.format("%10.3f", getVelocity()) + "\n" +
                "acceleration : " + String.format("%10.3f", getAcceleration()) + "\n" +
                "powerSet : " + Boolean.toString(isPowerSet()) + "\n" +
                "power : " + String.format("%5.2f", getPower()) + "\n" +
                "braking : " + Boolean.toString(isBraking());
    }

    // Returns text verbosely describing state
    @Override
    public String toStringVerbose() {
        return toString() + "\n" +
                "minPosition : " + String.format("%10.3f", getMinPosition()) + "\n" +
                "maxPosition : " + String.format("%10.3f", getMaxPosition()) + "\n" +
                "pCoefficient : " + String.format("%8.2f", getPCoefficient()) + "\n" +
                "iCoefficient : " + String.format("%8.2f", getICoefficient()) + "\n" +
                "dCoefficient : " + String.format("%8.2f", getDCoefficient());
    }
}