package org.firstinspires.ftc.teamcode.components;

import java.util.Locale;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.teamcode.util.Inches;

// Simplifies speed controlling a DcMotor
// Note: AndyMark AM-2992 encoder cable 4-pin housing is not keyed for FTC dcMotor controllers. Make sure black cable is towards bottom of dcMotor controller.
public class Motor extends Component {
    // Ftc-provided class that controls motor
    // Does not provide an easy-to-use interface
    // For example, position is given in ticks instead of degrees
    private final DcMotor dcMotor;

    // Amount getPosition() returns per motor shaft degree
    // This is < -1.0  or > 1.0 if geared down
    // This is > 0.0 if counterclockwise motor shaft rotation should produce positive velocities
    // This is < 0.0 if counterclockwise motor shaft rotation should produce negative velocities
    private final double gearing;

    // The value getPosition() gos to when robot is restarted
    private final double initialPosition;

    // Lower limit that as getPosition() moves beyond setTargetVelocity() will set the target velocity to zero
    // This should be less than initialPosition and can be Double.NEGATIVE_INFINITY
    private final double minPosition;

    // Upper limit that as getPosition() moves beyond setTargetVelocity() will set the target velocity to zero
    // This should be greater than initialPosition and can be Double.POSITIVE_INFINITY
    private final double maxPosition;

    // See getter methods
    private double position;
    private double velocity = 0.0;
    private double acceleration = 0.0;

    // deviceName is name used to retrieve DcMotor from hardwareMap
    // shouldBrake indicates whether the motor should brake when target velocity is set to zero
    // (Braking can save power because power is not used to stop the motor, but the motor is essentially turned into a generator under high load;
    // however, I'm not sure if braking is actually ever used when the motor is in run-with-encoders mode as it is when using this class)
    // gearing is the amount getPosition() returns per motor shaft degree
    public Motor(Telemetry telemetry, HardwareMap hardwareMap, String deviceName, boolean shouldBrake, double gearing) {
        super(telemetry, hardwareMap);

        this.dcMotor = createDcMotor(deviceName, shouldBrake);
        this.gearing = gearing;
        this.initialPosition = 0.0;
        this.minPosition = Double.NEGATIVE_INFINITY;
        this.maxPosition = Double.POSITIVE_INFINITY;

        position = (dcMotor.getCurrentPosition() / dcMotor.getMotorType().getTicksPerRev() * 360.0 * gearing) + initialPosition;
    }

    // deviceName is name used to retrieve DcMotor from hardwareMap
    // shouldBrake indicates whether the motor should brake when target velocity is set to zero
    // (Braking can save power because power is not used to stop the motor, but the motor is essentially turned into a generator under high load;
    // however, I'm not sure if braking is actually ever used when the motor is in run-with-encoders mode as it is when using this class)
    // gearing is the amount getPosition() returns per motor shaft degree
    // initialPosition is the value getPosition() gos to when robot is restarted
    // minPosition is the lower limit that as getPosition() moves beyond setTargetVelocity() will set the target velocity to zero
    // This should be less than initialPosition and can be Double.NEGATIVE_INFINITY
    // minPosition is the upper limit that as getPosition() moves beyond setTargetVelocity() will set the target velocity to zero
    // This should be greater than initialPosition and can be Double.POSITIVE_INFINITY
    public Motor(Telemetry telemetry, HardwareMap hardwareMap, String deviceName, boolean shouldBrake, double gearing,
                 double initialPosition, double minPosition, double maxPosition
    ) {
        super(telemetry, hardwareMap);

        this.dcMotor = createDcMotor(deviceName, shouldBrake);
        this.gearing = gearing;
        this.initialPosition = initialPosition;
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;

        position = (dcMotor.getCurrentPosition() / dcMotor.getMotorType().getTicksPerRev() * 360.0 * gearing) + initialPosition;
    }

    private DcMotor createDcMotor(String deviceName, boolean shouldBrake) {
        // Default PID parameters are ratio=25, p=160, i=32, d=112
        DcMotor dcMotor = hardwareMap.get(DcMotor.class, deviceName);
        dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (shouldBrake) {
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // Set positive rotation to be counterclockwise when facing motor shaft
        if (dcMotor.getMotorType().getOrientation() == Rotation.CCW) {
            dcMotor.setDirection(DcMotor.Direction.FORWARD);
        } else {
            dcMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        return dcMotor;
    }

    // In units (implied by gearing) per second
    public double getTargetVelocity() {
        // Power expressed as fraction of max supported speed
        return dcMotor.getPower() * dcMotor.getMotorType().getMaxRPM() / 60.0 * 360.0 * gearing;
    }

    // In units (implied by gearing) per second
    public void setTargetVelocity(double targetVelocity) {
        if ((targetVelocity < 0.0 && getPosition() <= minPosition) || (targetVelocity > 0.0 && getPosition() >= maxPosition)) {
            targetVelocity = 0.0;
        }

        // Power expressed as fraction of max supported speed
        double power = targetVelocity / (dcMotor.getMotorType().getMaxRPM() / 60.0 * 360.0 * gearing);

        // Check for NaN and constrain power
        if (Double.isNaN(power)) power = 0.0;
        if (power < -1.0) power = -1.0;
        else if (power > 1.0) power = 1.0;

        dcMotor.setPower(power);
    }

    // In units (implied by gearing) starting at initialPosition
    // This does not loop over so this can be outside of the range [0, 360)
    public double getPosition() {
        return position;
    }

    // Returns true if position <= minPosition
    public boolean isAtMinPosition() {
        return position <= minPosition;
    }

    // Returns true if position >= maxPosition
    public boolean isAtMaxPosition() {
        return position >= maxPosition;
    }

    // In units (implied by gearing) per second
    public double getVelocity() {
        return velocity;
    }

    // In units (implied by gearing) per second per second
    public double getAcceleration() {
        return acceleration;
    }

    // Called through Component.update()
    @Override
    void updateImpl() {
        double previousPosition = position;
        double previousVelocity = velocity;

        position = (dcMotor.getCurrentPosition() / dcMotor.getMotorType().getTicksPerRev() * 360.0 * gearing) + initialPosition;
        velocity = (position - previousPosition) / deltaTime;
        acceleration = (velocity - previousVelocity) / deltaTime;
    }

    // Returns text describing state
    @Override
    public String toString() {
        return "position :       " + Inches.toString(getPosition()) + "\n" +
                "targetVelocity : " + Inches.toString(getTargetVelocity()) + "\n" +
                "velocity :       " + Inches.toString(getVelocity()) + "\n" +
                "acceleration :   " + Inches.toString(getAcceleration());
    }
}