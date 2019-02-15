package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoEx extends Component {
    // Ftc-provided class that controls servo
    private final Servo servo;

    private final double approximateMaxVelocity;

    private double approximatePosition;

    // targetPosition is recorded here instead of being read by servo.getPosition()
    // because using servo.setPosition() and then servo.getPosition() converts the position to an unscaled range and then converts it back to a scaled range,
    // possibly not resulting in the exact same value,
    // which could cause equality operators used with getTargetPosition() and setTargetPosition() to not work as expected
    private double targetPosition = Double.NaN;

    ServoEx(Telemetry telemetry, HardwareMap hardwareMap, String deviceName, boolean directionReversed, double minUnscaledPosition, double maxUnscaledPosition,
            double initialApproximatePosition, double approximateMaxVelocity) {
        super(telemetry, hardwareMap);
        approximatePosition = initialApproximatePosition;
        this.approximateMaxVelocity = approximateMaxVelocity;

        servo = hardwareMap.servo.get(deviceName);
        servo.setDirection(directionReversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo.scaleRange(minUnscaledPosition, maxUnscaledPosition);
    }

    public double getApproximatePosition() {
        return approximatePosition;
    }

    public void setApproximatePosition(double approximatePosition) {
        this.approximatePosition = Range.clip(approximatePosition, 0.0, 1.0);
    }

    boolean isTargetPositionSet() {
        return !Double.isNaN(targetPosition);
    }

    double getTargetPosition() {
        if (Double.isNaN(targetPosition)) {
            return getApproximatePosition();
        } else {
            return targetPosition;
        }
    }

    void setTargetPosition(double targetPosition) {
        targetPosition = Range.clip(targetPosition, 0.0, 1.0);
        servo.setPosition(targetPosition);
        this.targetPosition = targetPosition;
    }

    // Called through Component.update()
    @Override
    void updateImpl() {
        if (approximatePosition < getTargetPosition()) {
            approximatePosition += approximateMaxVelocity * deltaTime;

            if (approximatePosition > getTargetPosition()) approximatePosition = getTargetPosition();
        } else {
            approximatePosition -= approximateMaxVelocity * deltaTime;

            if (approximatePosition < getTargetPosition()) approximatePosition = getTargetPosition();
        }
    }

    // Returns text describing state
    @Override
    public String toString() {
        return "targetPositionSet : " + Boolean.toString(isTargetPositionSet()) + "\n" +
                "targetPosition : " + String.format("%4.2f", getTargetPosition()) + "\n" +
                "approximatePosition : " + String.format("%4.2f", getApproximatePosition());
    }
}
