package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.DoubleMap;
import org.firstinspires.ftc.teamcode.util.Inches;
import org.firstinspires.ftc.teamcode.util.Vector2;

// Represents bucket
public class Bucket extends Component {
    private static final String PIVOT_SHAFT_MOTOR_NAME = "BucketPivotShaft";
    private static final String SLIDE_MOTOR_NAME = "BucketSlide";
    private static final String TENSIONER_MOTOR_NAME = "BucketTensioner";

    // In revolutions
    private static final double PIVOT_SHAFT_TRAVEL = 18.5;

    // Between [0, 1]
    private static final double PIVOT_SHAFT_POWER_DEADZONE = 0.06;

    // In inches of spool
    private static final double SLIDE_PULLEY_DIAMETER = 3.1;

    // In inches of slide starting perpendicularly-to-slide below pivot and going forward parallel to slide to approximate center of bucket
    private static final double INITIAL_SLIDE_POSITION = -3.3;
    private static final double MIN_SLIDE_POSITION = -64.5;
    private static final double MAX_SLIDE_POSITION = 51.36; //46.5;

    // Between [0, 1]
    private static final double SLIDE_POWER_DEADZONE = 0.06;

    // In revolutions
    private static final double TENSIONER_TRAVEL = 0.5;

    // In inches from ground and center of robot to arm pivot with y-axis vertical
    private static final Vector2 PIVOT_POSITION = new Vector2(6.93, 16.9);

    // In inches from slide pivot to bucket measured perpendicular to slide
    private static final double BUCKET_DISTANCE_BELOW_ARM_PIVOT = 8.0;

    // Height above ground the bucket should be at when delivering minerals into lander
    // This only affects the slide's position, not the pivotShaft's position
    private static final double LANDER_DELIVERY_HEIGHT = 32.0;

    // Converts the slides position into the target position for the pivotShaft that will either
    // place the bucket on the ground (if bucket is in front of the robot) or will raise bucket to height for delivering minerals into lander
    private static final DoubleMap SLIDE_POSITION_TO_PIVOT_SHAFT_POSITION_MAP = createSlidePositionToPivotShaftPositionMap();

    // Position between [0, 1] with 0.0 being the initial stored position
    public final Motor pivotShaft;

    // Position in inches starting with bucket below pivot and going forward
    private final Motor slide;

    // Position between [0, 1] with 1.0 being fully taunt
    private final Motor tensioner;

    private static DoubleMap createSlidePositionToPivotShaftPositionMap() {
        DoubleMap map = new DoubleMap();

        // For lander delivery height

        // For ground height
        map.put(-2.0, 0.0);
        map.put(0.0, 16.86);
        map.put(16.5, 16.86);
        map.put(20.0, 12.49);
        map.put(30.0, 8.14);
        map.put(40.0, 6.17);
        map.put(50.0, 5.08);
        map.put(51.36, 4.90);

        return map;
    }

    public Bucket(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);

        pivotShaft = new Motor(
                telemetry, hardwareMap, PIVOT_SHAFT_MOTOR_NAME, 1.0 / PIVOT_SHAFT_TRAVEL, PIVOT_SHAFT_POWER_DEADZONE,
                0.0, 0.0, 1.0
        );

        slide = new Motor(
                telemetry, hardwareMap, SLIDE_MOTOR_NAME, -SLIDE_PULLEY_DIAMETER * Math.PI, SLIDE_POWER_DEADZONE,
                INITIAL_SLIDE_POSITION, MIN_SLIDE_POSITION, MAX_SLIDE_POSITION
        );

        tensioner = new Motor(
                telemetry, hardwareMap, TENSIONER_MOTOR_NAME, 1.0 / TENSIONER_TRAVEL, 0.0,
                1.0, 0.0, 1.0
        );
    }

    // In inches of slide starting below slide pivot
    public double getMinPosition() {
        return slide.getMinPosition();
    }

    // In inches of slide starting below slide pivot
    public double getMaxPosition() {
        return slide.getMaxPosition();
    }

    // In inches of slide starting below slide pivot
    public double getPosition() {
        return slide.getPosition();
    }

    // In inches of slide starting below slide pivot
    public boolean isPositionAt(double position) {
        return slide.isPositionAt(position);
    }

    // Returns true if position <= minPosition
    public boolean isPositionAtMin() {
        return slide.isPositionAtMin();
    }

    // Returns true if position >= maxPosition
    public boolean isPositionAtMax() {
        return slide.isPositionAtMax();
    }

    // In inches per second
    public double getVelocity() {
        return slide.getVelocity();
    }

    // In inches per second per second
    public double getAcceleration() {
        return slide.getAcceleration();
    }

    // Between [0, 1] with 1.0 being fully taunt
    public double getTension() {
        return tensioner.getPosition();
    }

    // Between [0, 1] with 1.0 being fully taunt
    public double getTargetTension() {
        return tensioner.getTargetPosition();
    }

    // True if slide has set power and is not running with target velocity or target position
    public boolean isPowerSet() {
        return slide.isPowerSet();
    }

    public double getPower() {
        return slide.getPower();
    }

    // power is amount of power output of slide expressed between [-1, 1] (actual voltage varies with battery voltage)
    // targetTension is between [0, 1] with 0.0 being full tension
    // If power is negative, targetTension will be ignored and will instead be set to fully taunt
    public void setPowerAndTargetTension(double power, double targetTension) {
        slide.setPower(power);

        if (slide.getPower() < 0.0) {
            tensioner.setTargetPosition(1.0);
        } else {
            tensioner.setTargetPosition(targetTension);
        }
    }

    public boolean isBraking() {
        return slide.isBraking();
    }

    // This uses no battery power, and simply connects the motor wires together (the motor is essentially turned into a generator under high load),
    // making the motor harder to turn than setPower(0.0)
    // Use setTargetVelocity(0.0) if more turning resistance is needed
    public void brakeAndSetTargetTension(double targetTension) {
        slide.brake();

        tensioner.setTargetPosition(targetTension);
    }

    // True if slide has set target velocity and is not running with power or target position
    public boolean isTargetVelocitySet() {
        return slide.isTargetVelocitySet();
    }

    // In inches per second
    // If not running with target velocity, will return current velocity
    public double getTargetVelocity() {
        return slide.getTargetVelocity();
    }

    // targetVelocity is in inches per second of slide
    // targetTension is between [0, 1] with 0.0 being full tension
    // If target velocity is negative, targetTension will be ignored and will instead be set to fully taunt
    public void setTargetVelocityAndTargetTension(double targetVelocity, double targetTension) {
        slide.setTargetVelocity(targetVelocity);

        if (slide.getTargetVelocity() < 0.0) {
            tensioner.setTargetPosition(1.0);
        } else {
            tensioner.setTargetPosition(targetTension);
        }
    }

    // True if slide has set target position and is not running with power or target velocity
    public boolean isTargetPositionSet() {
        return slide.isTargetPositionSet();
    }

    // In inches of slide starting below slide pivot
    // If not running with target position, will return current position
    public double getTargetPosition() {
        return slide.getTargetPosition();
    }

    // targetPosition is in inches starting below slide pivot and going forward
    // targetTension is between [0, 1] with 0.0 being full tension
    // maxSpeed is in inches per second
    // If target position is behind current position, targetTension will be ignored and will instead be set to fully taunt
    public void setTargetPositionAndTargetTension(double targetPosition, double targetTension, double maxSpeed) {
        slide.setTargetPosition(targetPosition, maxSpeed);

        if (slide.getTargetPosition() < slide.getPosition() && !slide.isPositionAt(slide.getTargetPosition())) {
            tensioner.setTargetPosition(1.0);
        } else {
            tensioner.setTargetPosition(targetTension);
        }
    }

    // targetPosition is in inches starting below slide pivot and going forward
    // targetTension is between [0, 1] with 0.0 being full tension
    // Moves motor as fast as possible to targetPosition
    // If target position is behind current position, targetTension will be ignored and will instead be set to fully taunt
    public void setTargetPositionAndTargetTension(double targetPosition, double targetTension) {
        setTargetPositionAndTargetTension(targetPosition, targetTension, Double.POSITIVE_INFINITY);
    }

    public boolean isXPositionAt(double xPosition) {
        double yPosition = xPosition < 0.0 ? LANDER_DELIVERY_HEIGHT : 0.0;

        double slideLength = PIVOT_POSITION.getDistanceBetween(new Vector2(xPosition, yPosition + BUCKET_DISTANCE_BELOW_ARM_PIVOT));
        double slidePosition = xPosition < 0.0 ? -slideLength : slideLength;

        return slide.isPositionAt(slidePosition) && pivotShaft.isPositionAt(SLIDE_POSITION_TO_PIVOT_SHAFT_POSITION_MAP.get(slidePosition));
    }

    public void setTargetYPosition() {
        pivotShaft.setTargetPosition(SLIDE_POSITION_TO_PIVOT_SHAFT_POSITION_MAP.get(slide.getPosition()));
    }

    public void setTargetXPositionAndYPositionAndTension(double targetXPosition, double targetTension, double maxSlideSpeed) {
        double targetYPosition = targetXPosition < 0.0 ? LANDER_DELIVERY_HEIGHT : 0.0;

        double targetSlideLength = PIVOT_POSITION.getDistanceBetween(new Vector2(targetXPosition, targetYPosition + BUCKET_DISTANCE_BELOW_ARM_PIVOT));

        setTargetPositionAndTargetTension(targetXPosition < 0.0 ? -targetSlideLength : targetSlideLength, targetTension, maxSlideSpeed);

        setTargetYPosition();
    }

    public void setTargetXPositionAndYPositionAndTension(double targetXPosition, double targetTension) {
        setTargetXPositionAndYPositionAndTension(targetXPosition, targetTension, Double.POSITIVE_INFINITY);
    }

    // Called through Component.update()
    @Override
    void updateImpl() {
        pivotShaft.update();
        slide.update();
        tensioner.update();
    }

    // Returns text describing state
    @Override
    public String toString() {
        return "pivotShaft {\n" +
                pivotShaft.toString() + "\n" +
                "}\n" +
                "slide {\n" +
                slide.toString() + "\n" +
                "}\n" +
                "tensioner {\n" +
                tensioner.toString() + "\n" +
                "}";
    }

    // Returns text verbosely describing state
    @Override
    public String toStringVerbose() {
        return "pivotShaft {\n" +
                pivotShaft.toStringVerbose() + "\n" +
                "}\n" +
                "slide {\n" +
                slide.toStringVerbose() + "\n" +
                "}\n" +
                "tensioner {\n" +
                tensioner.toStringVerbose() + "\n" +
                "}";
    }
}