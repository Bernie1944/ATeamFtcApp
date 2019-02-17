package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Degrees;
import org.firstinspires.ftc.teamcode.util.DoubleMap;
import org.firstinspires.ftc.teamcode.util.Vector2;

// Represents bucket
public class Bucket extends Component {
    private static final String PIVOT_SHAFT_MOTOR_NAME = "BucketPivotShaft";
    private static final String SLIDE_MOTOR_NAME = "BucketSlide";
    private static final String TENSIONER_MOTOR_NAME = "BucketTensioner";

    // In revolutions
    private static final double PIVOT_SHAFT_TRAVEL = 15.75;

    // Between [0, 1]
    private static final double PIVOT_SHAFT_POWER_DEADZONE = 0.07;

    // In inches of spool
    private static final double SLIDE_PULLEY_DIAMETER = 3.35; //3.37; //3.1;

    // In inches of slide starting perpendicularly-to-slide below pivot and going forward parallel to slide to approximate center of bucket
    private static final double INITIAL_SLIDE_POSITION = -3.3;
    private static final double MIN_SLIDE_POSITION = -64.25; //-64.5;
    private static final double MAX_SLIDE_POSITION = 56.5; //56.6; // 56 + (7 / 8) //51.36; //46.5;

    // Between [0, 1]
    private static final double SLIDE_POWER_DEADZONE = 0.09;

    private static final double MIN_SLIDE_POWER_WHILE_TENSIONER_MOVING_TO_MAX_POSITION = -0.1;
    private static final double MIN_SLIDE_VELOCITY_WHILE_TENSIONER_MOVING_TO_MAX_POSITION = -0.5;

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
    // If slide is set to drive backwards, tensioner's target position will be set to 1.0 (full tension)
    public final Motor slide;

    // Position between [0, 1] with 1.0 being fully taunt
    // If slide is set to drive backwards, any attempt to set tensioner's power, braking, target velocity, or target position will be ignored
    // and tensioner's target position will remain at 1.0 (full tension)
    public final Motor tensioner;

    private static DoubleMap createSlidePositionToPivotShaftPositionMap() {
        DoubleMap map = new DoubleMap();

        // For lander delivery height

        // For ground height
        map.put(20.860, 0.903);
        map.put(25.435, 0.701);
        map.put(34.349, 0.556);
        map.put(42.575, 0.461);
        map.put(53.381, 0.382);
        map.put(56.745, 0.372);

        return map;
    }

    public Bucket(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);

        pivotShaft = new Motor(
                telemetry, hardwareMap, PIVOT_SHAFT_MOTOR_NAME, 1.0 / PIVOT_SHAFT_TRAVEL, PIVOT_SHAFT_POWER_DEADZONE,
                0.0, 0.0, 1.0
        );

        class SlideMotor extends Motor {
            private SlideMotor(Telemetry telemetry, HardwareMap hardwareMap) {
                super(
                        telemetry, hardwareMap, SLIDE_MOTOR_NAME, -SLIDE_PULLEY_DIAMETER * Math.PI, SLIDE_POWER_DEADZONE,
                        INITIAL_SLIDE_POSITION, MIN_SLIDE_POSITION, MAX_SLIDE_POSITION
                );
            }

            @Override
            public void setPower(double power) {
                if (power < 0.0 && !isPositionAtMin()) {
                    tensioner.setTargetPosition(1.0);

                    if (tensioner.isPositionAtMax()) {
                        super.setPower(power);
                    } else {
                        super.setPower(MIN_SLIDE_POWER_WHILE_TENSIONER_MOVING_TO_MAX_POSITION);
                    }
                } else {
                    super.setPower(power);
                }
            }

            @Override
            public void setTargetVelocity(double targetVelocity) {
                if (targetVelocity < 0.0 && !isPositionAtMin()) {
                    tensioner.setTargetPosition(1.0);

                    if (tensioner.isPositionAtMax()) {
                        super.setTargetVelocity(targetVelocity);
                    } else {
                        super.setTargetVelocity(MIN_SLIDE_VELOCITY_WHILE_TENSIONER_MOVING_TO_MAX_POSITION);
                    }
                } else {
                    super.setTargetVelocity(targetVelocity);
                }
            }

            @Override
            public void setTargetPosition(double targetPosition, double maxSpeed) {
                super.setTargetPosition(targetPosition, maxSpeed);

                if (slide.getTargetPosition() < slide.getPosition() && !slide.isPositionAt(slide.getTargetPosition())) {
                    tensioner.setTargetPosition(1.0);
                }

                if (targetPosition < slide.getPosition() && !slide.isPositionAt(targetPosition)) {
                    tensioner.setTargetPosition(1.0);

                    if (tensioner.isPositionAtMax()) {
                        super.setTargetPosition(targetPosition, maxSpeed);
                    } else {
                        super.setTargetPosition(targetPosition, Math.abs(MIN_SLIDE_VELOCITY_WHILE_TENSIONER_MOVING_TO_MAX_POSITION));
                    }
                } else {
                    super.setTargetPosition(targetPosition, maxSpeed);
                }
            }

            @Override
            public void setTargetPosition(double targetPosition) {
                setTargetPosition(targetPosition, Double.POSITIVE_INFINITY);
            }
        }

        slide = new SlideMotor(telemetry, hardwareMap);

        class TensionerMotor extends Motor {
            private TensionerMotor(Telemetry telemetry, HardwareMap hardwareMap) {
                super(
                        telemetry, hardwareMap, TENSIONER_MOTOR_NAME, 1.0 / TENSIONER_TRAVEL, 0.0,
                        1.0, 0.0, 1.0
                );
            }

            private boolean isSlideDrivingBackwards() {
                return (slide.isPowerSet() && slide.getPower() < 0.0) ||
                        (slide.isTargetVelocitySet() && slide.getTargetVelocity() < 0.0) ||
                        (slide.isTargetPositionSet() && slide.getTargetPosition() < slide.getPosition() && !slide.isPositionAt(slide.getTargetPosition()));
            }

            @Override
            public void setPower(double power) {
                if (!isSlideDrivingBackwards()) {
                    super.setPower(power);
                }
            }

            @Override
            public void brake() {
                if (!isSlideDrivingBackwards()) {
                    super.brake();
                }
            }

            @Override
            public void setTargetVelocity(double targetVelocity) {
                if (!isSlideDrivingBackwards()) {
                    super.setTargetVelocity(targetVelocity);
                }
            }

            @Override
            public void setTargetPosition(double targetPosition, double maxSpeed) {
                if (!isSlideDrivingBackwards() || targetPosition >= 1.0) {
                    super.setTargetPosition(targetPosition, maxSpeed);
                }
            }

            @Override
            public void setTargetPosition(double targetPosition) {
                if (!isSlideDrivingBackwards() || targetPosition >= 1.0) {
                    super.setTargetPosition(targetPosition);
                }
            }
        }

        tensioner = new TensionerMotor(telemetry, hardwareMap);
    }

    // Y-position is inferred from xPosition (See SLIDE_POSITION_TO_PIVOT_SHAFT_POSITION_MAP)
    public boolean isPositionAt(double xPosition) {
        double yPosition = xPosition < 0.0 ? LANDER_DELIVERY_HEIGHT : 0.0;

        double slideLength = PIVOT_POSITION.getDistanceBetween(new Vector2(xPosition, yPosition + BUCKET_DISTANCE_BELOW_ARM_PIVOT));
        double slidePosition = xPosition < 0.0 ? -slideLength : slideLength;

        return slide.isPositionAt(slidePosition) && pivotShaft.isPositionAt(SLIDE_POSITION_TO_PIVOT_SHAFT_POSITION_MAP.get(slidePosition));
    }

    // Target y position is inferred from slide's position (See SLIDE_POSITION_TO_PIVOT_SHAFT_POSITION_MAP)
    public void setTargetYPosition() {
        pivotShaft.setTargetPosition(SLIDE_POSITION_TO_PIVOT_SHAFT_POSITION_MAP.get(slide.getPosition()));
    }

    // Target y-position is inferred from slide's position (See SLIDE_POSITION_TO_PIVOT_SHAFT_POSITION_MAP)
    // This function is not designed to work when bucket is close to being below slide pivot
    public void setTargetXVelocityAndYPosition(double targetXVelocity) {
        double targetYPosition = slide.getPosition() < 0.0 ? LANDER_DELIVERY_HEIGHT : 0.0;

        Vector2 position = new Vector2(Degrees.cos(Degrees.asin(targetYPosition / Math.abs(slide.getPosition()))) * slide.getPosition(), targetYPosition);

        Vector2 positionRelativeToPivot = position.sub(PIVOT_POSITION);

        slide.setTargetVelocity(positionRelativeToPivot.div(positionRelativeToPivot.getX()).mul(targetXVelocity).getMagnitude());

        // This uses slide's current position so the slides rotation can be guided throughout it's movement
        setTargetYPosition();
    }

    // Target y-position is inferred from targetXPosition (See SLIDE_POSITION_TO_PIVOT_SHAFT_POSITION_MAP)
    // This function is not designed to work when targetXPosition is close to being below slide pivot
    public void setTargetPosition(double targetXPosition, double maxSlideSpeed) {
        double targetYPosition = targetXPosition < 0.0 ? LANDER_DELIVERY_HEIGHT : 0.0;

        double targetSlideLength = PIVOT_POSITION.getDistanceBetween(new Vector2(targetXPosition, targetYPosition + BUCKET_DISTANCE_BELOW_ARM_PIVOT));

        slide.setTargetPosition(targetXPosition < 0.0 ? -targetSlideLength : targetSlideLength, maxSlideSpeed);

        // This uses slide's current position so the slides rotation can be guided throughout it's movement
        setTargetYPosition();
    }

    // Target y-position is inferred from targetXPosition (See SLIDE_POSITION_TO_PIVOT_SHAFT_POSITION_MAP)
    // This function is not designed to work when targetXPosition is close to being below slide pivot
    // Moves slide as fast as possible
    public void setTargetPosition(double targetXPosition) {
        setTargetPosition(targetXPosition, Double.POSITIVE_INFINITY);
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