package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private static final double PIVOT_SHAFT_POWER_DEADZONE = 0.07;

    // In inches of spool
    private static final double SLIDE_PULLEY_DIAMETER = 3.35;

    // In inches of slide starting perpendicularly-to-slide below pivot and going forward parallel to slide to approximate center of bucket
    private static final double INITIAL_SLIDE_POSITION = -3.3;
    private static final double MIN_SLIDE_POSITION = -64.25;
    private static final double MAX_SLIDE_POSITION = 56.5;

    // Between [0, 1]
    private static final double SLIDE_POWER_DEADZONE = 0.09;

    private static final double MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD = 0.25;

    private static final double MAX_SLIDE_POWER_MAGNITUDE_WHILE_TENSIONER_MOVING_TO_ALLOWED_POSITION = 0.1;
    private static final double MAX_SLIDE_SPEED_WHILE_TENSIONER_MOVING_TO_ALLOWED_POSITION = 0.5;

    // In revolutions
    private static final double TENSIONER_TRAVEL = 0.5;

    // In inches from ground and center of robot to arm pivot with y-axis vertical
    private static final Vector2 PIVOT_POSITION = new Vector2(6.93, 16.9);

    // In inches from slide pivot to bucket measured perpendicular to slide
    private static final double BUCKET_DISTANCE_BELOW_ARM_PIVOT = 8.0;

    // Height above ground the bucket should be at when delivering minerals into lander
    // This only affects the slide's position, not the pivotShaft's position
    private static final double LANDER_DELIVERY_HEIGHT = 34.0;

    // Converts the slides position into the target position for the pivotShaft that will either
    // place the bucket on the ground (if bucket is in front of the robot) or will raise bucket to height for delivering minerals into lander
    private static final DoubleMap SLIDE_POSITION_TO_PIVOT_SHAFT_POSITION_MAP = createSlidePositionToPivotShaftPositionMap();

    private static final double MAX_SLIDE_POSITION_FOR_DUMPING_MINERALS = -30.0;
    private static final double MIN_SLIDE_POSITION_FOR_SCOOPING_MINERALS = 16.0;

    // Position between [0, 1] with 0.0 being the initial stored position
    public final Motor pivotShaft;

    // Position in inches starting with bucket below pivot and going forward
    // If slide is set to drive backwards, tensioner's target position will be set to 1.0 (full tension)
    public final SlideMotor slide;

    // Position between [0, 1] with 1.0 being fully taunt
    // If slide is set to drive backwards, any attempt to set tensioner's power, braking, target velocity, or target position will be ignored
    // and tensioner's target position will remain at 1.0 (full tension)
    public final TensionerMotor tensioner;

    public class SlideMotor extends Motor {
        private SlideMotor(Telemetry telemetry, HardwareMap hardwareMap) {
            super(
                    telemetry, hardwareMap, SLIDE_MOTOR_NAME, -SLIDE_PULLEY_DIAMETER * Math.PI, SLIDE_POWER_DEADZONE,
                    INITIAL_SLIDE_POSITION, MIN_SLIDE_POSITION, MAX_SLIDE_POSITION
            );
        }

        @Override
        public void setPower(double power) {
            if (power < -Double.MIN_NORMAL && !isPositionAtMin()) {
                tensioner.setTargetPosition(1.0);

                if (tensioner.isPositionAtMax()) {
                    super.setPower(power);
                } else {
                    super.setPower(-MAX_SLIDE_POWER_MAGNITUDE_WHILE_TENSIONER_MOVING_TO_ALLOWED_POSITION);
                }
            } else if (power > Double.MIN_NORMAL && !isPositionAtMax()) {
                if (tensioner.getTargetPosition() < MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD) {
                    tensioner.setTargetPosition(MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD);
                }

                if (tensioner.isPositionAt(MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD) ||
                        tensioner.getPosition() > MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD) {
                    super.setPower(power);
                } else {
                    super.setPower(MAX_SLIDE_POWER_MAGNITUDE_WHILE_TENSIONER_MOVING_TO_ALLOWED_POSITION);
                }
            } else {
                super.setPower(power);
            }
        }

        @Override
        public void setTargetVelocity(double targetVelocity) {
            if (targetVelocity < -Double.MIN_NORMAL && !isPositionAtMin()) {
                tensioner.setTargetPosition(1.0);

                if (tensioner.isPositionAtMax()) {
                    super.setTargetVelocity(targetVelocity);
                } else {
                    super.setTargetVelocity(-MAX_SLIDE_SPEED_WHILE_TENSIONER_MOVING_TO_ALLOWED_POSITION);
                }
            } else if (targetVelocity > Double.MIN_NORMAL && !isPositionAtMax()) {
                if (tensioner.getTargetPosition() < MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD) {
                    tensioner.setTargetPosition(MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD);
                }

                if (tensioner.isPositionAt(MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD) ||
                        tensioner.getPosition() > MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD) {
                    super.setTargetVelocity(targetVelocity);
                } else {
                    super.setTargetVelocity(MAX_SLIDE_SPEED_WHILE_TENSIONER_MOVING_TO_ALLOWED_POSITION);
                }
            } else {
                super.setTargetVelocity(targetVelocity);
            }
        }

        @Override
        public void setTargetPosition(double targetPosition, double maxSpeedTowardsTargetPosition) {
            if (targetPosition < slide.getPosition() && !slide.isPositionAt(targetPosition)) {
                tensioner.setTargetPosition(1.0);

                if (tensioner.isPositionAtMax()) {
                    super.setTargetPosition(targetPosition, maxSpeedTowardsTargetPosition);
                } else {
                    super.setTargetPosition(targetPosition, MAX_SLIDE_SPEED_WHILE_TENSIONER_MOVING_TO_ALLOWED_POSITION);
                }
            } else if (targetPosition > slide.getPosition() && !slide.isPositionAt(targetPosition)) {
                if (tensioner.getTargetPosition() < MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD) {
                    tensioner.setTargetPosition(MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD);
                }

                if (tensioner.isPositionAt(MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD) ||
                        tensioner.getPosition() > MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD) {
                    super.setTargetPosition(targetPosition, maxSpeedTowardsTargetPosition);
                } else {
                    super.setTargetPosition(targetPosition, MAX_SLIDE_SPEED_WHILE_TENSIONER_MOVING_TO_ALLOWED_POSITION);
                }
            } else {
                super.setTargetPosition(targetPosition, maxSpeedTowardsTargetPosition);
            }
        }
    }

    public class TensionerMotor extends Motor {
        private TensionerMotor(Telemetry telemetry, HardwareMap hardwareMap) {
            super(
                    telemetry, hardwareMap, TENSIONER_MOTOR_NAME, 1.0 / TENSIONER_TRAVEL, 0.0,
                    1.0, 0.0, 1.0
            );
        }

        @Override
        public void setPower(double power) {
            if (!slide.isDriving()) {
                super.setPower(power);
            }
        }

        @Override
        public void brake() {
            if (!slide.isDriving()) {
                super.brake();
            }
        }

        @Override
        public void setTargetVelocity(double targetVelocity) {
            if (!slide.isDriving()) {
                super.setTargetVelocity(targetVelocity);
            }
        }

        @Override
        public void setTargetPosition(double targetPosition, double maxSpeedTowardsTargetPosition) {
            if (slide.isDrivingBackward()) {
                targetPosition = 1.0;
            } else if (slide.isDrivingForward() && targetPosition < MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD) {
                targetPosition = MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD;
            }

            if ((slide.isDrivingBackward() && !isPositionAtMax()) ||
                    (slide.isDrivingForward() && getPosition() < MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD &&
                            !isPositionAt(MIN_TENSIONER_POSITION_WHILE_SLIDE_SLIDE_MOVING_FORWARD))
            ) {
                super.setTargetPosition(targetPosition, Double.POSITIVE_INFINITY);
            } else {
                super.setTargetPosition(targetPosition, maxSpeedTowardsTargetPosition);
            }
        }
    }

    private static DoubleMap createSlidePositionToPivotShaftPositionMap() {
        DoubleMap map = new DoubleMap();

        // For mineral dump height
        map.put(-63.721, 0.674);
        map.put(-52.990, 0.714);
        map.put(-46.111, 0.767);
        map.put(-38.668, 0.839);
        map.put(-31.020, 1.000);

        // For mineral scoop height
        map.put(16.583, 0.649);
        map.put(20.399, 0.669);
        map.put(22.184, 0.664);
        map.put(25.567, 0.576);
        map.put(28.329, 0.510);
        map.put(31.036, 0.480);
        map.put(33.930, 0.453);
        map.put(36.993, 0.426);
        map.put(40.883, 0.391);
        map.put(44.435, 0.372);
        map.put(47.696, 0.349);
        map.put(50.788, 0.340);
        map.put(52.498, 0.327);
        map.put(56.388, 0.303);

        return map;
    }

    public Bucket(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);

        pivotShaft = new Motor(
                telemetry, hardwareMap, PIVOT_SHAFT_MOTOR_NAME, 1.0 / PIVOT_SHAFT_TRAVEL, PIVOT_SHAFT_POWER_DEADZONE,
                0.0, 0.0, 1.0
        );

        slide = new SlideMotor(telemetry, hardwareMap);
        tensioner = new TensionerMotor(telemetry, hardwareMap);
    }

    // In inches starting with bucket below pivot and going forward
    public double getMaxSlidePositionForDumpingMinerals() {
        return MAX_SLIDE_POSITION_FOR_DUMPING_MINERALS;
    }

    // In inches starting with bucket below pivot and going forward
    public double getMinSlidePositionForScoopingMinerals() {
        return MIN_SLIDE_POSITION_FOR_SCOOPING_MINERALS;
    }

    // See correctYPosition()
    public boolean isYPositionCorrect() {
        return pivotShaft.isPositionAt(SLIDE_POSITION_TO_PIVOT_SHAFT_POSITION_MAP.get(slide.getPosition()));
    }

    // In inches as if isYPositionCorrect() returns true
    public double getXPositionForCorrectYPosition() {
        double correctYPosition = slide.getPosition() < 0.0 ? LANDER_DELIVERY_HEIGHT : 0.0;

        double xDistanceFromSlidePivot = Math.sqrt(Math.pow(slide.getPosition(), 2) - Math.pow(correctYPosition - PIVOT_POSITION.getY(), 2));
        if (Double.isNaN(xDistanceFromSlidePivot)) xDistanceFromSlidePivot = 0.0;

        return (slide.getPosition() < 0.0 ? -xDistanceFromSlidePivot : xDistanceFromSlidePivot) + PIVOT_POSITION.getX();
    }

    // Target y position is inferred from slide's position (See SLIDE_POSITION_TO_PIVOT_SHAFT_POSITION_MAP)
    // Returns [0, 1]
    // 0.0 indicates y-position is far enough below target y-position that slide should not move
    // 1.0 indicates y-position is high enough that slide movement should not be reduced
    public void correctYPosition() {
        pivotShaft.setTargetPosition(SLIDE_POSITION_TO_PIVOT_SHAFT_POSITION_MAP.get(slide.getPosition()));
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
        return "yPositionCorrect : " + Boolean.toString(isYPositionCorrect()) + "\n" +
                "xPositionForCorrectYPosition : " + Inches.toString(getXPositionForCorrectYPosition()) + "\n" +
                "pivotShaft {\n" +
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
        return "yPositionCorrect : " + Boolean.toString(isYPositionCorrect()) + "\n" +
                "xPositionForCorrectYPosition : " + Inches.toString(getXPositionForCorrectYPosition()) + "\n" +
                "pivotShaft {\n" +
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