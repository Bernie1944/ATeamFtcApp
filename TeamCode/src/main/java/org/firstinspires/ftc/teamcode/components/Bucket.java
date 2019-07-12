package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Degrees;
import org.firstinspires.ftc.teamcode.util.DoubleMap;
import org.firstinspires.ftc.teamcode.util.Vector2;

public class Bucket extends Component {
    private static final String LEFT_SLIDE_MOTOR_NAME = "BucketLeftSlide";
    private static final String RIGHT_SLIDE_MOTOR_NAME = "BucketRightSlide";
    private static final String PIVOT_MOTOR_NAME = "BucketPivotAndLatchDrive";
    private static final String TENSIONER_MOTOR_NAME = "BucketTensioner";

    // In inches of spool
    private static final double SLIDE_PULLEY_DIAMETER = 3.35;

    // In inches of slide starting perpendicularly-to-slide below pivot and going forward parallel to slide to approximate center of bucket dumping position
    private static final double INITIAL_SLIDE_POSITION = -3.8; //-3.3;
    private static final double MIN_SLIDE_POSITION = -62.5; //-61.0; //-60.0; //-63.5; //-63.9;
    private static final double MAX_SLIDE_POSITION = 54.6;
    private static final double GROUND_HEIGHT_MIN_SLIDE_POSITION = 23.4;
    public static final double MINERAL_DELIVERY_HEIGHT_MAX_SLIDE_POSITION = -36.0; //-29.5; //-28.8; //-34.2;
    public static final double MIN_UNEXTENDED_SLIDE_POSITION = -13.0;
    public static final double OPEN_BUCKET_FOR_MINERAL_PICKUP_MIN_SLIDE_POSITION = 2.0; //8.0;
    public static final double MAX_UNEXTENDED_SLIDE_POSITION = 10.0; //8.0;
    public static final double MINERAL_PICKUP_MIN_SLIDE_POSITION = 13.0;

    // Between [0, 1]
    private static final double SLIDE_POWER_DEADZONE = 0.09;

    // In revolutions
    private static final double PIVOT_MOTOR_TRAVEL = 3.175;

    // In revolutions
    private static final double TENSIONER_TRAVEL = 0.32; //0.33; //0.34; //0.4; //0.32; //0.34; //0.35; //0.32; //0.3;

    // Less than 0
    private static final double INITIAL_TENSIONER_POSITION = -0.1;

    // In inches from ground and center of robot to arm pivot with y-axis vertical
    private static final Vector2 PIVOT_POSITION = new Vector2(6.93, 16.9);

    // Height above ground the bucket should be at when delivering minerals into lander
    private static final double MINERAL_DELIVERY_HEIGHT = 34.0;

    // Converts the slide position into a tensioner position that will be used to place the bucket on the crater berm/ground
    private static final DoubleMap SLIDE_POSITION_TO_TENSIONER_POSITION_MAP = createSlidePositionToTensionerPositionMap();

    // Converts the degrees between the nav heading and the line tangent to the crater berms (the crater berms are close to a straight line)
    // into a pivot position that will be used to place the bucket on the crater berm
    private static final DoubleMap NAV_HEADING_FROM_CRATER_BERM_TO_PIVOT_POSITION_MAP = createNavHeadingFromCraterBermToPivotPositionMap();

    // Converts the slide position into a pivot position that will be used to place the bucket on the ground or hold it to the height for delivering minerals into lander
    private static final DoubleMap SLIDE_POSITION_TO_PIVOT_POSITION_MAP = createSlidePositionToPivotPositionMap();

    // In seconds
    private static final double SLIDE_POSITION_LATENCY_CORRECTION = 0.25; //0.2; //0.4; //0.5; //0.3;

    // Between [0, 1]
    private static final double IS_ON_GROUND_SLIDE_POSITION_TO_PIVOT_POSITION_AMOUNT_GREATER_THAN_NAV_HEADING_FROM_CRATER_BERM_TO_PIVOT_POSITION_THRESHOLD = 0.1;

    // Position in inches starting with bucket below pivot and going forward
    public final Motor leftSlide;
    public final Motor rightSlide;

    // Position between [0, 1] with 0.0 being the initial, stored position and 1.0 pulling the pivot fully forward
    public final Motor pivot;

    // Position between [0, 1] with 0.0 being the fully slack position and 1.0 being the initial position that pulls the bucket fully closed
    public final Motor tensioner;

    // Bucket needs access to Nav and Latch
    private final Nav nav;
    private final Latch latch;

    private static DoubleMap createNavHeadingFromCraterBermToPivotPositionMap() {
        DoubleMap map = new DoubleMap();

        map.put(0.0, 0.72); //0.74); //0.75);
        map.put(45.0, 0.6);
        map.put(55.0, 0.58); //0.59);
        return map;
    }

    private static DoubleMap createSlidePositionToPivotPositionMap() {
        DoubleMap map = new DoubleMap();

        // For mineral dump height
        map.put(-62.5, 0.76); //0.75); //0.74); //0.76);
        map.put(-58.0, 0.77); //0.76); //0.78); //0.79);
        map.put(-50.0, 0.80); //0.79); //0.8);
        map.put(-42.0, 0.85); //0.84); //0.85);
        map.put(-36.0, 0.90); //0.89); //0.85);
        //map.put(MINERAL_DELIVERY_HEIGHT_MAX_SLIDE_POSITION, 1.0);

        // For mineral scoop height
        //map.put(19.2, 0.68);
        map.put(19.5, 1.0);
        map.put(23.4, 0.74); //0.76);
        map.put(54.6, 0.5); //0.57);

        return map;
    }

    private static DoubleMap createSlidePositionToTensionerPositionMap() {
        DoubleMap map = new DoubleMap();

        map.put(30.0, 0.22);
        map.put(MAX_SLIDE_POSITION, 0.0);
        return map;
    }

    public Bucket(Telemetry telemetry, HardwareMap hardwareMap, Nav nav, Latch latch) {
        super(telemetry, hardwareMap);

        this.nav = nav;
        this.latch = latch;

        leftSlide = new Motor(
                telemetry, hardwareMap, LEFT_SLIDE_MOTOR_NAME, "%.2fin", -SLIDE_PULLEY_DIAMETER * Math.PI, SLIDE_POWER_DEADZONE,
                INITIAL_SLIDE_POSITION, MIN_SLIDE_POSITION, MAX_SLIDE_POSITION
        );

        rightSlide = new Motor(
                telemetry, hardwareMap, RIGHT_SLIDE_MOTOR_NAME, "%.2fin", SLIDE_PULLEY_DIAMETER * Math.PI, SLIDE_POWER_DEADZONE,
                INITIAL_SLIDE_POSITION, MIN_SLIDE_POSITION, MAX_SLIDE_POSITION
        );

        pivot = new Motor(
                telemetry, hardwareMap, PIVOT_MOTOR_NAME, "%.2f", 1.0 / PIVOT_MOTOR_TRAVEL, 0.0,
                0.0, 0.0, 1.0
        );

        tensioner = new Motor(
                telemetry, hardwareMap, TENSIONER_MOTOR_NAME, "%.2f", 1.0 / TENSIONER_TRAVEL, 0.0,
                INITIAL_TENSIONER_POSITION, 0.0, 1.0
        );
    }

    // In inches relative to the nav with positive towards front of the robot
    // Assumes pivot is at the target position of setPivotTargetPosition()
    public static double convertSlidePositionToRelativePosition(double slidePosition) {
        // Rough estimate of y position as if slide is at target position after calling setPivotTargetPosition()
        double estimatedHeight = Range.clip(
                Range.scale(
                        slidePosition,
                        MINERAL_DELIVERY_HEIGHT_MAX_SLIDE_POSITION, GROUND_HEIGHT_MIN_SLIDE_POSITION,
                        MINERAL_DELIVERY_HEIGHT, 0.0),
                0.0, MINERAL_DELIVERY_HEIGHT
        );

        double horizontalDistanceFromSlidePivot = Math.sqrt(Math.pow(slidePosition, 2) - Math.pow(estimatedHeight - PIVOT_POSITION.getY(), 2));
        if (Double.isNaN(horizontalDistanceFromSlidePivot)) horizontalDistanceFromSlidePivot = 0.0;

        return (slidePosition < 0.0 ? -horizontalDistanceFromSlidePivot : horizontalDistanceFromSlidePivot) + PIVOT_POSITION.getX();
    }

    public static double convertRelativePositionToSlidePosition(double relativePosition) {
        // Rough estimate of y position as if slide is at target position after calling setPivotTargetPosition()
        double estimatedHeight = Range.clip(
                Range.scale(
                        relativePosition,
                        convertSlidePositionToRelativePosition(MINERAL_DELIVERY_HEIGHT_MAX_SLIDE_POSITION),
                        convertSlidePositionToRelativePosition(GROUND_HEIGHT_MIN_SLIDE_POSITION),
                        MINERAL_DELIVERY_HEIGHT, 0.0),
                0.0, MINERAL_DELIVERY_HEIGHT
        );

        double absSlidePosition = new Vector2(relativePosition, estimatedHeight).sub(PIVOT_POSITION).getMagnitude();

        return relativePosition < PIVOT_POSITION.getX() ? -absSlidePosition : absSlidePosition;
    }

    // In inches starting with bucket below pivot and going forward
    // Same as leftSlide.getPosition() but with latency correction applied
    public double getSlidePosition() {
        return leftSlide.getPosition() + (leftSlide.getVelocity() * SLIDE_POSITION_LATENCY_CORRECTION);
    }

    // In inches relative to the nav with positive towards front of the robot
    // Assumes pivot is at the target position of setPivotTargetPosition()
    public double getRelativePosition() {
        return convertSlidePositionToRelativePosition(getSlidePosition());
    }

    // In inches relative to center of playing field with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Vector2 getPosition() {
        return new Vector2(getRelativePosition(), 0.0).addRotation(nav.getHeading()).add(nav.getPosition());
    }

    // Is bucket resting on ground?
    public boolean isOnGround() {
        return getSlidePosition() > MAX_UNEXTENDED_SLIDE_POSITION &&
                SLIDE_POSITION_TO_PIVOT_POSITION_MAP.get(getSlidePosition()) - NAV_HEADING_FROM_CRATER_BERM_TO_PIVOT_POSITION_MAP.get(getNavHeadingFromCraterBerm()) <
                        IS_ON_GROUND_SLIDE_POSITION_TO_PIVOT_POSITION_AMOUNT_GREATER_THAN_NAV_HEADING_FROM_CRATER_BERM_TO_PIVOT_POSITION_THRESHOLD;
    }

    // Calculate the degrees between the nav heading and the line tangent to the crater berms (the crater berms are close to a straight line)
    private double getNavHeadingFromCraterBerm() {
        double navHeadingFromCraterBerm = Degrees.between(nav.getHeading(), 45.0);
        if (navHeadingFromCraterBerm > 90.0) navHeadingFromCraterBerm = 180.0 - navHeadingFromCraterBerm;
        return navHeadingFromCraterBerm;
    }

    // Pivot target position is inferred from getSlidePosition() and nav heading so that bucket is
    // held to height for delivering minerals into lander, placed along the crater berm, or placed on the ground
    public void setPivotTargetPosition() {
        if (latch.getCatchEngagementAmount() > 0.0) {
            latch.disengage();
        } else if (getSlidePosition() < MIN_UNEXTENDED_SLIDE_POSITION) {
            pivot.setTargetPosition(SLIDE_POSITION_TO_PIVOT_POSITION_MAP.get(getSlidePosition()));
        } else {
            pivot.setTargetPosition(Math.min(
                    NAV_HEADING_FROM_CRATER_BERM_TO_PIVOT_POSITION_MAP.get(getNavHeadingFromCraterBerm()),
                    SLIDE_POSITION_TO_PIVOT_POSITION_MAP.get(getSlidePosition())
            ));
        }
    }

    // Tensioner target position is inferred from getSlidePosition() and nav heading so that bucket is
    // placed on the crater berm or ground when in front of the robot or is otherwise closed
    public void setTensionerTargetPosition() {
        if (getSlidePosition() < OPEN_BUCKET_FOR_MINERAL_PICKUP_MIN_SLIDE_POSITION) {
            tensioner.setTargetPosition(tensioner.getMaxPosition());
        } else if (isOnGround()){
            tensioner.setTargetPosition(SLIDE_POSITION_TO_TENSIONER_POSITION_MAP.get(getSlidePosition()));
        } else {
            tensioner.setTargetPosition(tensioner.getMinPosition());
        }
    }

    // Called through Component.update()
    @Override
    void internalUpdate() {
        leftSlide.update();
        rightSlide.update();
        pivot.update();
        tensioner.update();
    }

    @Override
    public String toString() {
        return createStateString("slidePosition", "%.2fin", getSlidePosition()) +
                createStateString("relativePosition", "%.2fin", getRelativePosition()) +
                createStateString("position", getPosition().toString("%.2fin")) +
                createStateString("onGround", isOnGround()) +
                leftSlide.toString() + rightSlide.toString() + pivot.toString() + tensioner.toString();
    }
}