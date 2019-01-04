package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.Degrees;
import org.firstinspires.ftc.teamcode.util.Inches;
import org.firstinspires.ftc.teamcode.util.Vector2;

// Controls bucket system
public class Bucket extends Component {
    private static final String LIFT_MOTOR_NAME = "BucketLift";
    private static final String SLIDE_MOTOR_NAME = "BucketSlide";
    private static final String ROTATION_LINE_MOTOR_NAME = "BucketRotationLine";

    private static final double LIFT_GEAR_RATIO = -3.0;
    private static final double LIFT_THREADS_PER_INCH = 16.0;

    // In inches from lift's universal joint to lift's upper pivot
    private static final double INITIAL_LIFT_POSITION = 8.5;
    private static final double MIN_LIFT_POSITION = 8.7;
    private static final double MAX_LIFT_POSITION = 12.1;

    // In degrees per second
    private static final double MAX_ARM_ROTATION_SPEED = 45.0;

    // In degrees per second per second
    private static final double MAX_ARM_ROTATION_ACCELERATION_FROM_STOP = 180.0;

    // In degrees per inch starting below arm pivot
    private static final double ARM_ROTATION_SAG_PER_SLIDE_POSITION = 0.0378;

    // In inches from arm pivot to lift's upper pivot
    private static final double ARM_PIVOT_TO_LIFT_PIVOT_DISTANCE = 3.31;

    // In inches from arm pivot to lift's universal joint
    private static final double ARM_PIVOT_TO_LIFT_JOINT_DISTANCE = 10.72;

    // In degrees starting forward going counterclockwise to angle formed from lift's universal joint to arm pivot
    private static final double LIFT_JOINT_TO_ARM_PIVOT_ROTATION = 74.0;

    // In inches from ground and center of robot to arm pivot with y-axis vertical
    private static final Vector2 ARM_PIVOT_POSITION = new Vector2(6.93, 17.06);

    // In inches from arm pivot to bucket measured perpendicular to slide
    private static final double BUCKET_DISTANCE_BELOW_ARM_PIVOT = 8.75;

    // In inches of spool
    private static final double SLIDE_PULLEY_DIAMETER = 3.2;

    // In inches of bucket starting perpendicularly-to-slide below arm pivot and going forward parallel to slide
    private static final double INITIAL_SLIDE_POSITION = -9.0;
    private static final double MIN_SLIDE_POSITION = -64.5;
    private static final double MAX_SLIDE_POSITION = 46.5;
    private static final double MIN_SLIDE_POSITION_MAGNITUDE_FOR_NONZERO_TARGET_ARM_ROTATION_VELOCITY = 4.0;

    // In inches per second
    private static final double MAX_SLIDE_SPEED = 72.0;

    // In inches per second per second when velocity equals zero
    private static final double MAX_SLIDE_ACCELERATION_FROM_STOP = 288.0;

    // In inches of spool
    private static final double ROTATION_LINE_PULLEY_DIAMETER = 3.3;

    // In inches from where rotation line leaves pulley to where it goes through the eyelet mounted beside the bucket measured relative to slide when slide position is zero
    private static final Vector2 ROTATION_LINE_EYELET_POSITION_RELATIVE_TO_PULLEY_WHEN_SLIDE_POSITION_IS_ZERO = new Vector2(5.5, -3.0);

    // In inches from where rotation line is connected to spring to eyelet
    private static final double MIN_DISTANCE_BETWEEN_ROTATION_LINE_EYELET_AND_SPRING = 1.69;
    private static final double MAX_DISTANCE_BETWEEN_ROTATION_LINE_EYELET_AND_SPRING = 6.5;

    // In inches between where line leaves pulley and connects to harvester
    private static final double INITIAL_ROTATION_LINE_POSITION = 7.19;
    private static final double MIN_ROTATION_LINE_POSITION = 3.0;
    private static final double MAX_ROTATION_LINE_POSITION = 80.0;

    // In inches
    // Defines a box around robot where harvester can not freely move in
    // Used in setTargetVelocity() to keep harvester from hitting robot
    private static final double MAX_X_POSITION_BEHIND_ROBOT = -9.0;
    private static final double MIN_X_POSITION_IN_FRONT_OF_ROBOT = 10.0;
    private static final double MIN_Y_POSITION_ABOVE_ROBOT = 4.3;

    // In inches
    // Defines a width inside both sides of the box defined by the 3 constants above so the following three conditions can be determined in setTargetVelocity():
    // If harvester is next to back of robot,
    // make make forward movement instead move harvester up to clear top of robot and make downward movement instead move harvester back away from back of robot
    // If harvester is next to front of robot,
    // make make backward movement instead move harvester up to clear top of robot and make downward movement instead move harvester forward away from front of robot
    // If harvester is next to top of robot,
    // make make forward or backward movement instead move harvester up to clear top of robot and do not allow downward movement
    private static final double SIDE_ACTION_WIDTH = 1.0;

    public final Motor lift;
    public final Motor slide;
    public final Motor rotationLine;

    // See getter methods
    private Vector2 velocity = Vector2.ZERO;
    private Vector2 acceleration = Vector2.ZERO;
    private double rotationAmountVelocity = 0.0;
    private double rotationAmountAcceleration = 0.0;
    private double armRotationVelocity = 0.0;
    private double armRotationAcceleration = 0.0;

    public Bucket(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);

        lift = new Motor(
                telemetry, hardwareMap, LIFT_MOTOR_NAME, true, LIFT_GEAR_RATIO / 360.0 / LIFT_THREADS_PER_INCH,
                INITIAL_LIFT_POSITION, MIN_LIFT_POSITION, MAX_LIFT_POSITION
        );

        slide = new Motor(
                telemetry, hardwareMap, SLIDE_MOTOR_NAME, true, SLIDE_PULLEY_DIAMETER * Math.PI / 360.0,
                INITIAL_SLIDE_POSITION, MIN_SLIDE_POSITION, MAX_SLIDE_POSITION
        );

        rotationLine = new Motor(
                telemetry, hardwareMap, ROTATION_LINE_MOTOR_NAME, true, -ROTATION_LINE_PULLEY_DIAMETER * Math.PI / 360.0,
                INITIAL_ROTATION_LINE_POSITION, MIN_ROTATION_LINE_POSITION, MAX_ROTATION_LINE_POSITION
        );
    }

    // In inches per second of front bottom lip of harvester
    // X coordinate is horizontal and relative to center of robot
    // Y coordinate is vertical and relative to ground
    public Vector2 getTargetVelocity() {
        // Using Law of Cosines (c^2 = a^2 + b^2 - 2ab cos(C)) derivative, find clockwise target arm rotation velocity from the known lift target velocity
        // The the angle C is getAngleFromLiftJointToLiftPivotAboutArmPivot(),
        // the angle from LIFT_JOINT_TO_ARM_PIVOT_ROTATION to slide (this angle is positive and therefore goes clockwise)
        // c is the side of the triangle opposite C, making it the lift position
        // a and b are the other sides of the triangle, making them constants ARM_PIVOT_TO_LIFT_PIVOT_DISTANCE and ARM_PIVOT_TO_LIFT_JOINT_DISTANCE
        // (which one is a and which one is b does not make a difference)
        // The derivative of the law of cosines with respect to time (t) is:
        // 2c(dc / dt) = 0 + 0 + 2ab sin(C)(dC / dt)
        // (dc / dt) is lift target velocity
        // (dC / dt) is clockwiseTargetRotationVelocity
        // Solving for (dC / dt) and simplifying:
        // (dC / dt) = c(dc / dt) / (ab sin(C))
        double clockwiseTargetRotationVelocity =
                lift.getPosition() * lift.getTargetVelocity() /
                        (ARM_PIVOT_TO_LIFT_PIVOT_DISTANCE * ARM_PIVOT_TO_LIFT_JOINT_DISTANCE * Degrees.sin(getAngleFromLiftJointToLiftPivotAboutArmPivot())
        );

        double targetRotationVelocity = -clockwiseTargetRotationVelocity;

        // X-axis is along slide and Y-axis corresponds to rotation
        Vector2 targetVelocityRelativeToArmRotation = new Vector2(slide.getTargetVelocity(), Degrees.toRadians(targetRotationVelocity) * slide.getPosition());

        return targetVelocityRelativeToArmRotation.addRotation(getArmRotation());
    }

    // targetVelocity is in inches per second of front bottom lip of harvester
    // X coordinate is horizontal and relative to center of robot
    // Y coordinate is vertical and relative to ground
    // If targetVelocity is not obtainable, will attempt to set target velocity with the same direction as targetVelocity but with a smaller magnitude
    // (This does not hold true if rotation or slide position cannot move any farther)
    public void setTargetVelocity(Vector2 targetVelocity) {
        double targetRotationAmountVelocity = getTargetRotationAmountVelocity();

        /*
        if (getPosition().getX() > MAX_X_POSITION_BEHIND_ROBOT && getPosition().getX() < MIN_X_POSITION_IN_FRONT_OF_ROBOT && getPosition().getY() < MIN_Y_POSITION_ABOVE_ROBOT) {
            // BucketLocalizer is next to robot, so adjust targetVelocity so harvester will not hit robot
            if (getPosition().getX() < MAX_X_POSITION_BEHIND_ROBOT + SIDE_ACTION_WIDTH) {
                // BucketLocalizer is next to back of robot,
                // so make forward movement instead move harvester up to clear top of robot and disallow downward movement if forward movement is requested
                if (targetVelocity.getX() > 0.0) {
                    targetVelocity = new Vector2(0.0, targetVelocity.getX() + (targetVelocity.getY() > 0.0 ? targetVelocity.getY() : 0.0));
                }
            } else if (getPosition().getX() > MIN_X_POSITION_IN_FRONT_OF_ROBOT - SIDE_ACTION_WIDTH) {
                // BucketLocalizer is next to front of robot,
                // so make backward movement instead move harvester up to clear top of robot and disallow downward movement if backward movement is requested
                if (targetVelocity.getX() < 0.0) {
                    targetVelocity = new Vector2(0.0, -targetVelocity.getX() + (targetVelocity.getY() > 0.0 ? targetVelocity.getY() : 0.0));
                }
            } else if (getPosition().getX() < 0.0) {
                // BucketLocalizer is next to top of robot nearer the back side of the robot,
                // so make downward movement instead move harvester towards back side of the robot if forward movement is not requested
                if (targetVelocity.getY() < 0.0) {
                    targetVelocity = new Vector2(targetVelocity.getX() + (targetVelocity.getX() <= 0.0 ? targetVelocity.getY() : 0.0), 0.0);
                }
            }  else {
                // BucketLocalizer is next to top of robot nearer the front side of the robot,
                // so make downward movement instead move harvester towards front side of the robot if backward movement is not requested
                if (targetVelocity.getY() < 0.0) {
                    targetVelocity = new Vector2(targetVelocity.getX() + (targetVelocity.getX() >= 0.0 ? -targetVelocity.getY() : 0.0), 0.0);
                }
            }
        }
        */

        // X-axis is along slide and Y-axis corresponds to rotation
        Vector2 requestedVelocityRelativeArmRotation = targetVelocity.subRotation(getArmRotation());

        // Requested velocities are not used directly because if one of the motors lags behind the other, the harvester will not move in the requested direction
        // This could make the harvester jam into the ground instead of sliding along the ground when picking up minerals

        double requestedArmRotationVelocity = Math.abs(slide.getPosition()) >= MIN_SLIDE_POSITION_MAGNITUDE_FOR_NONZERO_TARGET_ARM_ROTATION_VELOCITY ?
                Degrees.fromRadians(requestedVelocityRelativeArmRotation.getY() / slide.getPosition()) : 0.0;
        double requestedSlideVelocity = requestedVelocityRelativeArmRotation.getX();

        double minAchievableArmRotationAcceleration = -MAX_ARM_ROTATION_ACCELERATION_FROM_STOP * (1.0 + (getArmRotationVelocity() / MAX_ARM_ROTATION_SPEED));
        double maxAchievableArmRotationAcceleration = MAX_ARM_ROTATION_ACCELERATION_FROM_STOP * (1.0 - (getArmRotationVelocity() / MAX_ARM_ROTATION_SPEED));
        double minAchievableSlideAcceleration = -MAX_SLIDE_ACCELERATION_FROM_STOP * (1.0 + (slide.getVelocity() / MAX_SLIDE_SPEED));
        double maxAchievableSlideAcceleration = MAX_SLIDE_ACCELERATION_FROM_STOP * (1.0 - (slide.getVelocity() / MAX_SLIDE_SPEED));

        double minAchievableArmRotationVelocity = getArmRotationVelocity() + (minAchievableArmRotationAcceleration * deltaTime);
        double maxAchievableArmRotationVelocity = getArmRotationVelocity() + (maxAchievableArmRotationAcceleration * deltaTime);
        double minAchievableSlideVelocity = slide.getVelocity() + (minAchievableSlideAcceleration * deltaTime);
        double maxAchievableSlideVelocity = slide.getVelocity() + (maxAchievableSlideAcceleration * deltaTime);

        double minAchievableFractionOfRequestedArmRotationVelocity = Math.min(
                minAchievableArmRotationVelocity / requestedArmRotationVelocity,
                maxAchievableArmRotationVelocity / requestedArmRotationVelocity
        );
        double maxAchievableFractionOfRequestedArmRotationVelocity = Math.max(
                minAchievableArmRotationVelocity / requestedArmRotationVelocity,
                maxAchievableArmRotationVelocity / requestedArmRotationVelocity
        );
        double minAchievableFractionOfRequestedSlideVelocity = Math.min(
                minAchievableSlideVelocity / requestedSlideVelocity,
                maxAchievableSlideVelocity / requestedSlideVelocity
        );
        double maxAchievableFractionOfRequestedSlideVelocity = Math.max(
                minAchievableSlideVelocity / requestedSlideVelocity,
                maxAchievableSlideVelocity / requestedSlideVelocity
        );

        // The highest min achievable fraction of requested velocity is what limits the overall min
        double minAchievableFractionOfRequestedVelocity = Math.max(minAchievableFractionOfRequestedArmRotationVelocity, minAchievableFractionOfRequestedSlideVelocity);

        // The lowest max achievable fraction of requested velocity is what limits the overall max
        double maxAchievableFractionOfRequestedVelocity = Math.min(maxAchievableFractionOfRequestedArmRotationVelocity, maxAchievableFractionOfRequestedSlideVelocity);

        // It is very possible for minAchievableFractionOfRequestedVelocity to be greater than maxAchievableFractionOfRequestedVelocity
        // In this case there is no achievable velocity where targetVelocity's direction is kept the same
        // Therefore, targetFractionOfRequestedVelocity will be forced to not be both more than minAchievableFractionOfRequestedVelocity and less than maxAchievableFractionOfRequestedVelocity
        // In whatever case, targetFractionOfRequestedVelocity will be set to whatever value is closest to the requested velocity (closest to 1.0) in the range
        // [minAchievableFractionOfRequestedVelocity, maxAchievableFractionOfRequestedVelocity] or [maxAchievableFractionOfRequestedVelocity, minAchievableFractionOfRequestedVelocity]
        double targetFractionOfRequestedVelocity;
        if (minAchievableFractionOfRequestedVelocity < 1.0 && maxAchievableFractionOfRequestedVelocity < 1.0) {
            targetFractionOfRequestedVelocity = Math.max(minAchievableFractionOfRequestedVelocity, maxAchievableFractionOfRequestedVelocity);
        } else if (minAchievableFractionOfRequestedVelocity > 1.0 && maxAchievableFractionOfRequestedVelocity > 1.0) {
            targetFractionOfRequestedVelocity = Math.min(minAchievableFractionOfRequestedVelocity, maxAchievableFractionOfRequestedVelocity);
        } else {
            targetFractionOfRequestedVelocity = 1.0;
        }

        slide.setTargetVelocity(requestedSlideVelocity * targetFractionOfRequestedVelocity);

        double targetArmRotationVelocity = requestedArmRotationVelocity * targetFractionOfRequestedVelocity;
        double clockwiseTargetArmRotationVelocity = -targetArmRotationVelocity;

        // Using Law of Cosines (c^2 = a^2 + b^2 - 2ab cos(C)) derivative, find lift target velocity from the known clockwise target arm rotation velocity
        // The the angle C is getAngleFromLiftJointToLiftPivotAboutArmPivot(),
        // the angle from LIFT_JOINT_TO_ARM_PIVOT_ROTATION to slide (this angle is positive and therefore goes clockwise)
        // c is the side of the triangle opposite C, making it the lift position
        // a and b are the other sides of the triangle, making them constants ARM_PIVOT_TO_LIFT_PIVOT_DISTANCE and ARM_PIVOT_TO_LIFT_JOINT_DISTANCE
        // (which one is a and which one is b does not make a difference)
        // The derivative of the law of cosines with respect to time (t) is:
        // 2c(dc / dt) = 0 + 0 + 2ab sin(C)(dC / dt)
        // (dc / dt) is the target lift velocity
        // (dC / dt) is clockwiseTargetArmRotationVelocity
        // Solving for (dc / dt) and simplifying:
        // (dc / dt) = ab sin(C)(dC / dt) / c
        lift.setTargetVelocity(
                ARM_PIVOT_TO_LIFT_PIVOT_DISTANCE * ARM_PIVOT_TO_LIFT_JOINT_DISTANCE * Degrees.sin(getAngleFromLiftJointToLiftPivotAboutArmPivot()) *
                        clockwiseTargetArmRotationVelocity / lift.getPosition()
        );

        setTargetRotationAmountVelocity(targetRotationAmountVelocity);
    }

    // In units (see getRotationAmount()) per second
    public double getTargetRotationAmountVelocity() {
        Vector2 rotationLineEyeletPositionRelativeToPulley =
                ROTATION_LINE_EYELET_POSITION_RELATIVE_TO_PULLEY_WHEN_SLIDE_POSITION_IS_ZERO.addX(slide.getPosition());

        // Using the derivative of a^2 + b^2 = c^2, find the tilt line target velocity from the known slide target velocity
        // 2a(da / dt) = 2c(dc / dt)
        // Solving for (dc / dt), which is the target tilt line velocity:
        // (dc / dt) = 2a(da / dt) / 2c
        // Simplifying:
        // (dc / dt) = a(da / dt) / c
        double targetVelocityOfRotationLineEyeletFromPulley =
                rotationLineEyeletPositionRelativeToPulley.getX() * slide.getTargetVelocity() / rotationLineEyeletPositionRelativeToPulley.getMagnitude();

        double targetVelocityOfRotationLineSpringFromEyelet = rotationLine.getTargetVelocity() - targetVelocityOfRotationLineEyeletFromPulley;

        return targetVelocityOfRotationLineSpringFromEyelet / (MIN_DISTANCE_BETWEEN_ROTATION_LINE_EYELET_AND_SPRING - MAX_DISTANCE_BETWEEN_ROTATION_LINE_EYELET_AND_SPRING);
    }

    public void setTargetRotationAmountVelocity(double targetRotationAmountVelocity) {
        Vector2 rotationLineEyeletPositionRelativeToPulley =
                ROTATION_LINE_EYELET_POSITION_RELATIVE_TO_PULLEY_WHEN_SLIDE_POSITION_IS_ZERO.addX(slide.getPosition());

        // Using the derivative of a^2 + b^2 = c^2, find the tilt line target velocity from the known slide target velocity
        // 2a(da / dt) = 2c(dc / dt)
        // Solving for (dc / dt), which is the target tilt line velocity:
        // (dc / dt) = 2a(da / dt) / 2c
        // Simplifying:
        // (dc / dt) = a(da / dt) / c
        double rotationLineTargetVelocity = rotationLineEyeletPositionRelativeToPulley.getX() * slide.getTargetVelocity() / rotationLineEyeletPositionRelativeToPulley.getMagnitude();

        rotationLineTargetVelocity += targetRotationAmountVelocity * (MIN_DISTANCE_BETWEEN_ROTATION_LINE_EYELET_AND_SPRING - MAX_DISTANCE_BETWEEN_ROTATION_LINE_EYELET_AND_SPRING);

        rotationLine.setTargetVelocity(rotationLineTargetVelocity);
    }

    // targetVelocity is in inches per second of front bottom lip of harvester
    // X coordinate is horizontal and relative to center of robot
    // Y coordinate is vertical and relative to ground
    // If targetVelocity is not obtainable, will attempt to set target velocity with the same direction as targetVelocity but with a smaller magnitude
    // (This does not hold true if rotation or slide position cannot move any farther)
    // targetRotationAmountVelocity is in units per second (see getRotationAmount())
    public void setTargetVelocityAndTargetRotationAmountVelocity(Vector2 targetVelocity, double targetRotationAmountVelocity) {
        setTargetVelocity(targetVelocity);
        setTargetRotationAmountVelocity(targetRotationAmountVelocity);
    }

    // In inches of bucket
    // X coordinate is horizontal and relative to center of robot
    // Y coordinate is vertical and relative to ground
    public Vector2 getPosition() {
        return ARM_PIVOT_POSITION.add(new Vector2(slide.getPosition(), -BUCKET_DISTANCE_BELOW_ARM_PIVOT).addRotation(getArmRotation()));
    }

    // In inches per second of bucket
    // X coordinate is horizontal and relative to center of robot
    // Y coordinate is vertical and relative to ground
    public Vector2 getVelocity() {
        return velocity;
    }

    // In inches per second per second bucket
    // X coordinate is horizontal and relative to center of robot
    // Y coordinate is vertical and relative to ground
    public Vector2 getAcceleration() {
        return acceleration;
    }

    // Should be around [0, 1] with 0 or lower indicating the bucket is fully lowered and 1 or higher indicating the bucket is fully raised
    public double getRotationAmount() {
        double distanceBetweenRotationLinePulleyAndEyelet =
                ROTATION_LINE_EYELET_POSITION_RELATIVE_TO_PULLEY_WHEN_SLIDE_POSITION_IS_ZERO.addX(slide.getPosition()).getMagnitude();

        double distanceBetweenRotationLineEyeletAndSpring = rotationLine.getPosition() - distanceBetweenRotationLinePulleyAndEyelet;

        return Range.scale(
                distanceBetweenRotationLineEyeletAndSpring,
                MIN_DISTANCE_BETWEEN_ROTATION_LINE_EYELET_AND_SPRING, MAX_DISTANCE_BETWEEN_ROTATION_LINE_EYELET_AND_SPRING,
                1.0, 0.0
        );
    }

    // In units (see getRotationAmount()) per second
    public double getRotationAmountVelocity() {
        return rotationAmountVelocity;
    }

    // In units (see getRotationAmount()) per second per second
    public double getRotationAmountAcceleration() {
        return rotationAmountAcceleration;
    }

    // In positive degrees
    private double getAngleFromLiftJointToLiftPivotAboutArmPivot() {
        // Using the Law of Cosines (c^2 = a^2 + b^2 - 2ab cos(C)), find angle using lift position
        // The unknown is the angle C, the return value,
        // the angle from LIFT_JOINT_TO_ARM_PIVOT_ROTATION to arm rotation (this angle is positive and therefore goes clockwise)
        // c is the side of the triangle opposite C, making it the lift position
        // a and b are the other sides of the triangle, making them constants ARM_PIVOT_TO_LIFT_PIVOT_DISTANCE and ARM_PIVOT_TO_LIFT_JOINT_DISTANCE
        // (which one is a and which one is b does not make a difference)
        // Solving the Law of Cosines for C:
        // C = acos((a^2 + b^2 - c^2) / (2ab))
        return Degrees.acos(
                (Math.pow(ARM_PIVOT_TO_LIFT_PIVOT_DISTANCE, 2) + Math.pow(ARM_PIVOT_TO_LIFT_JOINT_DISTANCE, 2) - Math.pow(lift.getPosition(), 2)) /
                        (2.0 * ARM_PIVOT_TO_LIFT_PIVOT_DISTANCE * ARM_PIVOT_TO_LIFT_JOINT_DISTANCE)
        );
    }

    // In degrees of arm around arm pivot starting with slides horizontal and moving counterclockwise towards back of robot
    public double getArmRotation() {
        // Since getAngleFromLiftJointToLiftPivotAboutArmPivot() always returns a positive angle, this angle goes clockwise, and therefore the negative is taken
        double rotationFromLiftJointToArmPivotRotation = -getAngleFromLiftJointToLiftPivotAboutArmPivot();
        double rotation = rotationFromLiftJointToArmPivotRotation + LIFT_JOINT_TO_ARM_PIVOT_ROTATION;

        // Adjust rotation to take into account sag of the slide
        rotation -= slide.getPosition() * ARM_ROTATION_SAG_PER_SLIDE_POSITION;

        return rotation;
    }

    // In degrees per second of arm around arm pivot with positive counterclockwise towards back of robot
    public double getArmRotationVelocity() {
        return armRotationVelocity;
    }

    // In degrees per second per second of arm around arm pivot with positive counterclockwise towards back of robot
    public double getArmRotationAcceleration() {
        return armRotationAcceleration;
    }

    // Called through Component.update()
    @Override
    void updateImpl() {
        // These values will change once lift.update() and slide.update() are called, so record them now so velocities and accelerations can be calculated
        Vector2 previousPosition = getPosition();
        double previousRotationAmount = getRotationAmount();
        double previousArmRotation = getArmRotation();

        lift.update();
        slide.update();
        rotationLine.update();

        // Calculate velocities and accelerations

        Vector2 previousVelocity = velocity;
        velocity = getPosition().sub(previousPosition).div(deltaTime);
        acceleration = velocity.sub(previousVelocity).div(deltaTime);

        double previousRotationAmountVelocity = rotationAmountVelocity;
        rotationAmountVelocity = (getRotationAmount() - previousRotationAmount) / deltaTime;
        rotationAmountAcceleration = (rotationAmountVelocity - previousRotationAmountVelocity) / deltaTime;

        double previousArmRotationVelocity = armRotationVelocity;
        armRotationVelocity = (getArmRotation() - previousArmRotation) / deltaTime;
        armRotationAcceleration = (armRotationVelocity - previousArmRotationVelocity) / deltaTime;
    }

    // Returns text describing state
    @Override
    public String toString() {
        return "position :       " + Inches.toString(getPosition()) + "\n" +
                "targetVelocity : " + Inches.toString(getTargetVelocity()) + "\n" +
                "velocity :       " + Inches.toString(getVelocity()) + "\n" +
                "acceleration :   " + Inches.toString(getAcceleration()) + "\n" +
                "rotationAmount :             " + String.format("%6.2f", getRotationAmount()) + "\n" +
                "rotationAmountVelocity :     " + String.format("%6.2f", getRotationAmountVelocity()) + "\n" +
                "rotationAmountAcceleration : " + String.format("%6.2f", getRotationAmountAcceleration()) + "\n" +
                "armRotation :             " + Degrees.toString(getArmRotation()) + "\n" +
                "armRotationVelocity :     " + Degrees.toString(getArmRotationVelocity()) + "\n" +
                "armRotationAcceleration : " + Degrees.toString(getArmRotationAcceleration());
    }

    // Returns text verbosely describing state
    @Override
    public String toStringVerbose() {
        return toString() + "\n" +
                "lift {\n" +
                lift.toStringVerbose() + "\n" +
                "}\n" +
                "slide {\n" +
                slide.toStringVerbose() + "\n" +
                "}\n" +
                "rotationLine {\n" +
                rotationLine.toStringVerbose() + "\n" +
                "}";
    }
}