package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.Degrees;
import org.firstinspires.ftc.teamcode.util.Inches;
import org.firstinspires.ftc.teamcode.util.Vector2;

// Controls reverseSlideLine system
public class Bucket extends Component {
    private static final String LIFT_MOTOR_NAME = "BucketLift";
    private static final String REVERSE_SLIDE_LINE_MOTOR_NAME = "BucketReverseSlideLine";
    private static final String RECLOCKING_SERVO_NAME = "BucketReclockingServo";

    private static final double LIFT_GEAR_RATIO = -3.0;
    private static final double LIFT_THREADS_PER_INCH = 16.0;

    // In inches from lift's universal joint to lift's upper pivot
    private static final double INITIAL_LIFT_POSITION = 8.5;
    private static final double MIN_LIFT_POSITION = 8.5;
    private static final double MAX_LIFT_POSITION = 11.5; //12.1;

    // In degrees per second
    private static final double MAX_ARM_ROTATION_SPEED = 16.0;

    // In degrees per second per second
    private static final double MAX_ARM_ROTATION_ACCELERATION_FROM_STOP = 20.0;

    // In degrees per inch starting below arm pivot
    private static final double ARM_ROTATION_SAG_PER_ARM_POSITION = 0.0378;

    // In inches from arm pivot to lift's upper pivot
    private static final double ARM_PIVOT_TO_LIFT_PIVOT_DISTANCE = 3.31;

    // In inches from arm pivot to lift's universal joint
    private static final double ARM_PIVOT_TO_LIFT_JOINT_DISTANCE = 10.72;

    // In degrees starting forward going counterclockwise to angle formed from lift's universal joint to arm pivot
    private static final double LIFT_JOINT_TO_ARM_PIVOT_ROTATION = 72.0; //74.0;

    // In inches from ground and center of robot to arm pivot with y-axis vertical
    private static final Vector2 ARM_PIVOT_POSITION = new Vector2(6.93, 16.9);

    // In inches from arm pivot to reverseSlideLine measured perpendicular to slide
    private static final double BUCKET_DISTANCE_BELOW_ARM_PIVOT = 8.75;

    // In inches of spool
    //private static final double FORWARD_SLIDE_PULLEY_DIAMETER = 3.1; //3.085; //3.2;
    private static final double SLIDE_PULLEY_DIAMETER = 3.3; //3.315; //3.2;

    // In inches of reverse slide line starting perpendicularly-to-slide below arm pivot and going forward parallel to slide
    private static final double INITIAL_ARM_POSITION = -3.3; //-3.0; //-2.55; //-3.7; //-4.25;
    private static final double MIN_ARM_POSITION = -64.5;
    private static final double MAX_ARM_POSITION = 46.5;
    private static final double MIN_ARM_POSITION_MAGNITUDE_FOR_NONZERO_TARGET_ARM_ROTATION_VELOCITY = 4.0;

    private static final double MAX_REVERSE_SLIDE_LINE_SLACK = 4.2;

    // In inches per second
    private static final double MAX_ARM_SPEED = 72.0;

    // In inches per second per second when velocity equals zero
    private static final double MAX_ARM_ACCELERATION_FROM_STOP = 288.0;

    private static final double RECLOCKING_SERVO_DISENGADED_POSITION = 0.25;
    private static final double RECLOCKING_SERVO_ENGADED_POSITION = 0.75;

    /*
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
    */

    private final Motor lift;
    private final Motor reverseSlideLine;
    private final Servo reclockingServo;

    // See getter methods

    private double slack = 0.0;
    private double slackVelocity = 0.0;
    private double slackAcceleration = 0.0;
    private double armVelocity = 0.0;
    private double armAcceleration = 0.0;
    private double armRotationVelocity = 0.0;
    private double armRotationAcceleration = 0.0;
    private Vector2 velocity = Vector2.ZERO;
    private Vector2 acceleration = Vector2.ZERO;

    public Bucket(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);

        lift = new Motor(
                telemetry, hardwareMap, LIFT_MOTOR_NAME, true, LIFT_GEAR_RATIO / 360.0 / LIFT_THREADS_PER_INCH,
                INITIAL_LIFT_POSITION, MIN_LIFT_POSITION, MAX_LIFT_POSITION
        );

        reverseSlideLine = new Motor(
                telemetry, hardwareMap, REVERSE_SLIDE_LINE_MOTOR_NAME, true, SLIDE_PULLEY_DIAMETER * Math.PI / 360.0,
                INITIAL_ARM_POSITION, MIN_ARM_POSITION, MAX_ARM_POSITION + MAX_REVERSE_SLIDE_LINE_SLACK
        );

        reclockingServo = hardwareMap.get(Servo.class, RECLOCKING_SERVO_NAME);
        reclockingServo.scaleRange(RECLOCKING_SERVO_DISENGADED_POSITION, RECLOCKING_SERVO_ENGADED_POSITION);
    }

    // Between [0, 1] with 0 indicating the slide lines are fully taunt and the bucket is fully raised
    // and 1 indicating the slide lines are fully slack and the bucket is completely loose
    public double getSlack() {
        return slack;
    }

    // In units (see getSlack()) per second
    public double getSlackVelocity() {
        return slackVelocity;
    }

    // In units (see getSlack()) per second per second
    public double getSlackAcceleration() {
        return slackAcceleration;
    }

    public double getArmPosition() {
        return reverseSlideLine.getPosition() - (slack * MAX_REVERSE_SLIDE_LINE_SLACK);
    }

    public boolean isArmAtMinPosition() {
        return getArmPosition() <= MIN_ARM_POSITION;
    }

    public boolean isArmAtMaxPosition() {
        return getArmPosition() >= MAX_ARM_POSITION;
    }

    public double getArmVelocity() {
        return armVelocity;
    }

    public double getArmAcceleration() {
        return armAcceleration;
    }

    // In positive degrees
    protected double getAngleFromLiftJointToLiftPivotAboutArmPivot() {
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
        rotation -= getArmPosition() * ARM_ROTATION_SAG_PER_ARM_POSITION;

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

    // In inches of reverseSlideLine
    // X coordinate is horizontal and relative to center of robot
    // Y coordinate is vertical and relative to ground
    public Vector2 getPosition() {
        return ARM_PIVOT_POSITION.add(new Vector2(getArmPosition(), -BUCKET_DISTANCE_BELOW_ARM_PIVOT).addRotation(getArmRotation()));
    }

    // In inches per second of reverseSlideLine
    // X coordinate is horizontal and relative to center of robot
    // Y coordinate is vertical and relative to ground
    public Vector2 getVelocity() {
        return velocity;
    }

    // In inches per second per second reverseSlideLine
    // X coordinate is horizontal and relative to center of robot
    // Y coordinate is vertical and relative to ground
    public Vector2 getAcceleration() {
        return acceleration;
    }

    public double getTargetArmVelocity() {
        if (reclockingServo.getPosition() == 0.0) {
            return reverseSlideLine.getTargetVelocity();
        } else {
            return 0.0;
        }
    }

    public void setTargetArmVelocity(double targetArmVelocity) {
        // Reclocking cannot happen at the same time as slide moves
        reclockingServo.setPosition(0.0);

        if ((isArmAtMinPosition() && targetArmVelocity < 0.0) || (isArmAtMaxPosition() && targetArmVelocity > 0.0)) {
            reverseSlideLine.setTargetVelocity(0.0);
        } else {
            reverseSlideLine.setTargetVelocity(targetArmVelocity);
        }
    }

    public double getTargetArmRotationVelocity() {
        // Using Law of Cosines (c^2 = a^2 + b^2 - 2ab cos(C)) derivative, find clockwise target arm rotation velocity from the known lift target velocity
        // The the angle C is getAngleFromLiftJointToLiftPivotAboutArmPivot(),
        // the angle from LIFT_JOINT_TO_ARM_PIVOT_ROTATION to slide (this angle is positive and therefore goes clockwise)
        // c is the side of the triangle opposite C, making it the lift position
        // a and b are the other sides of the triangle, making them constants ARM_PIVOT_TO_LIFT_PIVOT_DISTANCE and ARM_PIVOT_TO_LIFT_JOINT_DISTANCE
        // (which one is a and which one is b does not make a difference)
        // The derivative of the law of cosines with respect to time (t) is:
        // 2c(dc / dt) = 0 + 0 + 2ab sin(C)(dC / dt)
        // (dc / dt) is lift target velocity
        // (dC / dt) is clockwiseTargetArmRotationVelocity
        // Solving for (dC / dt) and simplifying:
        // (dC / dt) = c(dc / dt) / (ab sin(C))
        double clockwiseTargetArmRotationVelocity =
                lift.getPosition() * lift.getTargetVelocity() /
                        (ARM_PIVOT_TO_LIFT_PIVOT_DISTANCE * ARM_PIVOT_TO_LIFT_JOINT_DISTANCE * Degrees.sin(getAngleFromLiftJointToLiftPivotAboutArmPivot())
                        );

        return -clockwiseTargetArmRotationVelocity;
    }

    public void setTargetArmRotationVelocity(double targetArmRotationVelocity) {
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
    }

    public double getTargetSlackVelocity() {
        if (reclockingServo.getPosition() == 1.0) {
            return reverseSlideLine.getTargetVelocity() / MAX_REVERSE_SLIDE_LINE_SLACK;
        } else {
            return 0.0;
        }
    }

    public void setTargetSlackVelocity(double targetSlackVelocity) {
        // To change slack, reclocking has to happen
        reclockingServo.setPosition(1.0);

        reverseSlideLine.setTargetVelocity(targetSlackVelocity * MAX_REVERSE_SLIDE_LINE_SLACK);
    }

    // In inches per second of front bottom lip of harvester
    // X coordinate is horizontal and relative to center of robot
    // Y coordinate is vertical and relative to ground
    public Vector2 getTargetVelocity() {
        // X-axis is along slide and Y-axis corresponds to rotation
        Vector2 targetVelocityRelativeToArmRotation = new Vector2(getTargetArmVelocity(), Degrees.toRadians(getTargetArmRotationVelocity()) * getArmPosition());

        return targetVelocityRelativeToArmRotation.addRotation(getArmRotation());
    }

    // targetVelocity is in inches per second of front bottom lip of harvester
    // X coordinate is horizontal and relative to center of robot
    // Y coordinate is vertical and relative to ground
    // If targetVelocity is not obtainable, will attempt to set target velocity with the same direction as targetVelocity but with a smaller magnitude
    // (This does not hold true if rotation or slide position cannot move any farther)
    public void setTargetVelocity(Vector2 targetVelocity) {
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

        double requestedArmRotationVelocity = Math.abs(getArmPosition()) >= MIN_ARM_POSITION_MAGNITUDE_FOR_NONZERO_TARGET_ARM_ROTATION_VELOCITY ?
                Degrees.fromRadians(requestedVelocityRelativeArmRotation.getY() / getArmPosition()) : 0.0;
        double requestedArmVelocity = requestedVelocityRelativeArmRotation.getX();

        double minAchievableArmRotationAcceleration = -MAX_ARM_ROTATION_ACCELERATION_FROM_STOP * (1.0 + (getArmRotationVelocity() / MAX_ARM_ROTATION_SPEED));
        double maxAchievableArmRotationAcceleration = MAX_ARM_ROTATION_ACCELERATION_FROM_STOP * (1.0 - (getArmRotationVelocity() / MAX_ARM_ROTATION_SPEED));
        double minAchievableArmAcceleration = -MAX_ARM_ACCELERATION_FROM_STOP * (1.0 + (getArmVelocity() / MAX_ARM_SPEED));
        double maxAchievableArmAcceleration = MAX_ARM_ACCELERATION_FROM_STOP * (1.0 - (getArmVelocity() / MAX_ARM_SPEED));

        double minAchievableArmRotationVelocity = getArmRotationVelocity() + (minAchievableArmRotationAcceleration * deltaTime);
        double maxAchievableArmRotationVelocity = getArmRotationVelocity() + (maxAchievableArmRotationAcceleration * deltaTime);
        double minAchievableArmVelocity = getArmVelocity() + (minAchievableArmAcceleration * deltaTime);
        double maxAchievableArmVelocity = getArmVelocity() + (maxAchievableArmAcceleration * deltaTime);

        double minAchievableFractionOfRequestedArmRotationVelocity = Math.min(
                minAchievableArmRotationVelocity / requestedArmRotationVelocity,
                maxAchievableArmRotationVelocity / requestedArmRotationVelocity
        );
        double maxAchievableFractionOfRequestedArmRotationVelocity = Math.max(
                minAchievableArmRotationVelocity / requestedArmRotationVelocity,
                maxAchievableArmRotationVelocity / requestedArmRotationVelocity
        );
        double minAchievableFractionOfRequestedArmVelocity = Math.min(
                minAchievableArmVelocity / requestedArmVelocity,
                maxAchievableArmVelocity / requestedArmVelocity
        );
        double maxAchievableFractionOfRequestedArmVelocity = Math.max(
                minAchievableArmVelocity / requestedArmVelocity,
                maxAchievableArmVelocity / requestedArmVelocity
        );

        // The highest min achievable fraction of requested velocity is what limits the overall min
        double minAchievableFractionOfRequestedVelocity = Math.max(minAchievableFractionOfRequestedArmRotationVelocity, minAchievableFractionOfRequestedArmVelocity);

        // The lowest max achievable fraction of requested velocity is what limits the overall max
        double maxAchievableFractionOfRequestedVelocity = Math.min(maxAchievableFractionOfRequestedArmRotationVelocity, maxAchievableFractionOfRequestedArmVelocity);

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

        setTargetArmVelocity(requestedArmVelocity * targetFractionOfRequestedVelocity);
        setTargetArmRotationVelocity(requestedArmRotationVelocity * targetFractionOfRequestedVelocity);
    }

    // Called through Component.update()
    @Override
    void updateImpl() {
        // These values will change once lift.update() and reverseSlideLine.update() are called or new slack is calculated,
        // so record them now so new slack, velocities, and accelerations can be calculated
        double previousReverseSlideLinePosition = reverseSlideLine.getPosition();
        double previousSlack = getSlack();
        double previousArmPosition = getArmPosition();
        double previousArmRotation = getArmRotation();
        Vector2 previousPosition = getPosition();

        lift.update();
        reverseSlideLine.update();

        if (reclockingServo.getPosition() == 1.0) {
            slack += (reverseSlideLine.getPosition() - previousReverseSlideLinePosition) / MAX_REVERSE_SLIDE_LINE_SLACK;

            // Mechanical stops should ensure this is true
            slack = Range.clip(slack, 0.0, 1.0);
        }

        // Calculate velocities and accelerations
        double previousSlackVelocity = slackVelocity;
        slackVelocity = (getSlack() - previousSlack) / deltaTime;
        slackAcceleration = (slackVelocity - previousSlackVelocity) / deltaTime;

        double previousArmRotationVelocity = armRotationVelocity;
        armRotationVelocity = (getArmRotation() - previousArmRotation) / deltaTime;
        armRotationAcceleration = (armRotationVelocity - previousArmRotationVelocity) / deltaTime;

        Vector2 previousVelocity = velocity;
        velocity = getPosition().sub(previousPosition).div(deltaTime);
        acceleration = velocity.sub(previousVelocity).div(deltaTime);
    }

    // Returns text describing state
    @Override
    public String toString() {
        return "slack :                " + String.format("%6.2f", getSlack()) + "\n" +
                "targetSlackVelocity : " + String.format("%6.2f", getTargetSlackVelocity()) + "\n" +
                "slackVelocity :       " + String.format("%6.2f", getSlackVelocity()) + "\n" +
                "slackAcceleration :   " + String.format("%6.2f", getSlackAcceleration()) + "\n" +
                "armPosition :       " + Inches.toString(getArmPosition()) + "\n" +
                "armAtMinPosition :  " + Boolean.toString(isArmAtMinPosition()) + "\n" +
                "armAtMaxPosition :  " + Boolean.toString(isArmAtMaxPosition()) + "\n" +
                "targetArmVelocity : " + Inches.toString(getTargetArmVelocity()) + "\n" +
                "armVelocity :       " + Inches.toString(getArmVelocity()) + "\n" +
                "armAcceleration :   " + Inches.toString(getArmAcceleration()) + "\n" +
                "armRotation :               " + Degrees.toString(getArmRotation()) + "\n" +
                "targetArmRotationVelocity : " + Degrees.toString(getTargetArmRotationVelocity()) + "\n" +
                "armRotationVelocity :       " + Degrees.toString(getArmRotationVelocity()) + "\n" +
                "armRotationAcceleration :   " + Degrees.toString(getArmRotationAcceleration()) + "\n" +
                "position :       " + Inches.toString(getPosition()) + "\n" +
                "targetVelocity : " + Inches.toString(getTargetVelocity()) + "\n" +
                "velocity :       " + Inches.toString(getVelocity()) + "\n" +
                "acceleration :   " + Inches.toString(getAcceleration());
    }

    // Returns text verbosely describing state
    @Override
    public String toStringVerbose() {
        return toString() + "\n" +
                "lift {\n" +
                lift.toStringVerbose() + "\n" +
                "}\n" +
                "reverseSlideLine {\n" +
                reverseSlideLine.toStringVerbose() + "\n" +
                "}\n" +
                "reclockingServo {\n" +
                "position : " + String.format("%4.2", reclockingServo.getPosition()) + "\n" +
                "}";
    }
}