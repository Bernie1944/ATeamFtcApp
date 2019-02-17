package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.Degrees;
import org.firstinspires.ftc.teamcode.util.Inches;
import org.firstinspires.ftc.teamcode.util.Vector2;

// Controls a 4-wheel omnidirectional-drive system where the wheels are mounted at 45 degree angles on the corners of the robot
public class Drive extends Component {
    private static final String FL_WHEEL_MOTOR_NAME = "DriveFlWheel";
    private static final String FR_WHEEL_MOTOR_NAME = "DriveFrWheel";
    private static final String BL_WHEEL_MOTOR_NAME = "DriveBlWheel";
    private static final String BR_WHEEL_MOTOR_NAME = "DriveBrWheel";

    // Distance between center of wheel base and the wheels in inches
    private static final double WHEEL_BASE_RADIUS = 8.25;

    // Effective diameter of front left, front right, back left, and back right wheels in inches
    private static final double FL_WHEEL_DIAMETER = 3.8;
    private static final double FR_WHEEL_DIAMETER = 3.8;
    private static final double BL_WHEEL_DIAMETER = 3.8;
    private static final double BR_WHEEL_DIAMETER = 3.8;

    // In inches per second
    // This value along with WHEEL_MAX_ACHIEVABLE_ACCELERATION_FROM_STOP are used in setTarget() to limit target velocities for each of the wheels to obtainable levels
    // while keeping these target wheel velocities proportional to each other,
    // thus keeping the robot moving in the same direction as requested and with the same proportion of velocity to angular velocity
    // This should be set as high as possible without noticeable decreases in handling
    private static final double WHEEL_MAX_ACHIEVABLE_SPEED = 48.0;

    // In inches per second per second
    // See WHEEL_MAX_ACHIEVABLE_SPEED
    // The max acceleration of the wheel when velocity is zero if the wheel is not allowed to slip
    // This should be set as high as possible without noticeable decreases in handling
    private static final double WHEEL_MAX_ACHIEVABLE_ACCELERATION_FROM_STOP = 100.0;

    // Used as a fudge factor compensating for amount wheels slip if the robot is driving along one of the drive wheel axises
    // 0.0 = no slippage; 1.0 = complete slippage
    private static final double VELOCITY_ALONG_AXIS_WHEEL_SLIP_FRACTION = 0.1;

    private final Motor flWheel;
    private final Motor frWheel;
    private final Motor blWheel;
    private final Motor brWheel;

    public Drive(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);

        flWheel = new Motor(telemetry, hardwareMap, FL_WHEEL_MOTOR_NAME, FL_WHEEL_DIAMETER * Math.PI, 0.0);
        frWheel = new Motor(telemetry, hardwareMap, FR_WHEEL_MOTOR_NAME, FR_WHEEL_DIAMETER * Math.PI, 0.0);
        blWheel = new Motor(telemetry, hardwareMap, BL_WHEEL_MOTOR_NAME, BL_WHEEL_DIAMETER * Math.PI, 0.0);
        brWheel = new Motor(telemetry, hardwareMap, BR_WHEEL_MOTOR_NAME, BR_WHEEL_DIAMETER * Math.PI, 0.0);
    }

    // In inches per second relative to the drive's rotation
    public Vector2 getVelocity() {
        // In inches per second
        // X-axis is velocity along direction of fr and bl wheels
        // Y-axis is velocity along direction of fl and br wheels
        Vector2 velocityOfWheelAxises = new Vector2(
                (frWheel.getVelocity() - blWheel.getVelocity()) / 2.0,
                (flWheel.getVelocity() - brWheel.getVelocity()) / 2.0
        );

        // If the robot is to drive along one of it's axises,
        // decrease targetVelocityOfDiagonals by taking into account VELOCITY_ALONG_AXIS_WHEEL_SLIP_FRACTION to compensate for wheel slippage
        velocityOfWheelAxises = velocityOfWheelAxises.mul(calculateWheelVelocityTransferFraction(velocityOfWheelAxises.getRotation()));

        // Rotate velocity to be relative to robot's axises
        return velocityOfWheelAxises.addRotation(45);
    }

    // In degrees per second with positive counterclockwise
    public double getAngularVelocity() {
        // The average linear velocities of the drive motors is what determines the drive's angular velocity
        double tangentialVelocityOfWheels = (flWheel.getVelocity() + frWheel.getVelocity() + blWheel.getVelocity() + brWheel.getVelocity()) / 4.0;

        return Degrees.fromRadians(tangentialVelocityOfWheels / WHEEL_BASE_RADIUS);
    }

    // 0.0 = complete slippage; 1.0 = no slippage
    protected double calculateWheelVelocityTransferFraction(double velocityRotationRelativeToWheelAxises) {
        // 0.0 = velocities's direction is the full 45 degrees off of the robots axises
        // 1.0 = velocities is directly in line with one of the robots axises (forward, backward, left, or right)
        double velocityAlongWheelAxisesFraction = 0.5 * Degrees.cos(velocityRotationRelativeToWheelAxises * 4.0) + 0.5;

        // Return wheel velocity transfer fraction fudge factor
        return 1.0 - (velocityAlongWheelAxisesFraction * VELOCITY_ALONG_AXIS_WHEEL_SLIP_FRACTION);
    }

    public boolean arePowersSet() {
        return flWheel.isPowerSet() && frWheel.isPowerSet() && blWheel.isPowerSet() && brWheel.isPowerSet();
    }

    public void setPowers(Vector2 power, double angularPower) {
        // Magnitude between [0, 1]
        // X-axis is velocity along direction of fl and br wheels
        // Y-axis is velocity along direction of fr and bl wheels
        Vector2 powerAlongWheelAxises = power.subRotation(45.0);

        Vector2 maxPowerAlongWheelAxis = powerAlongWheelAxises.withNormalizedMagnitude();

        flWheel.setPower(angularPower + (powerAlongWheelAxises.getY() / Math.abs(maxPowerAlongWheelAxis.getY())));
        frWheel.setPower(angularPower + (powerAlongWheelAxises.getX() / Math.abs(maxPowerAlongWheelAxis.getX())));
        blWheel.setPower(angularPower - (powerAlongWheelAxises.getX() / Math.abs(maxPowerAlongWheelAxis.getX())));
        brWheel.setPower(angularPower - (powerAlongWheelAxises.getY() / Math.abs(maxPowerAlongWheelAxis.getY())));
    }

    // True if drive has set target velocity and angular velocity and is not running to target position and rotation
    public boolean areTargetVelocitiesSet() {
        return flWheel.isTargetVelocitySet() && frWheel.isTargetVelocitySet() && blWheel.isTargetVelocitySet() && brWheel.isTargetVelocitySet();
    }

    // In inches per second relative to the drive's rotation
    public Vector2 getTargetVelocity() {
        // In inches per second
        // X-axis is velocity along direction of fl and br wheels
        // Y-axis is velocity along direction of fr and bl wheels
        Vector2 targetVelocityOfWheelAxises = new Vector2(
                (frWheel.getTargetVelocity() - blWheel.getTargetVelocity()) / 2.0,
                (flWheel.getTargetVelocity() - brWheel.getTargetVelocity()) / 2.0
        );

        // If the robot is to drive along one of it's axises,
        // decrease targetVelocityOfWheelAxises by taking into account VELOCITY_ALONG_AXIS_WHEEL_SLIP_FRACTION to compensate for wheel slippage
        targetVelocityOfWheelAxises = targetVelocityOfWheelAxises.mul(calculateWheelVelocityTransferFraction(targetVelocityOfWheelAxises.getRotation()));

        // Rotate velocity to be relative to robot's axises
        return targetVelocityOfWheelAxises.addRotation(45);
    }

    // In degrees per second with positive counterclockwise
    public double getTargetAngularVelocity() {
        // The average linear velocities of the drive motors is what determines the drive's angular velocity
        double targetTangentialVelocityOfWheels = (flWheel.getTargetVelocity() + frWheel.getTargetVelocity() + blWheel.getTargetVelocity() + brWheel.getTargetVelocity()) / 4.0;

        return Degrees.fromRadians(targetTangentialVelocityOfWheels / WHEEL_BASE_RADIUS);
    }

    // targetVelocity is in inches per second relative to the drive's rotation
    // targetAngularVelocity is in degrees per second with positive counterclockwise
    // If targetVelocity and targetAngularVelocity are not obtainable, both of these values will be scaled proportionally and targetVelocity's direction will be kept the same
    public void setTargetVelocities(Vector2 targetVelocity, double targetAngularVelocity) {
        // In inches per second
        // X-axis is velocity along direction of fl and br wheels
        // Y-axis is velocity along direction of fr and bl wheels
        Vector2 requestedVelocityOfWheelAxises = targetVelocity.subRotation(45.0);

        // If the robot is to drive along one of it's axises, increase requestedVelocityOfWheelAxises by taking into account VELOCITY_ALONG_AXIS_WHEEL_SLIP_FRACTION to compensate for wheel slippage
        requestedVelocityOfWheelAxises = requestedVelocityOfWheelAxises.div(calculateWheelVelocityTransferFraction(requestedVelocityOfWheelAxises.getRotation()));

        // The average linear velocities of the drive motors is what determines the drive's angular velocity
        double requestedTangentialVelocityOfWheels = Degrees.toRadians(targetAngularVelocity) * WHEEL_BASE_RADIUS;

        // The wheel target velocities are not set to the requested velocities directly because it can make the controls feel unresponsive
        // For example, when driving at high velocities, the robot may not want to rotate because the motors are already at full capacity trying to attain high velocities
        // Also, fast accelerations may rotate the robot because some of the wheels may not be able to keep up
        double flWheelRequestedVelocity = requestedTangentialVelocityOfWheels + requestedVelocityOfWheelAxises.getY();
        double frWheelRequestedVelocity = requestedTangentialVelocityOfWheels + requestedVelocityOfWheelAxises.getX();
        double blWheelRequestedVelocity = requestedTangentialVelocityOfWheels - requestedVelocityOfWheelAxises.getX();
        double brWheelRequestedVelocity = requestedTangentialVelocityOfWheels - requestedVelocityOfWheelAxises.getY();

        double flWheelMinAchievableAcceleration = -WHEEL_MAX_ACHIEVABLE_ACCELERATION_FROM_STOP * (1.0 + (flWheel.getVelocity() / WHEEL_MAX_ACHIEVABLE_SPEED));
        double frWheelMinAchievableAcceleration = -WHEEL_MAX_ACHIEVABLE_ACCELERATION_FROM_STOP * (1.0 + (flWheel.getVelocity() / WHEEL_MAX_ACHIEVABLE_SPEED));
        double blWheelMinAchievableAcceleration = -WHEEL_MAX_ACHIEVABLE_ACCELERATION_FROM_STOP * (1.0 + (flWheel.getVelocity() / WHEEL_MAX_ACHIEVABLE_SPEED));
        double brWheelMinAchievableAcceleration = -WHEEL_MAX_ACHIEVABLE_ACCELERATION_FROM_STOP * (1.0 + (flWheel.getVelocity() / WHEEL_MAX_ACHIEVABLE_SPEED));

        double flWheelMaxAchievableAcceleration = WHEEL_MAX_ACHIEVABLE_ACCELERATION_FROM_STOP * (1.0 - (flWheel.getVelocity() / WHEEL_MAX_ACHIEVABLE_SPEED));
        double frWheelMaxAchievableAcceleration = WHEEL_MAX_ACHIEVABLE_ACCELERATION_FROM_STOP * (1.0 - (frWheel.getVelocity() / WHEEL_MAX_ACHIEVABLE_SPEED));
        double blWheelMaxAchievableAcceleration = WHEEL_MAX_ACHIEVABLE_ACCELERATION_FROM_STOP * (1.0 - (blWheel.getVelocity() / WHEEL_MAX_ACHIEVABLE_SPEED));
        double brWheelMaxAchievableAcceleration = WHEEL_MAX_ACHIEVABLE_ACCELERATION_FROM_STOP * (1.0 - (brWheel.getVelocity() / WHEEL_MAX_ACHIEVABLE_SPEED));

        double flWheelMinAchievableVelocity = flWheel.getVelocity() + (flWheelMinAchievableAcceleration * deltaTime);
        double frWheelMinAchievableVelocity = frWheel.getVelocity() + (frWheelMinAchievableAcceleration * deltaTime);
        double blWheelMinAchievableVelocity = blWheel.getVelocity() + (blWheelMinAchievableAcceleration * deltaTime);
        double brWheelMinAchievableVelocity = brWheel.getVelocity() + (brWheelMinAchievableAcceleration * deltaTime);

        double flWheelMaxAchievableVelocity = flWheel.getVelocity() + (flWheelMaxAchievableAcceleration * deltaTime);
        double frWheelMaxAchievableVelocity = frWheel.getVelocity() + (frWheelMaxAchievableAcceleration * deltaTime);
        double blWheelMaxAchievableVelocity = blWheel.getVelocity() + (blWheelMaxAchievableAcceleration * deltaTime);
        double brWheelMaxAchievableVelocity = brWheel.getVelocity() + (brWheelMaxAchievableAcceleration * deltaTime);

        double flWheelMinAchievableFractionOfRequestedVelocity =
                Math.min(flWheelMinAchievableVelocity / flWheelRequestedVelocity, flWheelMaxAchievableVelocity / flWheelRequestedVelocity);
        double frWheelMinAchievableFractionOfRequestedVelocity =
                Math.min(frWheelMinAchievableVelocity / frWheelRequestedVelocity, frWheelMaxAchievableVelocity / frWheelRequestedVelocity);
        double blWheelMinAchievableFractionOfRequestedVelocity =
                Math.min(blWheelMinAchievableVelocity / blWheelRequestedVelocity, blWheelMaxAchievableVelocity / blWheelRequestedVelocity);
        double brWheelMinAchievableFractionOfRequestedVelocity =
                Math.min(brWheelMinAchievableVelocity / brWheelRequestedVelocity, brWheelMaxAchievableVelocity / brWheelRequestedVelocity);

        double flWheelMaxAchievableFractionOfRequestedVelocity =
                Math.max(flWheelMinAchievableVelocity / flWheelRequestedVelocity, flWheelMaxAchievableVelocity / flWheelRequestedVelocity);
        double frWheelMaxAchievableFractionOfRequestedVelocity =
                Math.max(frWheelMinAchievableVelocity / frWheelRequestedVelocity, frWheelMaxAchievableVelocity / frWheelRequestedVelocity);
        double blWheelMaxAchievableFractionOfRequestedVelocity =
                Math.max(blWheelMinAchievableVelocity / blWheelRequestedVelocity, blWheelMaxAchievableVelocity / blWheelRequestedVelocity);
        double brWheelMaxAchievableFractionOfRequestedVelocity =
                Math.max(brWheelMinAchievableVelocity / brWheelRequestedVelocity, brWheelMaxAchievableVelocity / brWheelRequestedVelocity);

        // The highest min achievable fraction of requested velocity is what limits the overall min
        double minAchievableFractionOfRequestedVelocity = Math.max(
                Math.max(flWheelMinAchievableFractionOfRequestedVelocity, frWheelMinAchievableFractionOfRequestedVelocity),
                Math.max(blWheelMinAchievableFractionOfRequestedVelocity, brWheelMinAchievableFractionOfRequestedVelocity)
        );

        // The lowest max achievable fraction of requested velocity is what limits the overall max
        double maxAchievableFractionOfRequestedVelocity = Math.min(
                Math.min(flWheelMaxAchievableFractionOfRequestedVelocity, frWheelMaxAchievableFractionOfRequestedVelocity),
                Math.min(blWheelMaxAchievableFractionOfRequestedVelocity, brWheelMaxAchievableFractionOfRequestedVelocity)
        );

        // It is very possible for minAchievableFractionOfRequestedVelocity to be greater than maxAchievableFractionOfRequestedVelocity
        // In this case there is no achievable velocity where targetVelocity and targetAngularVelocity are kept proportional and targetVelocity's direction is kept the same
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

        flWheel.setTargetVelocity(flWheelRequestedVelocity * targetFractionOfRequestedVelocity);
        frWheel.setTargetVelocity(frWheelRequestedVelocity * targetFractionOfRequestedVelocity);
        blWheel.setTargetVelocity(blWheelRequestedVelocity * targetFractionOfRequestedVelocity);
        brWheel.setTargetVelocity(brWheelRequestedVelocity * targetFractionOfRequestedVelocity);
    }

    // True if drive has set target position and rotation and is not running with target velocity and angular velocity
    public boolean isTargetRelativeOrientationSet() {
        return flWheel.isTargetPositionSet() && frWheel.isTargetPositionSet() && blWheel.isTargetPositionSet() && brWheel.isTargetPositionSet();
    }

    // In inches
    public Vector2 getTargetRelativePosition() {
        double flWheelTargetRelativePosition = flWheel.getTargetPosition() - flWheel.getPosition();
        double frWheelTargetRelativePosition = frWheel.getTargetPosition() - frWheel.getPosition();
        double blWheelTargetRelativePosition = blWheel.getTargetPosition() - blWheel.getPosition();
        double brWheelTargetRelativePosition = brWheel.getTargetPosition() - brWheel.getPosition();

        // In inches
        // X-axis is velocity along direction of fl and br wheels
        // Y-axis is velocity along direction of fr and bl wheels
        Vector2 targetRelativePositionAlongWheelAxises = new Vector2(
                (frWheelTargetRelativePosition - blWheelTargetRelativePosition) / 2.0,
                (flWheelTargetRelativePosition - brWheelTargetRelativePosition) / 2.0
        );

        // If the robot is to drive along one of it's axises,
        // decrease targetRelativePositionAlongWheelAxises by taking into account VELOCITY_ALONG_AXIS_WHEEL_SLIP_FRACTION to compensate for wheel slippage
        targetRelativePositionAlongWheelAxises =
                targetRelativePositionAlongWheelAxises.mul(calculateWheelVelocityTransferFraction(targetRelativePositionAlongWheelAxises.getRotation()));

        // Rotate velocity to be relative to robot's axises
        return targetRelativePositionAlongWheelAxises.addRotation(45);
    }

    // In degrees with positive counterclockwise
    public double getTargetRelativeRotation() {
        double flWheelTargetRelativePosition = flWheel.getTargetPosition() - flWheel.getPosition();
        double frWheelTargetRelativePosition = frWheel.getTargetPosition() - frWheel.getPosition();
        double blWheelTargetRelativePosition = blWheel.getTargetPosition() - blWheel.getPosition();
        double brWheelTargetRelativePosition = brWheel.getTargetPosition() - brWheel.getPosition();

        // The average positions of the drive motors is what determines the drive's rotation
        double targetRelativeTangentialPositionOfWheels =
                (flWheelTargetRelativePosition + frWheelTargetRelativePosition + blWheelTargetRelativePosition + brWheelTargetRelativePosition) / 4.0;

        return Degrees.fromRadians(targetRelativeTangentialPositionOfWheels / WHEEL_BASE_RADIUS);
    }

    // targetRelativePosition is relative to the robot's rotation, meaning if targetRelativeRotation is nonzero, robot will sweep an arc
    // (at any point in time moving towards targetRelativePosition relative to robot's rotation)
    // with arc length equal to targetRelativePosition's magnitude
    // targetRelativeRotation does not loop over, meaning it can be outside a range of 360 degrees
    public void setTargetRelativeOrientation(Vector2 targetRelativePosition, double targetRelativeRotation, double maxVelocity, double maxAngularVelocity) {
        // If maxVelocity or maxAngularVelocity are extremely small, big, or infinity, unexpected results could occur in the math
        // so constrain them between a small or unachievably high value
        maxVelocity = Range.clip(maxVelocity, Double.MIN_NORMAL, 1000.0);
        maxAngularVelocity = Range.clip(maxAngularVelocity, Double.MIN_NORMAL, 1000.0);

        double timeToGetToTargetRelativePositionAtMaxVelocity = targetRelativePosition.getMagnitude() / maxVelocity;
        double timeToGetToTargetRelativeRotationAtMaxAngularVelocity = Math.abs(targetRelativeRotation) / maxAngularVelocity;
        double timeToGetToTargetOrientationAtMaxVelocities = Math.max(timeToGetToTargetRelativePositionAtMaxVelocity, timeToGetToTargetRelativeRotationAtMaxAngularVelocity);

        setTargetVelocities(
                targetRelativePosition.withMagnitude(
                        maxVelocity * (timeToGetToTargetRelativePositionAtMaxVelocity / timeToGetToTargetOrientationAtMaxVelocities)
                ),
                (targetRelativeRotation < 0.0 ? -maxAngularVelocity : maxAngularVelocity) *
                        (timeToGetToTargetRelativeRotationAtMaxAngularVelocity / timeToGetToTargetOrientationAtMaxVelocities)
        );

        /*
        // In inches
        // X-axis is velocity along direction of fl and br wheels
        // Y-axis is velocity along direction of fr and bl wheels
        Vector2 targetRelativePositionAlongWheelAxises = targetRelativePosition.subRotation(45.0);

        // If the robot is to drive along one of it's axises,
        // increase targetRelativePositionAlongWheelAxises by taking into account VELOCITY_ALONG_AXIS_WHEEL_SLIP_FRACTION to compensate for wheel slippage
        targetRelativePositionAlongWheelAxises =
                targetRelativePositionAlongWheelAxises.div(calculateWheelVelocityTransferFraction(targetRelativePositionAlongWheelAxises.getRotation()));

        // The average positions of the drive motors is what determines the drive's rotation
        double targetRelativeTangentialPositionOfWheels = Degrees.toRadians(targetRelativeRotation) * WHEEL_BASE_RADIUS;

        double flWheelTargetRelativePosition = targetRelativeTangentialPositionOfWheels + targetRelativePositionAlongWheelAxises.getY();
        double frWheelTargetRelativePosition = targetRelativeTangentialPositionOfWheels + targetRelativePositionAlongWheelAxises.getX();
        double blWheelTargetRelativePosition = targetRelativeTangentialPositionOfWheels - targetRelativePositionAlongWheelAxises.getX();
        double brWheelTargetRelativePosition = targetRelativeTangentialPositionOfWheels - targetRelativePositionAlongWheelAxises.getY();

        flWheel.setTargetPosition(flWheel.getPosition() + flWheelTargetRelativePosition, Math.abs(flWheel.getTargetVelocity()));
        frWheel.setTargetPosition(frWheel.getPosition() + frWheelTargetRelativePosition, Math.abs(frWheel.getTargetVelocity()));
        blWheel.setTargetPosition(blWheel.getPosition() + blWheelTargetRelativePosition, Math.abs(blWheel.getTargetVelocity()));
        brWheel.setTargetPosition(brWheel.getPosition() + brWheelTargetRelativePosition, Math.abs(brWheel.getTargetVelocity()));
        */
    }

    // targetRelativePosition is relative to the robot's rotation, meaning if targetRelativeRotation is nonzero, robot will sweep an arc
    // (at any point in time moving towards targetRelativePosition relative to robot's rotation)
    // with arc length equal to targetRelativePosition's magnitude
    // targetRelativeRotation does not loop over, meaning it can be outside a range of 360 degrees
    // Max velocities are infinity
    public void setTargetRelativeOrientation(Vector2 targetRelativePosition, double targetRelativeRotation) {
        setTargetRelativeOrientation(targetRelativePosition, targetRelativeRotation, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    // Called through Component.update()
    @Override
    void updateImpl() {
        flWheel.update();
        frWheel.update();
        blWheel.update();
        brWheel.update();
    }

    // Returns text describing state
    @Override
    public String toString() {
        return "targetRelativeOrientationSet : " + Boolean.toString(isTargetRelativeOrientationSet()) + "\n" +
                "targetRelativePosition : " + Inches.toString(getTargetRelativePosition()) + "\n" +
                "targetRelativeRotation : " + Inches.toString(getTargetRelativeRotation()) + "\n" +
                "targetVelocitiesSet : " + Boolean.toString(areTargetVelocitiesSet()) + "\n" +
                "targetVelocity : " + Inches.toString(getTargetVelocity()) + "\n" +
                "velocity :       " + Inches.toString(getVelocity()) + "\n" +
                "targetAngularVelocity : " + Degrees.toString(getTargetAngularVelocity()) + "\n" +
                "angularVelocity :       " + Degrees.toString(getAngularVelocity());
    }

    // Returns text verbosely describing state
    @Override
    public String toStringVerbose() {
        return toString() + "\n" +
                "flWheel {\n" +
                flWheel.toStringVerbose() + "\n" +
                "}\n" +
                "frWheel {\n" +
                frWheel.toStringVerbose() + "\n" +
                "}\n" +
                "blWheel {\n" +
                blWheel.toStringVerbose() + "\n" +
                "}\n" +
                "brWheel {\n" +
                brWheel.toStringVerbose() + "\n" +
                "}";
    }
}