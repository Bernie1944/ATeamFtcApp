package org.firstinspires.ftc.teamcode.components;

import java.util.Locale;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    // thus keeping the robot moving in the same direction as requested and with the same proportion of velocity to rotation velocity
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

    // See getter methods
    private Vector2 acceleration = Vector2.ZERO;
    private double rotationAcceleration = 0.0;

    public Drive(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);

        flWheel = new Motor(telemetry, hardwareMap, FL_WHEEL_MOTOR_NAME, true, FL_WHEEL_DIAMETER * Math.PI / 360.0);
        frWheel = new Motor(telemetry, hardwareMap, FR_WHEEL_MOTOR_NAME, true, FR_WHEEL_DIAMETER * Math.PI / 360.0);
        blWheel = new Motor(telemetry, hardwareMap, BL_WHEEL_MOTOR_NAME, true, BL_WHEEL_DIAMETER * Math.PI / 360.0);
        brWheel = new Motor(telemetry, hardwareMap, BR_WHEEL_MOTOR_NAME, true, BR_WHEEL_DIAMETER * Math.PI / 360.0);
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

        // If the robot is to drive along one of it's axises, decrease targetVelocityOfWheelAxises by taking into account VELOCITY_ALONG_AXIS_WHEEL_SLIP_FRACTION to compensate for wheel slippage
        targetVelocityOfWheelAxises = targetVelocityOfWheelAxises.mul(calculateWheelVelocityTransferFraction(targetVelocityOfWheelAxises.getRotation()));

        // Rotate velocity to be relative to robot's axises
        return targetVelocityOfWheelAxises.addRotation(45);
    }

    // In degrees per second with positive counterclockwise
    public double getTargetRotationVelocity() {
        // The average linear velocities of the drive motors is what determines the drive's rotation velocity
        double linearVelocityOfRotation = (flWheel.getTargetVelocity() + frWheel.getTargetVelocity() + blWheel.getTargetVelocity() + brWheel.getTargetVelocity()) / 4.0;

        return Degrees.fromRadians(linearVelocityOfRotation / WHEEL_BASE_RADIUS);
    }

    // targetVelocity is in inches per second relative to the drive's rotation
    // targetRotationVelocity is in degrees per second with positive counterclockwise
    // If targetVelocity and targetRotationVelocity are not obtainable, both of these values will be scaled proportionally and targetVelocity's direction will be kept the same
    public void setTargetVelocityAndTargetRotationVelocity(Vector2 targetVelocity, double targetRotationVelocity) {
        // In inches per second
        // X-axis is velocity along direction of fl and br wheels
        // Y-axis is velocity along direction of fr and bl wheels
        Vector2 requestedVelocityOfWheelAxises = targetVelocity.subRotation(45.0);

        // If the robot is to drive along one of it's axises, increase requestedVelocityOfWheelAxises by taking into account VELOCITY_ALONG_AXIS_WHEEL_SLIP_FRACTION to compensate for wheel slippage
        requestedVelocityOfWheelAxises = requestedVelocityOfWheelAxises.div(calculateWheelVelocityTransferFraction(requestedVelocityOfWheelAxises.getRotation()));

        // The average linear velocities of the drive motors is what determines the drive's rotation velocity
        double requestedLinearVelocityOfRotation = Degrees.toRadians(targetRotationVelocity) * WHEEL_BASE_RADIUS;

        // The wheel target velocities are not set to the requested velocities directly because it can make the controls feel unresponsive
        // For example, when driving at high velocities, the robot may not want to rotate because the motors are already at full capacity trying to attain high velocities
        // Also, fast accelerations may rotate the robot because some of the wheels may not be able to keep up
        double flWheelRequestedVelocity = requestedLinearVelocityOfRotation + requestedVelocityOfWheelAxises.getY();
        double frWheelRequestedVelocity = requestedLinearVelocityOfRotation + requestedVelocityOfWheelAxises.getX();
        double blWheelRequestedVelocity = requestedLinearVelocityOfRotation - requestedVelocityOfWheelAxises.getX();
        double brWheelRequestedVelocity = requestedLinearVelocityOfRotation - requestedVelocityOfWheelAxises.getY();

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

        double flWheelMinAchievableFractionOfRequestedVelocity = Math.min(flWheelMinAchievableVelocity / flWheelRequestedVelocity, flWheelMaxAchievableVelocity / flWheelRequestedVelocity);
        double frWheelMinAchievableFractionOfRequestedVelocity = Math.min(frWheelMinAchievableVelocity / frWheelRequestedVelocity, frWheelMaxAchievableVelocity / frWheelRequestedVelocity);
        double blWheelMinAchievableFractionOfRequestedVelocity = Math.min(blWheelMinAchievableVelocity / blWheelRequestedVelocity, blWheelMaxAchievableVelocity / blWheelRequestedVelocity);
        double brWheelMinAchievableFractionOfRequestedVelocity = Math.min(brWheelMinAchievableVelocity / brWheelRequestedVelocity, brWheelMaxAchievableVelocity / brWheelRequestedVelocity);

        double flWheelMaxAchievableFractionOfRequestedVelocity = Math.max(flWheelMinAchievableVelocity / flWheelRequestedVelocity, flWheelMaxAchievableVelocity / flWheelRequestedVelocity);
        double frWheelMaxAchievableFractionOfRequestedVelocity = Math.max(frWheelMinAchievableVelocity / frWheelRequestedVelocity, frWheelMaxAchievableVelocity / frWheelRequestedVelocity);
        double blWheelMaxAchievableFractionOfRequestedVelocity = Math.max(blWheelMinAchievableVelocity / blWheelRequestedVelocity, blWheelMaxAchievableVelocity / blWheelRequestedVelocity);
        double brWheelMaxAchievableFractionOfRequestedVelocity = Math.max(brWheelMinAchievableVelocity / brWheelRequestedVelocity, brWheelMaxAchievableVelocity / brWheelRequestedVelocity);

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
        // In this case there is no achievable velocity where targetVelocity and targetRotationVelocity are kept proportional and targetVelocity's direction is kept the same
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

    // In inches per second relative to the drive's rotation
    public Vector2 getVelocity() {
        // In inches per second
        // X-axis is velocity along direction of fr and bl wheels
        // Y-axis is velocity along direction of fl and br wheels
        Vector2 velocityOfWheelAxises = new Vector2(
                (frWheel.getVelocity() - blWheel.getVelocity()) / 2.0,
                (flWheel.getVelocity() - brWheel.getVelocity()) / 2.0
        );

        // If the robot is to drive along one of it's axises, decrease targetVelocityOfDiagonals by taking into account VELOCITY_ALONG_AXIS_WHEEL_SLIP_FRACTION to compensate for wheel slippage
        velocityOfWheelAxises = velocityOfWheelAxises.mul(calculateWheelVelocityTransferFraction(velocityOfWheelAxises.getRotation()));

        // Rotate velocity to be relative to robot's axises
        return velocityOfWheelAxises.addRotation(45);
    }

    // In inches per second per second relative to the drive's rotation
    public Vector2 getAcceleration() {
        return acceleration;
    }

    // In degrees per second with positive counterclockwise
    public double getRotationVelocity() {
        // The average linear velocities of the drive motors is what determines the drive's rotation velocity
        double linearVelocityOfRotation = (flWheel.getVelocity() + frWheel.getVelocity() + blWheel.getVelocity() + brWheel.getVelocity()) / 4.0;

        return Degrees.fromRadians(linearVelocityOfRotation / WHEEL_BASE_RADIUS);
    }

    // In degrees per second per second with positive counterclockwise
    public double getRotationAcceleration() {
        return rotationAcceleration;
    }

    // 0.0 = complete slippage; 1.0 = no slippage
    private double calculateWheelVelocityTransferFraction(double velocityRotationRelativeToWheelAxises) {
        // 0.0 = velocities's direction is the full 45 degrees off of the robots axises
        // 1.0 = velocities is directly in line with one of the robots axises (forward, backward, left, or right)
        double velocityAlongWheelAxisesFraction = 0.5 * Degrees.cos(velocityRotationRelativeToWheelAxises * 4.0) + 0.5;

        // Return wheel velocity transfer fraction fudge factor
        return 1.0 - (velocityAlongWheelAxisesFraction * VELOCITY_ALONG_AXIS_WHEEL_SLIP_FRACTION);
    }

    // Called through Component.update()
    @Override
    void updateImpl() {
        // These values will change once lift.update() and slide.update() are called, so record them now so accelerations can be calculated
        Vector2 previousVelocity = getVelocity();
        double previousRotationVelocity = getRotationVelocity();

        flWheel.update();
        frWheel.update();
        blWheel.update();
        brWheel.update();

        // Calculate accelerations
        acceleration = getVelocity().sub(previousVelocity).div(deltaTime);
        rotationAcceleration = (getRotationVelocity() - previousRotationVelocity) / deltaTime;
    }

    // Returns text describing state
    @Override
    public String toString() {
        return "targetVelocity : " + Inches.toString(getTargetVelocity()) + "\n" +
                "velocity :       " + Inches.toString(getVelocity()) + "\n" +
                "acceleration :   " + Inches.toString(getAcceleration()) + "\n" +
                "targetRotationVelocity : " + Degrees.toString(getTargetRotationVelocity()) + "\n" +
                "rotationVelocity :       " + Degrees.toString(getRotationVelocity()) + "\n" +
                "rotationAcceleration :   " + Degrees.toString(getRotationAcceleration());
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