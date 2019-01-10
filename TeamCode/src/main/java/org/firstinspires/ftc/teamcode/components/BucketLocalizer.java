package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.Degrees;
import org.firstinspires.ftc.teamcode.util.Inches;
import org.firstinspires.ftc.teamcode.util.Vector2;
import org.firstinspires.ftc.teamcode.util.Vector3;

// Controls harvester system that is used to move harvester to a position
public class BucketLocalizer extends Component {
    // In inches around robot's center
    // setTargetVelocity() will not move harvester arm position through this area
    private static final double SMALLEST_HARVESTER_ARM_X_POSITION = 2.5;

    public final Bucket bucket;

    // Need access to get robots orientation relative to playing field
    private final DriveLocalizer driveLocalizer;

    // driveLocalizer and reverseSlideLine states may be modified
    public BucketLocalizer(Telemetry telemetry, HardwareMap hardwareMap, DriveLocalizer driveLocalizer) {
        super(telemetry, hardwareMap);
        this.driveLocalizer = driveLocalizer;
        this.bucket = new Bucket(telemetry, hardwareMap);
    }

    // In inches of front bottom lip of harvester relative to center of playing field
    // Positive x-axis is to the right of the driving team, positive y-axis is to the front of the driving team, and positive z-axis is up
    public Vector3 getPosition() {
        Vector2 horizontalPosition = new Vector2(
                bucket.getPosition().getX(),
                0.0
        ).addRotation(driveLocalizer.getRotation()).add(driveLocalizer.getPosition());

        return horizontalPosition.appendZ(bucket.getPosition().getY());
    }

    // In inches per second of front bottom lip of harvester relative to playing field
    // Positive x-axis is to the right of the driving team, positive y-axis is to the front of the driving team, and positive z-axis is up
    public Vector3 getVelocity() {
        Vector2 horizontalVelocity = new Vector2(
                bucket.getVelocity().getX(),
                Degrees.toRadians(driveLocalizer.getRotationVelocity()) * bucket.getPosition().getX()
        ).addRotation(driveLocalizer.getRotation()).add(driveLocalizer.getVelocity());

        return horizontalVelocity.appendZ(bucket.getVelocity().getY());
    }

    // In inches per second per second of front bottom lip of harvester relative to playing field
    // Positive x-axis is to the right of the driving team, positive y-axis is to the front of the driving team, and positive z-axis is up
    public Vector3 getAcceleration() {
        // "Center pointing" acceleration towards center of robot's rotation
        double centripetalAcceleration = bucket.getPosition().getX() * Math.pow(Degrees.toRadians(driveLocalizer.getRotationVelocity()), 2.0);

        Vector2 horizontalAcceleration = new Vector2(
                bucket.getAcceleration().getX() - centripetalAcceleration,
                Degrees.toRadians(driveLocalizer.getRotationAcceleration()) * bucket.getPosition().getX()
        ).addRotation(driveLocalizer.getRotation()).add(driveLocalizer.getAcceleration());

        return horizontalAcceleration.appendZ(bucket.getAcceleration().getY());
    }

    // In inches per second of front bottom lip of harvester relative to playing field
    // Positive x-axis is to the right of the driving team, positive y-axis is to the front of the driving team, and positive z-axis is up
    public Vector3 getTargetVelocity() {
        Vector2 horizontalTargetVelocity = new Vector2(
                bucket.getTargetVelocity().getX(),
                Degrees.toRadians(driveLocalizer.getTargetRotationVelocity()) * bucket.getPosition().getX()
        ).addRotation(driveLocalizer.getRotation()).add(driveLocalizer.getTargetVelocity());

        return horizontalTargetVelocity.appendZ(bucket.getTargetVelocity().getY());
    }

    // In inches per second of front bottom lip of harvester relative to playing field
    // Positive x-axis is to the right of the driving team, positive y-axis is to the front of the driving team, and positive z-axis is up
    // This method will set reverseSlideLine target velocity and driveLocalizer target rotation velocity and may modify driveLocalizer target velocity
    // It should not move harvester the whole way through the center of the robot to the opposite side (reverseSlideLine x position should not change signs)
    public void setTargetVelocity(Vector3 targetVelocity) {
        // Target velocity relative to robot
        // X-axis is under most circumstances (explained below) target reverseSlideLine x velocity
        // Y-axis is related to driveLocalizer target rotation velocity
        Vector2 horizontalRelativeTargetVelocity = targetVelocity.getXY().sub(driveLocalizer.getTargetVelocity()).subRotation(driveLocalizer.getRotation());

        // As harvester arm position gets close to 0.0 harvester's side-to-side motion becomes more and more constrained,
        // making the calculations below produce increasingly larger driveLocalizer target rotation velocity values, going to infinity or NaN
        // To avoid these extremely large values, allowedBucketXPosition is not allowed to be smaller than SMALLEST_HARVESTER_ARM_X_POSITION
        double allowedBucketXPosition;
        if (Math.abs(bucket.getPosition().getX()) >= SMALLEST_HARVESTER_ARM_X_POSITION) {
            allowedBucketXPosition = bucket.getPosition().getX();
        } else if (bucket.getPosition().getX() < 0.0) {
            allowedBucketXPosition = -SMALLEST_HARVESTER_ARM_X_POSITION;
        } else {
            allowedBucketXPosition = SMALLEST_HARVESTER_ARM_X_POSITION;
        }

        if (
                Math.abs(bucket.getPosition().getX()) >= SMALLEST_HARVESTER_ARM_X_POSITION ||
                        (bucket.getPosition().getX() < 0.0 && horizontalRelativeTargetVelocity.getX() < 0.0) ||
                        (bucket.getPosition().getX() >= 0.0 && horizontalRelativeTargetVelocity.getX() > 0.0)
                ) {
            // reverseSlideLine x position is either not smaller than SMALLEST_HARVESTER_ARM_X_POSITION or is going to be moving out of SMALLEST_HARVESTER_ARM_X_POSITION,
            // so no special considerations need to be made about keeping the harvester from moving through the center of the robot to the opposite side

            bucket.setTargetVelocity(new Vector2(horizontalRelativeTargetVelocity.getX(), targetVelocity.getZ()));

            Vector2 driveLocalizerAdditionalTargetVelocity =
                    ((horizontalRelativeTargetVelocity.getX() < 0.0 && bucket.isArmAtMinPosition()) ||
                            (horizontalRelativeTargetVelocity.getX() > 0.0 && bucket.isArmAtMaxPosition())) ?
                    new Vector2(horizontalRelativeTargetVelocity.getX() - bucket.getTargetVelocity().getX(), 0.0).addRotation(driveLocalizer.getRotation()) :
                            Vector2.ZERO;

            driveLocalizer.setTargetVelocityAndTargetRotationVelocity(
                    driveLocalizer.getTargetVelocity().add(driveLocalizerAdditionalTargetVelocity),
                    Degrees.fromRadians(horizontalRelativeTargetVelocity.getY() / allowedBucketXPosition)
            );
        } else {
            // reverseSlideLine x position would be moving into of SMALLEST_HARVESTER_ARM_X_POSITION,
            // so special consideration needs to be made to keep harvester from going through center of robot to the other side

            // First, don't run reverseSlideLine x position any farther into SMALLEST_HARVESTER_ARM_X_POSITION
            bucket.setTargetVelocity(new Vector2(0.0, targetVelocity.getZ()));

            // Because horizontalRelativeTargetVelocity x was not used to set the target velocity of reverseSlideLine,
            // instead of ignoring it, add its magnitude to horizontalRelativeTargetVelocity y
            // This is important because it will keep the harvester from getting "stuck" when targetVelocity is directing the harvester toward the opposite side of the robot
            double horizontalRelativeTargetVelocityXContributionToY =
                    horizontalRelativeTargetVelocity.getY() < 0.0 ? -Math.abs(horizontalRelativeTargetVelocity.getX()) : Math.abs(horizontalRelativeTargetVelocity.getX());

            driveLocalizer.setTargetVelocityAndTargetRotationVelocity(
                    driveLocalizer.getTargetVelocity(),
                    Degrees.fromRadians((horizontalRelativeTargetVelocity.getY() + horizontalRelativeTargetVelocityXContributionToY) / allowedBucketXPosition)
            );
        }
    }

    // Called through Component.update()
    @Override
    void updateImpl() {
        bucket.update();
    }

    // Returns text describing state
    @Override
    public String toString() {
        return "position :       " + Inches.toString(getPosition()) + "\n" +
                "targetVelocity : " + Inches.toString(getTargetVelocity()) + "\n" +
                "velocity :       " + Inches.toString(getVelocity()) + "\n" +
                "acceleration :   " + Inches.toString(getAcceleration());
    }

    // Returns text verbosely describing state
    @Override
    public String toStringVerbose() {
        return toString() + "\n" +
                "bucket {\n" +
                bucket.toStringVerbose() + "\n" +
                "}";
    }
}
