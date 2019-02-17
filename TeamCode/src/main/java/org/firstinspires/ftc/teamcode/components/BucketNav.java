package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.Degrees;
import org.firstinspires.ftc.teamcode.util.Inches;
import org.firstinspires.ftc.teamcode.util.Vector2;

// Controls bucket system that is used to move bucket to a position
public class BucketNav extends Component {
    // In inches around robot's center
    // setTargetVelocity() will not move bucket slide position through this area
    private static final double SMALLEST_BUCKET_SLIDE_POSITION = 2.5;

    private final Bucket bucket;
    private final Nav nav;

    // bucket and nav states may be modified
    public BucketNav(Telemetry telemetry, HardwareMap hardwareMap, Bucket bucket, Nav nav) {
        super(telemetry, hardwareMap);
        this.bucket = bucket;
        this.nav = nav;
    }

    // Z-position is inferred from xYPosition (See Bucket.SLIDE_POSITION_TO_PIVOT_SHAFT_POSITION_MAP)
    public boolean isBucketPositionAt(Vector2 xYPosition) {
        return bucket.isPositionAt(xYPosition.subRotation(nav.getRotation()).getX());
    }
/*
    // In inches per second of front bottom lip of harvester relative to playing field
    // Positive x-axis is to the right of the driving team, positive y-axis is to the front of the driving team, and positive z-axis is up
    // Target z-position is inferred from bucket.slide position (See Bucket.SLIDE_POSITION_TO_PIVOT_SHAFT_POSITION_MAP)
    // This function is not designed to work when bucket is close to being below slide pivot
    public void setTargetVelocitiesAndBucketZPosition(Vector2 targetBucketXYVelocity, ) {
        // Target velocity relative to robot
        // X-axis is under most circumstances (explained below) target reverseSlideLine x velocity
        // Y-axis is related to nav target rotation velocity
        Vector2 relativeTargetXYVelocity = targetXYVelocity.sub(nav.getTargetVelocity()).subRotation(nav.getRotation());

        // As harvester arm position gets close to 0.0 harvester's side-to-side motion becomes more and more constrained,
        // making the calculations below produce increasingly larger nav target rotation velocity values, going to infinity or NaN
        // To avoid these extremely large values, allowedBucketSlidePosition is not allowed to be smaller than SMALLEST_BUCKET_SLIDE_POSITION
        double allowedBucketSlidePosition;
        if (Math.abs(bucket.slide.getPosition()) >= SMALLEST_BUCKET_SLIDE_POSITION) {
            allowedBucketSlidePosition = bucket.slide.getPosition();
        } else if (bucket.slide.getPosition() < 0.0) {
            allowedBucketSlidePosition = -SMALLEST_BUCKET_SLIDE_POSITION;
        } else {
            allowedBucketSlidePosition = SMALLEST_BUCKET_SLIDE_POSITION;
        }

        if (
                Math.abs(bucket.slide.getPosition()) >= SMALLEST_BUCKET_SLIDE_POSITION ||
                        (bucket.slide.getPosition() < 0.0 && relativeTargetXYVelocity.getX() < 0.0) ||
                        (bucket.slide.getPosition() >= 0.0 && relativeTargetXYVelocity.getX() > 0.0)
                ) {
            // reverseSlideLine x position is either not smaller than SMALLEST_BUCKET_SLIDE_POSITION or is going to be moving out of SMALLEST_BUCKET_SLIDE_POSITION,
            // so no special considerations need to be made about keeping the harvester from moving through the center of the robot to the opposite side

            bucket.setTargetXVelocityAndYPosition(relativeTargetXYVelocity.getX());

            Vector2 driveLocalizerAdditionalTargetVelocity =
                    ((relativeTargetXYVelocity.getX() < 0.0 && bucket.slide.isPositionAtMin()) ||
                            (relativeTargetXYVelocity.getX() > 0.0 && bucket.slide.isPositionAtMax())) ?
                    new Vector2(relativeTargetXYVelocity.getX() - bucket.getTargetVelocity().getX(), 0.0).addRotation(nav.getRotation()) :
                            Vector2.ZERO;

            nav.setTargetVelocityAndTargetRotationVelocity(
                    nav.getTargetVelocity().add(driveLocalizerAdditionalTargetVelocity),
                    Degrees.fromRadians(relativeTargetXYVelocity.getY() / allowedBucketSlidePosition)
            );
        } else {
            // reverseSlideLine x position would be moving into of SMALLEST_BUCKET_SLIDE_POSITION,
            // so special consideration needs to be made to keep harvester from going through center of robot to the other side

            // First, don't run reverseSlideLine x position any farther into SMALLEST_BUCKET_SLIDE_POSITION
            bucket.setTargetVelocity(new Vector2(0.0, targetVelocity.getZ()));

            // Because relativeTargetXYVelocity x was not used to set the target velocity of reverseSlideLine,
            // instead of ignoring it, add its magnitude to relativeTargetXYVelocity y
            // This is important because it will keep the harvester from getting "stuck" when targetVelocity is directing the harvester toward the opposite side of the robot
            double horizontalRelativeTargetVelocityXContributionToY =
                    relativeTargetXYVelocity.getY() < 0.0 ? -Math.abs(relativeTargetXYVelocity.getX()) : Math.abs(relativeTargetXYVelocity.getX());

            nav.setTargetVelocityAndTargetRotationVelocity(
                    nav.getTargetVelocity(),
                    Degrees.fromRadians((relativeTargetXYVelocity.getY() + horizontalRelativeTargetVelocityXContributionToY) / allowedBucketSlidePosition)
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
    */
}
