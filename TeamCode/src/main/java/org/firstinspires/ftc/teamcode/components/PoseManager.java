package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.Degrees;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Vector2;
import org.firstinspires.ftc.teamcode.util.Vector3;

public class PoseManager extends Component {
    // In inches extra to move farther inside tolerance before stopping trying to correct the position
    private static final double LATCH_POSITION_TOLERANCE_BUFFER = 3.0;

    // In inches per second per inch away from target rotation
    private static final double LATCH_POSITION_CORRECTION_FACTOR = 1.0;

    // In inches extra to move farther inside tolerance before stopping trying to correct the position
    private static final double NAV_POSITION_TOLERANCE_BUFFER = 3.0;

    // In inches per second per inch away from target rotation
    private static final double NAV_POSITION_CORRECTION_FACTOR = 1.0;

    // In degrees extra to move farther inside tolerance before stopping trying to correct the rotation
    private static final double NAV_ROTATION_TOLERANCE_BUFFER = 3.0;

    // In degrees per second per degree away from target rotation
    private static final double NAV_ROTATION_CORRECTION_FACTOR = 1.0;

    // In inches per second to set bucketLocalizer arm x speed to move bucketLocalizer arm to correct side of robot
    private static final double HARVESTER_ARM_SIDE_CORRECTION_X_SPEED = 1.0;

    // In inches extra to move farther inside tolerance before stopping trying to correct the position
    private static final double HARVESTER_POSITION_TOLERANCE_BUFFER = 3.0;

    // In inches per second per inch away from target rotation
    private static final double HARVESTER_POSITION_CORRECTION_FACTOR = 1.0;

    private final Latch latch;
    private final DriveLocalizer driveLocalizer;
    private final BucketLocalizer bucketLocalizer;

    public PoseManager(Telemetry telemetry, HardwareMap hardwareMap, Latch latch, DriveLocalizer driveLocalizer, BucketLocalizer bucketLocalizer) {
        super(telemetry, hardwareMap);
        this.latch = latch;
        this.driveLocalizer = driveLocalizer;
        this.bucketLocalizer = bucketLocalizer;
    }

    public boolean isAtLatchPosition(Pose.LatchPosition latchPosition) {
        return latchPosition == null || Math.abs(latch.getPosition() - latchPosition.target) <= latchPosition.tolerance;
    }

    public boolean isAtNavPosition(Pose.NavPosition navPosition) {
        return navPosition == null || driveLocalizer.getPosition().sub(navPosition.target).getMagnitude() <= navPosition.tolerance;
    }

    public boolean isAtNavRotation(Pose.NavRotation navRotation) {
        return navRotation == null || Math.abs(Degrees.normalize(driveLocalizer.getRotation() - navRotation.target)) <= navRotation.tolerance;
    }

    public boolean isAtHarvesterPosition(Pose.HarvesterPosition harvesterPosition) {
        return harvesterPosition == null || (
                bucketLocalizer.getPosition().sub(harvesterPosition.target).getMagnitude() <= harvesterPosition.tolerance &&
                        bucketLocalizer.bucket.getPosition().getX() < 0.0 == harvesterPosition.armIsBehind
        );
    }

    public boolean isAtPose(Pose pose) {
        return isAtLatchPosition(pose.getLatchPosition()) &&
                isAtNavPosition(pose.getNavPosition()) &&
                isAtNavRotation(pose.getNavRotation()) &&
                isAtHarvesterPosition(pose.getHarvesterPosition());
    }

    public void setTargetLatchPosition(Pose.LatchPosition targetLatchPosition) {
        if (targetLatchPosition != null) {
            double relativeTarget = targetLatchPosition.target - latch.getPosition();

            double toleranceWithBuffer = targetLatchPosition.tolerance - LATCH_POSITION_TOLERANCE_BUFFER;
            if (toleranceWithBuffer > 0.0) {
                if (relativeTarget > toleranceWithBuffer) {
                    relativeTarget += toleranceWithBuffer;
                } else if (relativeTarget < -toleranceWithBuffer) {
                    relativeTarget -= toleranceWithBuffer;
                } else {
                    relativeTarget = 0.0;
                }
            }

            latch.setTargetVelocity(relativeTarget * LATCH_POSITION_CORRECTION_FACTOR);
        }
    }

    public void setTargetNavPosition(Pose.NavPosition targetNavPosition) {
        if (targetNavPosition != null) {
            Vector2 relativeTarget = targetNavPosition.target.sub(driveLocalizer.getPosition());

            double toleranceWithBuffer = targetNavPosition.tolerance - NAV_POSITION_TOLERANCE_BUFFER;
            if (toleranceWithBuffer > 0.0) {
                if (relativeTarget.getMagnitude() > toleranceWithBuffer) {
                    relativeTarget = relativeTarget.addMagnitude(toleranceWithBuffer);
                } else {
                    relativeTarget = Vector2.ZERO;
                }
            }

            //driveLocalizer.setTargetVelocity(relativeTarget.mul(NAV_POSITION_CORRECTION_FACTOR));
        }
    }

    public void setTargetNavRotation(Pose.NavRotation targetNavRotation) {
        if (targetNavRotation != null) {
            double relativeTarget = Degrees.normalize(targetNavRotation.target - driveLocalizer.getRotation());

            double toleranceWithBuffer = targetNavRotation.tolerance - NAV_ROTATION_TOLERANCE_BUFFER;
            if (toleranceWithBuffer > 0.0) {
                if (relativeTarget > toleranceWithBuffer) {
                    relativeTarget += toleranceWithBuffer;
                } else if (relativeTarget < -toleranceWithBuffer) {
                    relativeTarget -= toleranceWithBuffer;
                } else {
                    relativeTarget = 0.0;
                }
            }

            //driveLocalizer.setTargetRotationVelocity(relativeTarget * NAV_ROTATION_CORRECTION_FACTOR);
        }
    }

    public void setTargetHarvesterPosition(Pose.HarvesterPosition targetHarvesterPosition) {
        if (targetHarvesterPosition != null) {
            /*
            if (targetHarvesterPosition.armIsBehind && bucket.getPosition().getX() >= 0.0) {
                bucket.setTargetVelocity(new Vector2(-HARVESTER_ARM_SIDE_CORRECTION_X_SPEED, 0.0));
            } else if (!targetHarvesterPosition.armIsBehind && bucket.getPosition().getX() < 0.0) {
                bucket.setTargetVelocity(new Vector2(HARVESTER_ARM_SIDE_CORRECTION_X_SPEED, 0.0));
            } else {
                Vector3 relativeTarget = targetHarvesterPosition.target.sub(bucketLocalizer.getPosition());

                double toleranceWithBuffer = targetHarvesterPosition.tolerance - HARVESTER_POSITION_TOLERANCE_BUFFER;
                if (toleranceWithBuffer > 0.0) {
                    if (relativeTarget.getMagnitude() > toleranceWithBuffer) {
                        relativeTarget = relativeTarget.addMagnitude(toleranceWithBuffer);
                    } else {
                        relativeTarget = Vector3.ZERO;
                    }
                }

                bucketLocalizer.setTargetVelocity(relativeTarget.mul(HARVESTER_POSITION_CORRECTION_FACTOR));
            }
            */
        }
    }

    public void setTargetPose(Pose targetPose) {
        setTargetLatchPosition(targetPose.getPredictiveLatchPosition());
        setTargetNavPosition(targetPose.getPredictiveNavPosition());
        setTargetNavRotation(targetPose.getPredictiveNavRotation());
        setTargetHarvesterPosition(targetPose.getPredictiveHarvesterPosition());
    }

    /*
    // Returns text describing state
    @Override
    public String toString() {
        return String.format(
                Locale.US,
                "isAtLatchPosition :   %b" + "%n" +
                        "isAtNavPosition :   %b" + "%n" +
                        "isAtNavRotation :   %b" + "%n" +
                        "isAtHarvesterPosition :   %b" + "%n" +
                        "isAtPose :   %b",
                isAtLatchPosition(),
                isAtNavPosition(),
                isAtNavRotation(),
                isAtHarvesterPosition(),
                isAtPose()
        );
    }
    */
}