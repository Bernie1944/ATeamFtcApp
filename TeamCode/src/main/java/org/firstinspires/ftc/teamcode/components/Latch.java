package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Locale;

// Describes and controls latch for lowering off of and lifting onto the lander
public class Latch extends Component {
    private static final String DRIVE_MOTOR_NAME = "BucketPivotAndLatchDrive";
    private static final String CATCH_SERVO_NAME = "LatchCatch";

    private static final int NUM_PINION_TEETH = 24;
    private static final double RACK_TEETH_PER_CM = 4.0;

    private static final double MIN_CATCH_ENGAGEMENT_AMOUNT_WHEN_CATCH_TARGET_POSITION_NOT_DISENGAGED = 0.1;
    private static final double MAX_CATCH_ENGAGEMENT_AMOUNT = 0.4; //0.5; //0.3;

    // In inches from bottom of robot to bottom of lift hook
    private static final double INITIAL_DRIVE_POSITION = 15.25;
    private static final double MIN_DRIVE_POSITION = 14.4;
    private static final double MAX_DRIVE_POSITION = 22.75;

    private static final double DISENGAGED_CATCH_POSITION = 0.35;
    private static final double RATCHETING_CATCH_POSITION = 0.27;
    private static final double ENGAGED_CATCH_POSITION = 0.15; //0.16; //0.17;

    private final Motor drive;
    private final Servo theCatch;

    private static double catchEngagementAmount = MAX_CATCH_ENGAGEMENT_AMOUNT;

    // resetCatchEngagementAmount: should static variable catchEngagementAmount be set to the initial MAX_CATCH_ENGAGEMENT_AMOUNT
    public Latch(Telemetry telemetry, HardwareMap hardwareMap, boolean resetCatchEngagementAmount) {
        super(telemetry, hardwareMap);

        if (resetCatchEngagementAmount) catchEngagementAmount = MAX_CATCH_ENGAGEMENT_AMOUNT;

        drive = new Motor(telemetry, hardwareMap, DRIVE_MOTOR_NAME, "%.2fin", NUM_PINION_TEETH / (RACK_TEETH_PER_CM * 2.54), 0.0,
                INITIAL_DRIVE_POSITION, MIN_DRIVE_POSITION, MAX_DRIVE_POSITION);
        theCatch = hardwareMap.servo.get(CATCH_SERVO_NAME);
    }

    public boolean isEngaged() {
        return drive.isPositionLessThanOrAt(INITIAL_DRIVE_POSITION) && catchEngagementAmount > 0.0;
    }

    public void engage() {
        if (isEngaged() || drive.isPositionLessThanOrAt(INITIAL_DRIVE_POSITION - MAX_CATCH_ENGAGEMENT_AMOUNT)) {
            drive.brake();
            theCatch.setPosition(ENGAGED_CATCH_POSITION);
        } else {
            drive.setTargetPosition(INITIAL_DRIVE_POSITION - MAX_CATCH_ENGAGEMENT_AMOUNT);
            theCatch.setPosition(RATCHETING_CATCH_POSITION);
        }

        if (catchEngagementAmount < MIN_CATCH_ENGAGEMENT_AMOUNT_WHEN_CATCH_TARGET_POSITION_NOT_DISENGAGED) {
            catchEngagementAmount = MIN_CATCH_ENGAGEMENT_AMOUNT_WHEN_CATCH_TARGET_POSITION_NOT_DISENGAGED;
        }
    }

    public boolean isDisengaged() {
        return drive.isPositionGreaterThanOrAt(MAX_DRIVE_POSITION - 0.05);
    }

    public void disengage() {
        theCatch.setPosition(DISENGAGED_CATCH_POSITION);

        if (catchEngagementAmount > 0.0) {
            drive.setPower(-1.0);
        } else if (isDisengaged()) {
            drive.brake();
        } else {
            drive.setTargetPosition(MAX_DRIVE_POSITION);
        }
    }

    public boolean isStopped() {
        return drive.isBraking();
    }

    public void stop() {
        drive.brake();

        if (isDisengaged() && catchEngagementAmount == 0.0) {
            theCatch.setPosition(DISENGAGED_CATCH_POSITION);
        } else {
            theCatch.setPosition(ENGAGED_CATCH_POSITION);

            if (catchEngagementAmount < MIN_CATCH_ENGAGEMENT_AMOUNT_WHEN_CATCH_TARGET_POSITION_NOT_DISENGAGED) {
                catchEngagementAmount = MIN_CATCH_ENGAGEMENT_AMOUNT_WHEN_CATCH_TARGET_POSITION_NOT_DISENGAGED;
            }
        }
    }

    // In inches
    public double getCatchEngagementAmount() {
        return catchEngagementAmount;
    }

    // Called through Component.update()
    @Override
    void internalUpdate() {
        double previousDrivePosition = drive.getPosition();
        drive.update();
        double deltaDrivePosition = drive.getPosition() - previousDrivePosition;

        if (catchEngagementAmount > 0.0 || theCatch.getPosition() == ENGAGED_CATCH_POSITION || theCatch.getPosition() == RATCHETING_CATCH_POSITION) {
            catchEngagementAmount += deltaDrivePosition;
        }

        catchEngagementAmount = Range.clip(catchEngagementAmount, 0.0, MAX_CATCH_ENGAGEMENT_AMOUNT);
    }

    @Override
    public String toString() {
                return createStateString("engaged", isEngaged()) +
                        createStateString("disengaged", isDisengaged()) +
                        createStateString("stopped", isStopped()) +
                        createStateString("catchEngagementAmount", "%.2fin", catchEngagementAmount) +
                        createStateString("theCatch.position", "%.2f", theCatch.getPosition()) +
                        drive.toString();
            }
}