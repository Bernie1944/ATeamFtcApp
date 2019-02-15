package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Controls latch for lowering off of and lifting onto the lander
public class Latch extends Component {
    private static final String DRIVE_MOTOR_NAME = "LatchDrive";
    private static final String CATCH_SERVO_NAME = "LatchCatch";

    private static final int NUM_PINION_TEETH = 24;
    private static final double RACK_TEETH_PER_CM = 4.0;

    // In inches from bottom of robot to bottom of lift hook
    private static final double INITIAL_DRIVE_POSITION = 16.62;
    private static final double MIN_DRIVE_POSITION = 16.3;
    private static final double MAX_DRIVE_POSITION = 22.5;

    private static final double MAX_DRIVE_SPEED_BEFORE_TOUCHDOWN = 2.0;

    // In inches per second per inch
    private static final double DRIVE_SPEED_REDUCTION_RATE_BEFORE_TOUCHDOWN = 2.0;

    private static final double DISENGAGED_CATCH_POSITION = 0.35;
    private static final double RATCHETING_CATCH_POSITION = 0.6;
    private static final double ENGAGED_CATCH_POSITION = 0.75;

    private static final double POSITION_OVERSHOOT_WHEN_DISENGAGING_SERVO = 0.15;

    private final Motor drive;
    private final Servo theCatch;

    private double catchEngagementAmount;

    public Latch(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);

        drive = new Motor(telemetry, hardwareMap, DRIVE_MOTOR_NAME, NUM_PINION_TEETH / (RACK_TEETH_PER_CM / 2.54), 0.0, INITIAL_DRIVE_POSITION, MIN_DRIVE_POSITION, MAX_DRIVE_POSITION);
        theCatch = hardwareMap.servo.get(CATCH_SERVO_NAME);
    }

    public boolean isStopped() {
        return drive.isBraking() &&
                ((drive.isPositionAtMax() && theCatch.getController().getPwmStatus() == ServoController.PwmStatus.DISABLED) ||
                (theCatch.getController().getPwmStatus() == ServoController.PwmStatus.ENABLED && theCatch.getPosition() != DISENGAGED_CATCH_POSITION));
    }

    public void stop() {
        drive.brake();

        if (drive.isPositionAtMax()) {
            theCatch.getController().pwmDisable();
        } else {
            theCatch.getController().pwmEnable();
            theCatch.setPosition(ENGAGED_CATCH_POSITION);
        }
    }

    public boolean isEngaged() {
        return (drive.isPositionAt(INITIAL_DRIVE_POSITION) || drive.getPosition() < INITIAL_DRIVE_POSITION) && catchEngagementAmount > 0.0;
    }

    public void engage() {
        if (isEngaged() || drive.isPositionAtMin()) {
            drive.brake();
            theCatch.setPosition(ENGAGED_CATCH_POSITION);
        } else {
            drive.setTargetPosition(MIN_DRIVE_POSITION);
            theCatch.setPosition(RATCHETING_CATCH_POSITION);
        }
    }

    public boolean isDisengaged() {
        return drive.isPositionAtMax();
    }

    public void disengage() {
        theCatch.setPosition(DISENGAGED_CATCH_POSITION);

        if (isDisengaged()) {
            drive.brake();
        } else if (catchEngagementAmount > 0.0) {
            drive.setPower(-1.0);
        } else {
            drive.setTargetPosition(
                    MAX_DRIVE_POSITION,
                    MAX_DRIVE_SPEED_BEFORE_TOUCHDOWN + (DRIVE_SPEED_REDUCTION_RATE_BEFORE_TOUCHDOWN * (MAX_DRIVE_POSITION - drive.getPosition()))
            );
        }
    }
}