package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.components.Automator;
import org.firstinspires.ftc.teamcode.components.Bucket;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.components.Latch;
import org.firstinspires.ftc.teamcode.components.Nav;
import org.firstinspires.ftc.teamcode.util.Degrees;
import org.firstinspires.ftc.teamcode.util.Vector2;

@TeleOp(name="Tele Op")
public class TeleOpMode extends OpMode {
    // When using an Xbox 360 controller configured as a Logitech controller under the Driver Station app's Settings,
    // These should be set to a value greater than 0.0 (0.13 - 0.14 seems to work well),
    // which will result in more "smooth" joystick controls that do not seem to "stick" to the x and y axis (this, however, is not preferable in all cases)
    private static final double CONTROLLER_1_JOYSTICK_MAGNITUDE_DEADZONE = 0.14;
    private static final double CONTROLLER_2_JOYSTICK_MAGNITUDE_DEADZONE = 0.0;

    // In inches per second
    private static final double MAX_NAV_TARGET_SPEED = 22.0;

    // In degrees per second
    private static final double MAX_NAV_TARGET_ANGULAR_SPEED = 180.0;

    // In inches per second
    private static final double MAX_BUCKET_TARGET_SPEED = 40.0;

    // Between [0, 1]
    private static final double JOYSTICK_AT_MAX_MAGNITUDE_THRESHOLD = 0.97;

    // In inches
    private static final double MINERAL_PICKUP_POSITION_FORWARD_MOVEMENT_PER_TRIGGER_PRESS = 9.0; //12.0;

    // In inches
    private static final double MINERAL_PICKUP_POSITION_SIDEWAYS_MOVEMENT_PER_TRIGGER_PRESS = 7.0; //9.0;

    // In inches
    private static final double ADJUST_MINERAL_PICKUP_POSITION_STEP = 2.5; //1.25;

    // In inches
    private static final double ADJUST_MINERAL_DELIVERY_POSITION_STEP = 3.0; //4.0; //2.0; //1.0;

    // In seconds for calculating deltaTime
    private double previousTime;

    // In seconds between last two calls to update()
    // Op modes loop about every tenth of a second, but this provides a precise value
    // Initialized to a reasonable value
    private double deltaTime;

    private boolean initSuccessful = false;
    private boolean controller1AdjustedMineralPickupPosition = false;
    private boolean controller2AdjustedMineralPickupPosition = false;

    private Nav nav;
    private Bucket bucket;
    private Latch latch;
    private Automator automator;
    private Controller controller1;
    private Controller controller2;

    public TeleOpMode() {
        // The BNO055IMU inertial motion units can take a long time to initialize, as the class will try initialization five times before it gives up,
        // which takes close to 12 seconds, so it is necessary to increase msStuckDetectInit
        msStuckDetectInit = 12000;
    }

    @Override
    public void init() {
        try {
            Thread.currentThread().setPriority(Thread.MAX_PRIORITY);

            previousTime = time - 0.1;

            nav = new Nav(telemetry, hardwareMap);
            latch = new Latch(telemetry, hardwareMap, false);
            bucket = new Bucket(telemetry, hardwareMap, nav, latch);
            automator = new Automator(telemetry, hardwareMap, nav, bucket, latch);
            controller1 = new Controller(telemetry, hardwareMap, "Controller1", gamepad1, CONTROLLER_1_JOYSTICK_MAGNITUDE_DEADZONE);
            controller2 = new Controller(telemetry, hardwareMap, "Controller2", gamepad2, CONTROLLER_2_JOYSTICK_MAGNITUDE_DEADZONE);

            initSuccessful = true;

            // Update telemetry data only before telemetry data is being sent to driver station
            telemetry.addData("State", new Func<String>() {
                @Override
                public String value() {
                    return "\n" + TeleOpMode.this.toString();
                }
            });
        } catch (Component.InitializationFailedException e) {
            telemetry.log().add(e.toString());
            requestOpModeStop();
        }
    }

    private void adjustMineralPickupPosition() {
        if (controller1.isRightBumperPressed()) controller1AdjustedMineralPickupPosition = true;
        if (controller2.isRightBumperPressed()) controller2AdjustedMineralPickupPosition = true;

        boolean undoController1AdjustingMineralPickupPosition = controller1AdjustedMineralPickupPosition &&
                (controller1.isAButtonDown() || controller1.isXButtonDown() || controller1.isYButtonDown());

        boolean undoController2AdjustingMineralPickupPosition = controller2AdjustedMineralPickupPosition &&
                (controller2.isAButtonDown() || controller2.isXButtonDown() || controller2.isYButtonDown());

        if (undoController1AdjustingMineralPickupPosition) controller1AdjustedMineralPickupPosition = false;
        if (undoController2AdjustingMineralPickupPosition) controller2AdjustedMineralPickupPosition = false;

        double deltaMineralPickupRotationFactor = (controller1.isLeftBumperPressed() ? 1.0 : 0.0) + (controller2.isLeftBumperPressed() ? 1.0 : 0.0) +
                (controller1.isRightBumperPressed() ? -1.0 : 0.0) + (controller2.isRightBumperPressed() ? -1.0 : 0.0) +
                (undoController1AdjustingMineralPickupPosition ? 1.0 : 0.0) + (undoController1AdjustingMineralPickupPosition ? 1.0 : 0.0);

        Vector2 relativeMineralPickupPosition = automator.getMineralPickupPosition().sub(nav.getPosition());

        double deltaMineralPickupRotation = deltaMineralPickupRotationFactor * Degrees.fromRadians(
                ADJUST_MINERAL_PICKUP_POSITION_STEP /
                        (bucket.getRelativePosition() > Bucket.MAX_UNEXTENDED_SLIDE_POSITION ? bucket.getRelativePosition() : relativeMineralPickupPosition.getMagnitude())
        );

        relativeMineralPickupPosition = relativeMineralPickupPosition.addRotation(deltaMineralPickupRotation);

        automator.setMineralPickupPosition(relativeMineralPickupPosition.add(nav.getPosition()));
    }

    private void adjustMineralDeliveryPosition() {
        Vector2 deltaMineralDeliveryPositions = new Vector2(
                (controller1.isLeftButtonPressed() ? -1.0 : 0.0) + (controller2.isLeftButtonPressed() ? -1.0 : 0.0) +
                        (controller1.isRightButtonPressed() ? 1.0 : 0.0) + (controller2.isRightButtonPressed() ? 1.0 : 0.0),
                (controller1.isDownButtonPressed() ? -1.0 : 0.0) + (controller2.isDownButtonPressed() ? -1.0 : 0.0) +
                        (controller1.isUpButtonPressed() ? 1.0 : 0.0) + (controller2.isUpButtonPressed() ? 1.0 : 0.0)
        ).mul(ADJUST_MINERAL_DELIVERY_POSITION_STEP);

        nav.setPosition(nav.getPosition().sub(deltaMineralDeliveryPositions));
        automator.setMineralPickupPosition(automator.getMineralPickupPosition().sub(deltaMineralDeliveryPositions));
    }

    private void setMineralPickupPositionAtNavHeading() {
        automator.setMineralPickupPosition(nav.getPosition().add(
                new Vector2(automator.getMineralPickupPosition().getDistanceFrom(nav.getPosition()), 0.0).addRotation(nav.getHeading())
        ));
    }

    private void setMineralDeliveryPositionToBucketPosition(Vector2 mineralDeliveryPosition) {
        if (bucket.getSlidePosition() < Bucket.MIN_UNEXTENDED_SLIDE_POSITION) {
            Vector2 previousNavPosition = nav.getPosition();
            nav.setPosition(mineralDeliveryPosition.sub(new Vector2(bucket.getRelativePosition(), 0.0).addRotation(nav.getHeading())));
            automator.setMineralPickupPosition(automator.getMineralPickupPosition().add(nav.getPosition().sub(previousNavPosition)));
        }
    }

    private void setAutomatorMode() {
        if (controller1.isAButtonPressed() || controller2.isAButtonPressed() ||
                (controller1.isAButtonDown() &&
                        (controller1.isLeftButtonDown() || controller1.isRightButtonDown() || controller1.isDownButtonDown() || controller1.isUpButtonDown())) ||
                (controller2.isAButtonDown() &&
                        (controller2.isLeftButtonDown() || controller2.isRightButtonDown() || controller2.isDownButtonDown() || controller2.isUpButtonDown()))) {
            automator.setMode(Automator.Mode.PREPARE_TO_DELIVER_SILVER);
        } else if ((controller1.isXButtonPressed() && !controller1.isAButtonDown()) ||
                (controller2.isXButtonPressed() && !controller2.isAButtonDown()) ||
                (controller1.isXButtonDown() &&
                        (controller1.isLeftButtonDown() || controller1.isRightButtonDown() || controller1.isDownButtonDown() || controller1.isUpButtonDown())) ||
                (controller2.isXButtonDown() &&
                        (controller2.isLeftButtonDown() || controller2.isRightButtonDown() || controller2.isDownButtonDown() || controller2.isUpButtonDown()))) {
            automator.setMode(Automator.Mode.PREPARE_TO_DELIVER_GOLD);
        } else if ((controller1.isAButtonDown() && controller1.isXButtonReleased()) ||
                (controller1.isXButtonDown() && controller1.isAButtonReleased()) ||
                (controller1.isAButtonReleased() && controller1.isXButtonReleased()) ||
                (controller2.isAButtonDown() && controller2.isXButtonReleased()) ||
                (controller2.isXButtonDown() && controller2.isAButtonReleased()) ||
                (controller2.isAButtonReleased() && controller2.isXButtonReleased())) {
            automator.setMode(Automator.Mode.DELIVER_SILVER_THEN_GOLD);
        } else if (automator.getMode() != Automator.Mode.DELIVER_SILVER_THEN_GOLD && (controller1.isAButtonReleased() || controller2.isAButtonReleased())) {
            automator.setMode(Automator.Mode.DELIVER_SILVER_ONLY);
        } else if (automator.getMode() != Automator.Mode.DELIVER_SILVER_THEN_GOLD && (controller1.isXButtonReleased() || controller2.isXButtonReleased())) {
            automator.setMode(Automator.Mode.DELIVER_GOLD_ONLY);
        } else if ((controller1.isAButtonDown() && (!controller1.getLeftJoystickPosition().equals(Vector2.ZERO) || automator.getMode() == Automator.Mode.NONE)) ||
                (controller2.isAButtonDown() && (!controller2.getLeftJoystickPosition().equals(Vector2.ZERO) || automator.getMode() == Automator.Mode.NONE))) {
            setMineralDeliveryPositionToBucketPosition(Automator.SILVER_DELIVERY_BUCKET_TARGET_POSITION);
            automator.setMode(Automator.Mode.NONE);
        } else if ((controller1.isXButtonDown() && (!controller1.getLeftJoystickPosition().equals(Vector2.ZERO) || automator.getMode() == Automator.Mode.NONE)) ||
                (controller2.isXButtonDown() && (!controller2.getLeftJoystickPosition().equals(Vector2.ZERO) || automator.getMode() == Automator.Mode.NONE))) {
            setMineralDeliveryPositionToBucketPosition(Automator.GOLD_DELIVERY_BUCKET_TARGET_POSIITON);
            automator.setMode(Automator.Mode.NONE);
        }

        /*
        if ((controller1.isAButtonDown() && controller1.isXButtonDown()) || (controller2.isAButtonDown() && controller2.isXButtonDown())) {
            if ((controller1.isRightBumperDown() && controller1.isAButtonDown() && controller1.isXButtonDown() &&
                    (controller1.isRightBumperPressed() || controller1.isAButtonPressed() || controller1.isXButtonPressed())) ||
                    (controller2.isRightBumperDown() && controller2.isAButtonDown() && controller2.isXButtonDown() &&
                            (controller2.isRightBumperPressed() || controller2.isAButtonPressed() || controller2.isXButtonPressed()))) {
                setMineralDeliveryPositionToBucketPosition(Automator.SILVER_DELIVERY_BUCKET_TARGET_POSITION);
            }

            automator.setMode(Automator.Mode.DELIVER_SILVER_THEN_GOLD);
        } else if ((controller1.isAButtonDown() || controller2.isAButtonDown()) &&
                (automator.getMode() != Automator.Mode.DELIVER_SILVER_THEN_GOLD || controller1.isAButtonPressed() || controller2.isAButtonPressed())) {
            if ((controller1.isRightBumperDown() && controller1.isAButtonDown() &&
                    (controller1.isRightBumperPressed() || controller1.isAButtonPressed())) ||
                    (controller2.isRightBumperDown() && controller2.isAButtonDown() &&
                            (controller2.isRightBumperPressed() || controller2.isAButtonPressed()))) {
                setMineralDeliveryPositionToBucketPosition(Automator.SILVER_DELIVERY_BUCKET_TARGET_POSITION);
            }

            automator.setMode(Automator.Mode.DELIVER_SILVER_ONLY);
        } else if ((controller1.isXButtonDown() || controller2.isXButtonDown()) &&
                (automator.getMode() != Automator.Mode.DELIVER_SILVER_THEN_GOLD || controller1.isXButtonPressed() || controller2.isXButtonPressed())) {
            if ((controller1.isRightBumperDown() && controller1.isXButtonDown() &&
                    (controller1.isRightBumperPressed() || controller1.isXButtonPressed())) ||
                    (controller2.isRightBumperDown() && controller2.isXButtonDown() &&
                            (controller2.isRightBumperPressed() || controller2.isXButtonPressed()))) {
                setMineralDeliveryPositionToBucketPosition(Automator.GOLD_DELIVERY_BUCKET_TARGET_POSIITON);
            }

            automator.setMode(Automator.Mode.DELIVER_GOLD_ONLY);
        } */

        else if ((controller1.isLeftButtonDown() || controller2.isLeftButtonDown()) && (
                automator.getMode() == Automator.Mode.DISENGAGE_LATCH ||
                        (controller1.isRightBumperDown() && controller1.isYButtonDown()) ||
                        (controller2.isRightBumperDown() && controller2.isYButtonDown())
        )) {
            automator.setDisengageLatchNavTargetHeading(135.0);
            automator.setMode(Automator.Mode.DISENGAGE_LATCH);
        } else if ((controller1.isYButtonDown() && controller1.isLeftBumperDown()) ||
                (controller2.isYButtonDown() && controller2.isLeftBumperDown())) {
            automator.setMode(Automator.Mode.CLEAR_MINERALS_OFF_ROBOT);
        } else if ((controller1.isRightButtonDown() || controller2.isRightButtonDown()) && (
                automator.getMode() == Automator.Mode.DISENGAGE_LATCH ||
                        (controller1.isRightBumperDown() && controller1.isYButtonDown()) ||
                        (controller2.isRightBumperDown() && controller2.isYButtonDown())
        )) {
            automator.setDisengageLatchNavTargetHeading(-135.0);
            automator.setMode(Automator.Mode.DISENGAGE_LATCH);
        } else if ((controller1.isRightBumperDown() && controller1.isYButtonDown()) ||
                (controller2.isRightBumperDown() && controller2.isYButtonDown())) {
            automator.setMode(Automator.Mode.DISENGAGE_LATCH);
        } else if (controller1.isYButtonPressed() || controller2.isYButtonPressed()) {
            automator.setMode(Automator.Mode.ENGAGE_LATCH);
        } else if (automator.getMode() == Automator.Mode.ENGAGE_LATCH && !controller1.isYButtonDown() && !controller2.isYButtonDown()) {
            automator.setMode(Automator.Mode.STOP_LATCH);
        } else if (!controller1.getLeftJoystickPosition().equals(Vector2.ZERO) || !controller2.getLeftJoystickPosition().equals(Vector2.ZERO) ||
                (!automator.getMode().isPickupMinerals() && !automator.getMode().isLatch() &&
                        (controller1.isLeftTriggerDown() || controller2.isLeftTriggerDown() || controller1.isRightTriggerDown() || controller2.isRightTriggerDown()))
        ) {
            // If bucket is told to move manually, disable automation
            automator.setMode(Automator.Mode.NONE);
        } else if (controller1.isBButtonDown() || controller2.isBButtonDown()) {
            if (automator.getMode() != Automator.Mode.PICKUP_MINERALS_WITHOUT_DELIVERY &&
                    (controller1.isBButtonPressed() || controller2.isBButtonPressed())) {
                setMineralPickupPositionAtNavHeading();
            }

            automator.setMode(Automator.Mode.PICKUP_MINERALS_WITHOUT_DELIVERY);
        } else if (automator.getMode().isPickupMinerals() && (controller1.isBButtonReleased() || controller2.isBButtonReleased())) {
            automator.setMode(Automator.Mode.DELIVER_SILVER_THEN_GOLD);
        }
    }

    @Override
    public void init_loop() {
        // time <= previousTime is paranoia so deltaTime cannot be set to zero
        if (!initSuccessful || time <= previousTime) return;

        deltaTime = time - previousTime;
        previousTime = time;

        nav.update();
        bucket.update();
        latch.update();
        automator.update();
        controller1.update();
        controller2.update();

        //adjustMineralPickupPosition();
        adjustMineralDeliveryPosition();
        setAutomatorMode();
    }

    private Vector2 calculateNavTargetVelocityFromControllers() {
        return Controller.fuseControls(
                controller1.isRightJoystickDown() ? Vector2.ZERO : controller1.getRightJoystickPosition(),
                controller2.isRightJoystickDown() ? Vector2.ZERO : controller2.getRightJoystickPosition()
        ).mul(MAX_NAV_TARGET_SPEED);
    }

    private double calculateNavTargetAngularVelocityFromControllers() {
        double effectiveBucketRelativePosition;
        if (bucket.getSlidePosition() > Bucket.MIN_UNEXTENDED_SLIDE_POSITION &&
                bucket.getSlidePosition() < Bucket.MAX_UNEXTENDED_SLIDE_POSITION) {
            if (calculateBucketSlideTargetVelocityFromControllers() < 0.0) {
                effectiveBucketRelativePosition = Bucket.convertSlidePositionToRelativePosition(Bucket.MIN_UNEXTENDED_SLIDE_POSITION);
            } else {
                effectiveBucketRelativePosition = Bucket.convertSlidePositionToRelativePosition(Bucket.MAX_UNEXTENDED_SLIDE_POSITION);
            }
        } else {
            effectiveBucketRelativePosition = bucket.getRelativePosition();
        }

        Vector2 bucketTargetVelocity = Controller.fuseControls(controller1.getLeftJoystickPosition(), controller2.getLeftJoystickPosition()).mul(MAX_BUCKET_TARGET_SPEED);

        double navTargetAngularVelocityFromBucketMovement = Degrees.fromRadians(
                bucketTargetVelocity.subRotation(nav.getHeading()).getY() / effectiveBucketRelativePosition
        );

        double navTargetAngularVelocityFromTriggers = MAX_NAV_TARGET_ANGULAR_SPEED * Controller.fuseControls(
                controller1.isRightJoystickDown() ? 0.0 : controller1.getLeftTriggerPosition() - controller1.getRightTriggerPosition(),
                controller2.isRightJoystickDown() ? 0.0 : controller2.getLeftTriggerPosition() - controller2.getRightTriggerPosition()
        );

        return navTargetAngularVelocityFromBucketMovement + navTargetAngularVelocityFromTriggers;
    }

    private double calculateBucketSlideTargetVelocityFromControllers() {
        Vector2 fusedLeftJoystickPositions = Controller.fuseControls(controller1.getLeftJoystickPosition(), controller2.getLeftJoystickPosition());

        Vector2 bucketTargetRelativeVelocity = fusedLeftJoystickPositions
                .mul(MAX_BUCKET_TARGET_SPEED)
                .subRotation(nav.getHeading());

        double bucketTargetRelativeVelocityRotation = bucketTargetRelativeVelocity.getRotation();

        boolean fusedLeftJoystickAtMaxMagnitude = fusedLeftJoystickPositions.getMagnitude() >= JOYSTICK_AT_MAX_MAGNITUDE_THRESHOLD;

        if (fusedLeftJoystickAtMaxMagnitude && Degrees.between(bucketTargetRelativeVelocityRotation, 0.0) < 45.0) {
            return Double.POSITIVE_INFINITY;
        } else if (fusedLeftJoystickAtMaxMagnitude && Degrees.between(bucketTargetRelativeVelocityRotation, 180.0) < 45.0) {
            return Double.NEGATIVE_INFINITY;
        } else {
            return bucketTargetRelativeVelocity.getX();
        }
    }

    private double calculateBucketPivotTargetVelocityFromControllers() {
        return Controller.fuseControls(
                controller1.isRightJoystickDown() ? controller1.getRightJoystickPosition().getY() : 0.0,
                controller2.isRightJoystickDown() ? controller2.getRightJoystickPosition().getY() : 0.0
        );
    }

    private double calculateBucketTensionerTargetPositionFromControllers() {
        return 1.0 - Controller.fuseControls(
                controller1.isRightJoystickDown() ? controller1.getRightTriggerPosition() : 0.0,
                controller2.isRightJoystickDown() ? controller2.getRightTriggerPosition() : 0.0
        );
    }

    @Override
    public final void loop() {
        // time <= previousTime is paranoia since init_loop() will immediately return if time is not greater than previousTime
        if (!initSuccessful || time <= previousTime) return;

        double previousController1LeftTriggerPosition = controller1.getLeftTriggerPosition();
        double previousController1RightTriggerPosition = controller1.getRightTriggerPosition();
        double previousController2LeftTriggerPosition = controller2.getLeftTriggerPosition();
        double previousController2RightTriggerPosition = controller2.getRightTriggerPosition();

        init_loop();

        if (controller1.isStartButtonDown() && controller1.isLeftJoystickDown()) {
            nav.setHeading(controller1.getLeftJoystickPosition().getRotation());
            nav.setTargetVelocities(Vector2.ZERO, 0.0);
            bucket.leftSlide.setPower(0.0);
            bucket.rightSlide.setPower(0.0);
            return;
        } else if (controller2.isStartButtonDown() && controller2.isLeftJoystickDown()) {
            nav.setHeading(controller2.getLeftJoystickPosition().getRotation());
            nav.setTargetVelocities(Vector2.ZERO, 0.0);
            bucket.leftSlide.setPower(0.0);
            bucket.rightSlide.setPower(0.0);
            return;
        }

        if ((controller1.isStartButtonDown() && controller1.isYButtonPressed()) ||
                (controller2.isStartButtonDown() && controller2.isYButtonPressed())) {
            nav.switchImu();
        }

        if (automator.getMode() == Automator.Mode.NONE) {
            nav.setTargetVelocities(calculateNavTargetVelocityFromControllers(), calculateNavTargetAngularVelocityFromControllers());

            double bucketSlideTargetVelocity = calculateBucketSlideTargetVelocityFromControllers();
            if (bucketSlideTargetVelocity == Double.NEGATIVE_INFINITY) {
                bucket.leftSlide.setPower(-1.0);
                bucket.rightSlide.setPower(-1.0);
            } else if (bucketSlideTargetVelocity == Double.POSITIVE_INFINITY) {
                bucket.leftSlide.setPower(1.0);
                bucket.rightSlide.setPower(1.0);
            } else {
                bucket.leftSlide.setTargetVelocity(bucketSlideTargetVelocity);
                bucket.rightSlide.setTargetVelocity(bucketSlideTargetVelocity);
            }

            if (controller1.isRightJoystickDown() || controller2.isRightJoystickDown()) {
                if (latch.getCatchEngagementAmount() > 0.0) latch.disengage();
                else bucket.pivot.setTargetVelocity(calculateBucketPivotTargetVelocityFromControllers());

                bucket.tensioner.setTargetPosition(calculateBucketTensionerTargetPositionFromControllers());
            } else {
                bucket.setPivotTargetPosition();

                if (bucket.getSlidePosition() < Bucket.MINERAL_PICKUP_MIN_SLIDE_POSITION) {
                    bucket.tensioner.setTargetPosition(bucket.tensioner.getMaxPosition());
                } else {
                    bucket.setTensionerTargetPosition();
                }
            }
        } else if (automator.getMode() == Automator.Mode.PICKUP_MINERALS_WITHOUT_DELIVERY && bucket.getSlidePosition() > Bucket.MAX_UNEXTENDED_SLIDE_POSITION) {
            double deltaController1LeftTriggerPosition = controller1.getLeftTriggerPosition() - previousController1LeftTriggerPosition;
            double deltaController1RightTriggerPosition = controller1.getRightTriggerPosition() - previousController1RightTriggerPosition;
            double deltaController2LeftTriggerPosition = controller2.getLeftTriggerPosition() - previousController2LeftTriggerPosition;
            double deltaController2RightTriggerPosition = controller2.getRightTriggerPosition() - previousController2RightTriggerPosition;

            if (deltaController1LeftTriggerPosition == 0.0 && deltaController1RightTriggerPosition == 0.0 &&
                    deltaController2LeftTriggerPosition == 0.0 && deltaController2RightTriggerPosition == 0.0 &&
                    controller1.getLeftTriggerPosition() == 0.0 && controller1.getRightTriggerPosition() == 0.0 &&
                    controller2.getLeftTriggerPosition() == 0.0 && controller2.getRightTriggerPosition() == 0.0) {
                automator.setMineralPickupPosition(bucket.getPosition());
                nav.setTargetVelocities(calculateNavTargetVelocityFromControllers(), 0.0);
                bucket.leftSlide.setPower(0.0);
                bucket.rightSlide.setPower(0.0);
                bucket.setPivotTargetPosition();
                bucket.setTensionerTargetPosition();
            } else {
                double increaseOfController1LeftTriggerPosition = Range.clip(deltaController1LeftTriggerPosition, 0.0, 1.0);
                double increaseOfController1RightTriggerPosition = Range.clip(deltaController1RightTriggerPosition, 0.0, 1.0);
                double increaseOfController2LeftTriggerPosition = Range.clip(deltaController2LeftTriggerPosition, 0.0, 1.0);
                double increaseOfController2RightTriggerPosition = Range.clip(deltaController2RightTriggerPosition, 0.0, 1.0);

                Vector2 deltaMineralPickupPosition;
                if ((controller1.isLeftTriggerDown() && controller1.isRightTriggerDown()) || (controller2.isLeftTriggerDown() && controller2.isRightTriggerDown())) {
                    if (((controller1.isLeftTriggerPressed() || controller1.isRightTriggerPressed()) && (controller2.isLeftTriggerPressed() || controller2.isRightTriggerPressed()))) {
                        deltaMineralPickupPosition = new Vector2(
                                Controller.fuseControls(
                                        controller1.getLeftTriggerPosition() + controller1.getRightTriggerPosition(),
                                        controller2.getLeftTriggerPosition() + controller2.getRightTriggerPosition()
                                ) * MINERAL_PICKUP_POSITION_FORWARD_MOVEMENT_PER_TRIGGER_PRESS,
                                Controller.fuseControls(
                                        previousController1LeftTriggerPosition - previousController1RightTriggerPosition,
                                        previousController2LeftTriggerPosition - previousController2RightTriggerPosition
                                ) * -MINERAL_PICKUP_POSITION_SIDEWAYS_MOVEMENT_PER_TRIGGER_PRESS
                        ).addRotation(nav.getHeading());
                    } else if ((controller1.isLeftTriggerPressed() || controller1.isRightTriggerPressed()) && !(controller2.isLeftTriggerDown() && controller2.isRightTriggerDown())) {
                        deltaMineralPickupPosition = new Vector2(
                                (controller1.getLeftTriggerPosition() + controller1.getRightTriggerPosition()) * MINERAL_PICKUP_POSITION_FORWARD_MOVEMENT_PER_TRIGGER_PRESS,
                                (previousController1LeftTriggerPosition - previousController1RightTriggerPosition) * -MINERAL_PICKUP_POSITION_SIDEWAYS_MOVEMENT_PER_TRIGGER_PRESS
                        ).addRotation(nav.getHeading());
                    } else if ((controller2.isLeftTriggerPressed() || controller2.isRightTriggerPressed()) && !(controller1.isLeftTriggerDown() && controller1.isRightTriggerDown())) {
                        deltaMineralPickupPosition = new Vector2(
                                (controller2.getLeftTriggerPosition() + controller2.getRightTriggerPosition()) * MINERAL_PICKUP_POSITION_FORWARD_MOVEMENT_PER_TRIGGER_PRESS,
                                (previousController2LeftTriggerPosition - previousController2RightTriggerPosition) * -MINERAL_PICKUP_POSITION_SIDEWAYS_MOVEMENT_PER_TRIGGER_PRESS
                        ).addRotation(nav.getHeading());
                    } else {
                        deltaMineralPickupPosition = new Vector2(
                                Controller.fuseControls(
                                        increaseOfController1LeftTriggerPosition + increaseOfController1RightTriggerPosition,
                                        increaseOfController2LeftTriggerPosition + increaseOfController2RightTriggerPosition
                                ) * MINERAL_PICKUP_POSITION_FORWARD_MOVEMENT_PER_TRIGGER_PRESS,
                                0.0
                        ).addRotation(nav.getHeading());
                    }
                } else {
                    deltaMineralPickupPosition = new Vector2(
                            0.0,
                            Controller.fuseControls(
                                    increaseOfController1LeftTriggerPosition - increaseOfController1RightTriggerPosition,
                                    increaseOfController2LeftTriggerPosition - increaseOfController2RightTriggerPosition
                            ) * MINERAL_PICKUP_POSITION_SIDEWAYS_MOVEMENT_PER_TRIGGER_PRESS
                    ).addRotation(nav.getHeading());
                }

                automator.setMineralPickupPosition(automator.getMineralPickupPosition().add(deltaMineralPickupPosition));
                automator.setTargets(calculateNavTargetVelocityFromControllers(), calculateNavTargetAngularVelocityFromControllers());
            }
        } else if (automator.getMode().isPickupMinerals()) {
            if (controller1.isLeftTriggerDown() || controller1.isRightTriggerDown() || controller2.isLeftTriggerDown() || controller2.isRightTriggerDown()) {
                automator.setMineralPickupPosition(nav.getPosition().add(
                        new Vector2(Bucket.convertSlidePositionToRelativePosition(Bucket.MINERAL_PICKUP_MIN_SLIDE_POSITION), 0.0).addRotation(nav.getHeading())
                ));
            } else {
                automator.setMineralPickupPosition(nav.getPosition().add(
                        new Vector2(Bucket.convertSlidePositionToRelativePosition(Bucket.MINERAL_PICKUP_MIN_SLIDE_POSITION), 0.0)
                                .addRotation(automator.getMineralPickupPosition().getRotationFrom(nav.getPosition()))
                ));
            }

            automator.setTargets(calculateNavTargetVelocityFromControllers(), calculateNavTargetAngularVelocityFromControllers());
        } else {
            automator.setTargets(calculateNavTargetVelocityFromControllers(), calculateNavTargetAngularVelocityFromControllers());
        }
    }

    @Override
    public final void stop() {
        if (initSuccessful) {
            nav.saveImuCalibration();
        }
    }

    @Override
    public String toString() {
        return automator.toString() + nav.toString() + bucket.toString() + latch.toString() + controller1.toString() + controller2.toString();
    }
}