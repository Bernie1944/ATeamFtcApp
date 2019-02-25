package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Vector2;

public class Controller extends Component {
    // Ftc-provided class used to query input
    private final Gamepad gamepad;

    private final double joystickMagnitudeDeadzone;

    // Becomes true when buttons are pressed together and only goes to false once both buttons are up
    private boolean startPlusADown = false;
    private boolean startPlusBDown = false;

    // Between [-1, 1]
    private Vector2 leftJoystickPosition = Vector2.ZERO;
    private Vector2 rightJoystickPosition = Vector2.ZERO;
    private Vector2 dpadPosition = Vector2.ZERO;

    // Between [0, 1]
    private double leftTriggerPosition = 0.0;
    private double rightTriggerPosition = 0.0;

    private boolean leftJoystickDown = false;
    private boolean rightJoystickDown = false;
    private boolean leftTriggerDown = false;
    private boolean rightTriggerDown = false;
    private boolean leftBumperDown = false;
    private boolean rightBumperDown = false;
    private boolean leftButtonDown = false;
    private boolean rightButtonDown = false;
    private boolean downButtonDown = false;
    private boolean upButtonDown = false;
    private boolean aButtonDown = false;
    private boolean bButtonDown = false;
    private boolean xButtonDown = false;
    private boolean yButtonDown = false;
    private boolean startButtonDown = false;

    private boolean leftJoystickPressed = false;
    private boolean rightJoystickPressed = false;
    private boolean leftTriggerPressed = false;
    private boolean rightTriggerPressed = false;
    private boolean leftBumperPressed = false;
    private boolean rightBumperPressed = false;
    private boolean leftButtonPressed = false;
    private boolean rightButtonPressed = false;
    private boolean downButtonPressed = false;
    private boolean upButtonPressed = false;
    private boolean aButtonPressed = false;
    private boolean bButtonPressed = false;
    private boolean xButtonPressed = false;
    private boolean yButtonPressed = false;
    private boolean startButtonPressed = false;

    private boolean leftJoystickReleased = false;
    private boolean rightJoystickReleased = false;
    private boolean leftTriggerReleased = false;
    private boolean rightTriggerReleased = false;
    private boolean leftBumperReleased = false;
    private boolean rightBumperReleased = false;
    private boolean leftButtonReleased = false;
    private boolean rightButtonReleased = false;
    private boolean downButtonReleased = false;
    private boolean upButtonReleased = false;
    private boolean aButtonReleased = false;
    private boolean bButtonReleased = false;
    private boolean xButtonReleased = false;
    private boolean yButtonReleased = false;
    private boolean startButtonReleased = false;

    private boolean leftJoystickToggleOn = false;
    private boolean rightJoystickToggleOn = false;
    private boolean leftTriggerToggleOn = false;
    private boolean rightTriggerToggleOn = false;
    private boolean leftBumperToggleOn = false;
    private boolean rightBumperToggleOn = false;
    private boolean leftButtonToggleOn = false;
    private boolean rightButtonToggleOn = false;
    private boolean downButtonToggleOn = false;
    private boolean upButtonToggleOn = false;
    private boolean aButtonToggleOn = false;
    private boolean bButtonToggleOn = false;
    private boolean xButtonToggleOn = false;
    private boolean yButtonToggleOn = false;
    private boolean startButtonToggleOn = false;

    private boolean leftJoystickToggleActivated = false;
    private boolean rightJoystickToggleActivated = false;
    private boolean leftTriggerToggleActivated = false;
    private boolean rightTriggerToggleActivated = false;
    private boolean leftBumperToggleActivated = false;
    private boolean rightBumperToggleActivated = false;
    private boolean leftButtonToggleActivated = false;
    private boolean rightButtonToggleActivated = false;
    private boolean downButtonToggleActivated = false;
    private boolean upButtonToggleActivated = false;
    private boolean aButtonToggleActivated = false;
    private boolean bButtonToggleActivated = false;
    private boolean xButtonToggleActivated = false;
    private boolean yButtonToggleActivated = false;
    private boolean startButtonToggleActivated = false;

    private boolean leftJoystickToggleDeactivated = false;
    private boolean rightJoystickToggleDeactivated = false;
    private boolean leftTriggerToggleDeactivated = false;
    private boolean rightTriggerToggleDeactivated = false;
    private boolean leftBumperToggleDeactivated = false;
    private boolean rightBumperToggleDeactivated = false;
    private boolean leftButtonToggleDeactivated = false;
    private boolean rightButtonToggleDeactivated = false;
    private boolean downButtonToggleDeactivated = false;
    private boolean upButtonToggleDeactivated = false;
    private boolean aButtonToggleDeactivated = false;
    private boolean bButtonToggleDeactivated = false;
    private boolean xButtonToggleDeactivated = false;
    private boolean yButtonToggleDeactivated = false;
    private boolean startButtonToggleDeactivated = false;

    // When using an Xbox 360 controller configured as a Logitech controller under the Driver Station app's Settings,
    // joystickMagnitudeDeadzone should be set to a value greater than 0.0 (0.13 - 0.14 seems to work well),
    // which will result in more "smooth" joystick controls that do not seem to "stick" to the x and y axis (this, however, is not preferable in all cases)
    public Controller(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad, double joystickMagnitudeDeadzone) {
        super(telemetry, hardwareMap);
        this.gamepad = gamepad;
        this.joystickMagnitudeDeadzone = joystickMagnitudeDeadzone;
    }

    public Vector2 getLeftJoystickPosition() {
        return leftJoystickPosition;
    }

    public Vector2 getRightJoystickPosition() {
        return rightJoystickPosition;
    }

    public Vector2 getDpadPosition() {
        return dpadPosition;
    }

    public double getLeftTriggerPosition() {
        return leftTriggerPosition;
    }

    public double getRightTriggerPosition() {
        return rightTriggerPosition;
    }

    public boolean isLeftJoystickDown() {
        return leftJoystickDown;
    }

    public boolean isRightJoystickDown() {
        return rightJoystickDown;
    }

    public boolean isLeftTriggerDown() {
        return leftTriggerDown;
    }

    public boolean isRightTriggerDown() {
        return rightTriggerDown;
    }

    public boolean isLeftBumperDown() {
        return leftBumperDown;
    }

    public boolean isRightBumperDown() {
        return rightBumperDown;
    }

    public boolean isLeftButtonDown() {
        return leftButtonDown;
    }

    public boolean isRightButtonDown() {
        return rightButtonDown;
    }

    public boolean isDownButtonDown() {
        return downButtonDown;
    }

    public boolean isUpButtonDown() {
        return upButtonDown;
    }

    public boolean isAButtonDown() {
        return aButtonDown;
    }

    public boolean isBButtonDown() {
        return bButtonDown;
    }

    public boolean isXButtonDown() {
        return xButtonDown;
    }

    public boolean isYButtonDown() {
        return yButtonDown;
    }

    public boolean isStartButtonDown() {
        return startButtonDown;
    }

    public boolean isLeftJoystickPressed() {
        return leftJoystickPressed;
    }

    public boolean isRightJoystickPressed() {
        return rightJoystickPressed;
    }

    public boolean isLeftTriggerPressed() {
        return leftTriggerPressed;
    }

    public boolean isRightTriggerPressed() {
        return rightTriggerPressed;
    }

    public boolean isLeftBumperPressed() {
        return leftBumperPressed;
    }

    public boolean isRightBumperPressed() {
        return rightBumperPressed;
    }

    public boolean isLeftButtonPressed() {
        return leftButtonPressed;
    }

    public boolean isRightButtonPressed() {
        return rightButtonPressed;
    }

    public boolean isDownButtonPressed() {
        return downButtonPressed;
    }

    public boolean isUpButtonPressed() {
        return upButtonPressed;
    }

    public boolean isAButtonPressed() {
        return aButtonPressed;
    }

    public boolean isBButtonPressed() {
        return bButtonPressed;
    }

    public boolean isXButtonPressed() {
        return xButtonPressed;
    }

    public boolean isYButtonPressed() {
        return yButtonPressed;
    }

    public boolean isStartButtonPressed() {
        return startButtonPressed;
    }

    public boolean isLeftJoystickReleased() {
        return leftJoystickReleased;
    }

    public boolean isRightJoystickReleased() {
        return rightJoystickReleased;
    }

    public boolean isLeftTriggerReleased() {
        return leftTriggerReleased;
    }

    public boolean isRightTriggerReleased() {
        return rightTriggerReleased;
    }

    public boolean isLeftBumperReleased() {
        return leftBumperReleased;
    }

    public boolean isRightBumperReleased() {
        return rightBumperReleased;
    }

    public boolean isLeftButtonReleased() {
        return leftButtonReleased;
    }

    public boolean isRightButtonReleased() {
        return rightButtonReleased;
    }

    public boolean isDownButtonReleased() {
        return downButtonReleased;
    }

    public boolean isUpButtonReleased() {
        return upButtonReleased;
    }

    public boolean isAButtonReleased() {
        return aButtonReleased;
    }

    public boolean isBButtonReleased() {
        return bButtonReleased;
    }

    public boolean isXButtonReleased() {
        return xButtonReleased;
    }

    public boolean isYButtonReleased() {
        return yButtonReleased;
    }

    public boolean isStartButtonReleased() {
        return startButtonReleased;
    }

    public boolean isLeftJoystickToggleOn() {
        return leftJoystickToggleOn;
    }

    public boolean isRightJoystickToggleOn() {
        return rightJoystickToggleOn;
    }

    public boolean isLeftTriggerToggleOn() {
        return leftTriggerToggleOn;
    }

    public boolean isRightTriggerToggleOn() {
        return rightTriggerToggleOn;
    }

    public boolean isLeftBumperToggleOn() {
        return leftBumperToggleOn;
    }

    public boolean isRightBumperToggleOn() {
        return rightBumperToggleOn;
    }

    public boolean isLeftButtonToggleOn() {
        return leftButtonToggleOn;
    }

    public boolean isRightButtonToggleOn() {
        return rightButtonToggleOn;
    }

    public boolean isDownButtonToggleOn() {
        return downButtonToggleOn;
    }

    public boolean isUpButtonToggleOn() {
        return upButtonToggleOn;
    }

    public boolean isAButtonToggleOn() {
        return aButtonToggleOn;
    }

    public boolean isBButtonToggleOn() {
        return bButtonToggleOn;
    }

    public boolean isXButtonToggleOn() {
        return xButtonToggleOn;
    }

    public boolean isYButtonToggleOn() {
        return yButtonToggleOn;
    }

    public boolean isStartButtonToggleOn() {
        return startButtonToggleOn;
    }

    public boolean isLeftJoystickToggleActivated() {
        return leftJoystickToggleActivated;
    }

    public boolean isRightJoystickToggleActivated() {
        return rightJoystickToggleActivated;
    }

    public boolean isLeftTriggerToggleActivated() {
        return leftTriggerToggleActivated;
    }

    public boolean isRightTriggerToggleActivated() {
        return rightTriggerToggleActivated;
    }

    public boolean isLeftBumperToggleActivated() {
        return leftBumperToggleActivated;
    }

    public boolean isRightBumperToggleActivated() {
        return rightBumperToggleActivated;
    }

    public boolean isLeftButtonToggleActivated() {
        return leftButtonToggleActivated;
    }

    public boolean isRightButtonToggleActivated() {
        return rightButtonToggleActivated;
    }

    public boolean isDownButtonToggleActivated() {
        return downButtonToggleActivated;
    }

    public boolean isUpButtonToggleActivated() {
        return upButtonToggleActivated;
    }

    public boolean isAButtonToggleActivated() {
        return aButtonToggleActivated;
    }

    public boolean isBButtonToggleActivated() {
        return bButtonToggleActivated;
    }

    public boolean isXButtonToggleActivated() {
        return xButtonToggleActivated;
    }

    public boolean isYButtonToggleActivated() {
        return yButtonToggleActivated;
    }

    public boolean isStartButtonToggleActivated() {
        return startButtonToggleActivated;
    }

    public boolean isLeftJoystickToggleDeactivated() {
        return leftJoystickToggleDeactivated;
    }

    public boolean isRightJoystickToggleDeactivated() {
        return rightJoystickToggleDeactivated;
    }

    public boolean isLeftTriggerToggleDeactivated() {
        return leftTriggerToggleDeactivated;
    }

    public boolean isRightTriggerToggleDeactivated() {
        return rightTriggerToggleDeactivated;
    }

    public boolean isLeftBumperToggleDeactivated() {
        return leftBumperToggleDeactivated;
    }

    public boolean isRightBumperToggleDeactivated() {
        return rightBumperToggleDeactivated;
    }

    public boolean isLeftButtonToggleDeactivated() {
        return leftButtonToggleDeactivated;
    }

    public boolean isRightButtonToggleDeactivated() {
        return rightButtonToggleDeactivated;
    }

    public boolean isDownButtonToggleDeactivated() {
        return downButtonToggleDeactivated;
    }

    public boolean isUpButtonToggleDeactivated() {
        return upButtonToggleDeactivated;
    }

    public boolean isAButtonToggleDeactivated() {
        return aButtonToggleDeactivated;
    }

    public boolean isBButtonToggleDeactivated() {
        return bButtonToggleDeactivated;
    }

    public boolean isXButtonToggleDeactivated() {
        return xButtonToggleDeactivated;
    }

    public boolean isYButtonToggleDeactivated() {
        return yButtonToggleDeactivated;
    }

    public boolean isStartButtonToggleDeactivated() {
        return startButtonToggleDeactivated;
    }

    // Called through Component.update()
    @Override
    void updateImpl() {
        if (gamepad.start && gamepad.a) startPlusADown = true;
        else if (!gamepad.start && !gamepad.a) startPlusADown = false;

        if (gamepad.start && gamepad.b) startPlusBDown = true;
        else if (!gamepad.start && !gamepad.b) startPlusBDown = false;

        // gamepad.left_stick_y is inverted
        leftJoystickPosition = new Vector2(gamepad.left_stick_x, -gamepad.left_stick_y);
        leftJoystickPosition = leftJoystickPosition.withMagnitude(
                Range.clip(Range.scale(leftJoystickPosition.getMagnitude(), joystickMagnitudeDeadzone, 1.0, 0.0, 1.0), 0.0, 1.0)
        );

        // gamepad.right_stick_y is inverted
        rightJoystickPosition = new Vector2(gamepad.right_stick_x, -gamepad.right_stick_y);
        rightJoystickPosition = rightJoystickPosition.withMagnitude(
                Range.clip(Range.scale(rightJoystickPosition.getMagnitude(), joystickMagnitudeDeadzone, 1.0, 0.0, 1.0), 0.0, 1.0)
        );

        dpadPosition = new Vector2((gamepad.dpad_right ? 1.0 : 0.0) - (gamepad.dpad_left ? 1.0 : 0.0), (gamepad.dpad_up ? 1.0 : 0.0) - (gamepad.dpad_down ? 1.0 : 0.0));
        if (dpadPosition.getMagnitude() > 1.0) {
            dpadPosition = dpadPosition.withMagnitude(1.0);
        }

        leftTriggerPosition = gamepad.left_trigger;
        rightTriggerPosition = gamepad.right_trigger;

        leftJoystickPressed = gamepad.left_stick_button && !leftJoystickDown;
        leftJoystickReleased = leftJoystickDown && !gamepad.left_stick_button;
        leftJoystickDown = gamepad.left_stick_button;
        if (leftJoystickPressed) leftJoystickToggleOn = !leftJoystickToggleOn;
        leftJoystickToggleActivated = leftJoystickPressed && leftJoystickToggleOn;
        leftJoystickToggleDeactivated = leftJoystickPressed && !leftJoystickToggleOn;

        rightJoystickPressed = gamepad.right_stick_button && !rightJoystickDown;
        rightJoystickReleased = rightJoystickDown && !gamepad.right_stick_button;
        rightJoystickDown = gamepad.right_stick_button;
        if (rightJoystickPressed) rightJoystickToggleOn = !rightJoystickToggleOn;
        rightJoystickToggleActivated = rightJoystickPressed && rightJoystickToggleOn;
        rightJoystickToggleDeactivated = rightJoystickPressed && !rightJoystickToggleOn;

        leftTriggerPressed = gamepad.left_trigger != 0.0 && !leftTriggerDown;
        leftTriggerReleased = leftTriggerDown && gamepad.left_trigger == 0.0;
        leftTriggerDown = gamepad.left_trigger != 0.0;
        if (leftTriggerPressed) leftTriggerToggleOn = !leftTriggerToggleOn;
        leftTriggerToggleActivated = leftTriggerPressed && leftTriggerToggleOn;
        leftTriggerToggleDeactivated = leftTriggerPressed && !leftTriggerToggleOn;

        rightTriggerPressed = gamepad.right_trigger != 0.0 && !rightTriggerDown;
        rightTriggerReleased = rightTriggerDown && gamepad.right_trigger == 0.0;
        rightTriggerDown = gamepad.right_trigger != 0.0;
        if (rightTriggerPressed) rightTriggerToggleOn = !rightTriggerToggleOn;
        rightTriggerToggleActivated = rightTriggerPressed && rightTriggerToggleOn;
        rightTriggerToggleDeactivated = rightTriggerPressed && !rightTriggerToggleOn;

        leftBumperPressed = gamepad.left_bumper && !leftBumperDown;
        leftBumperReleased = leftBumperDown && !gamepad.left_bumper;
        leftBumperDown = gamepad.left_bumper;
        if (leftBumperPressed) leftBumperToggleOn = !leftBumperToggleOn;
        leftBumperToggleActivated = leftBumperPressed && leftBumperToggleOn;
        leftBumperToggleDeactivated = leftBumperPressed && !leftBumperToggleOn;

        rightBumperPressed = gamepad.right_bumper && !rightBumperDown;
        rightBumperReleased = rightBumperDown && !gamepad.right_bumper;
        rightBumperDown = gamepad.right_bumper;
        if (rightBumperPressed) rightBumperToggleOn = !rightBumperToggleOn;
        rightBumperToggleActivated = rightBumperPressed && rightBumperToggleOn;
        rightBumperToggleDeactivated = rightBumperPressed && !rightBumperToggleOn;

        leftButtonPressed = gamepad.dpad_left && !leftButtonDown;
        leftButtonReleased = leftButtonDown && !gamepad.dpad_left;
        leftButtonDown = gamepad.dpad_left;
        if (leftButtonPressed) leftButtonToggleOn = !leftButtonToggleOn;
        leftButtonToggleActivated = leftButtonPressed && leftButtonToggleOn;
        leftButtonToggleDeactivated = leftButtonPressed && !leftButtonToggleOn;

        rightButtonPressed = gamepad.dpad_right && !rightButtonDown;
        rightButtonReleased = rightButtonDown && !gamepad.dpad_right;
        rightButtonDown = gamepad.dpad_right;
        if (rightButtonPressed) rightButtonToggleOn = !rightButtonToggleOn;
        rightButtonToggleActivated = rightButtonPressed && rightButtonToggleOn;
        rightButtonToggleDeactivated = rightButtonPressed && !rightButtonToggleOn;

        downButtonPressed = gamepad.dpad_down && !downButtonDown;
        downButtonReleased = downButtonDown && !gamepad.dpad_down;
        downButtonDown = gamepad.dpad_down;
        if (downButtonPressed) downButtonToggleOn = !downButtonToggleOn;
        downButtonToggleActivated = downButtonPressed && downButtonToggleOn;
        downButtonToggleDeactivated = downButtonPressed && !downButtonToggleOn;

        upButtonPressed = gamepad.dpad_up && !upButtonDown;
        upButtonReleased = upButtonDown && !gamepad.dpad_up;
        upButtonDown = gamepad.dpad_up;
        if (upButtonPressed) upButtonToggleOn = !upButtonToggleOn;
        upButtonToggleActivated = upButtonPressed && upButtonToggleOn;
        upButtonToggleDeactivated = upButtonPressed && !upButtonToggleOn;

        aButtonPressed = gamepad.a && !aButtonDown && !startPlusADown;
        aButtonReleased = aButtonDown && !gamepad.a;
        aButtonDown = gamepad.a && !startPlusADown;
        if (aButtonPressed) aButtonToggleOn = !aButtonToggleOn;
        aButtonToggleActivated = aButtonPressed && aButtonToggleOn;
        aButtonToggleDeactivated = aButtonPressed && !aButtonToggleOn;

        bButtonPressed = gamepad.b && !bButtonDown && !startPlusBDown;
        bButtonReleased = bButtonDown && !gamepad.b;
        bButtonDown = gamepad.b && !startPlusBDown;
        if (bButtonPressed) bButtonToggleOn = !bButtonToggleOn;
        bButtonToggleActivated = bButtonPressed && bButtonToggleOn;
        bButtonToggleDeactivated = bButtonPressed && !bButtonToggleOn;

        xButtonPressed = gamepad.x && !xButtonDown;
        xButtonReleased = xButtonDown && !gamepad.x;
        xButtonDown = gamepad.x;
        if (xButtonPressed) xButtonToggleOn = !xButtonToggleOn;
        xButtonToggleActivated = xButtonPressed && xButtonToggleOn;
        xButtonToggleDeactivated = xButtonPressed && !xButtonToggleOn;

        yButtonPressed = gamepad.y && !yButtonDown;
        yButtonReleased = yButtonDown && !gamepad.y;
        yButtonDown = gamepad.y;
        if (yButtonPressed) yButtonToggleOn = !yButtonToggleOn;
        yButtonToggleActivated = yButtonPressed && yButtonToggleOn;
        yButtonToggleDeactivated = yButtonPressed && !yButtonToggleOn;

        startButtonPressed = gamepad.start && !startButtonDown;
        startButtonReleased = startButtonDown && !gamepad.start;
        startButtonDown = gamepad.start;
        if (startButtonPressed) startButtonToggleOn = !startButtonToggleOn;
        startButtonToggleActivated = startButtonPressed && startButtonToggleOn;
        startButtonToggleDeactivated = startButtonPressed && !startButtonToggleOn;
    }

    // Returns text describing state
    @Override
    public String toString() {
        return "leftJoystickPosition : " + String.format(
                "(%5.2f, %5.2f) (%5.2f @ %6.1f°)",
                getLeftJoystickPosition().getX(), getLeftJoystickPosition().getY(), getLeftJoystickPosition().getMagnitude(), getLeftJoystickPosition().getRotation()
        ) + "\n" +
                "rightJoystickPosition : " + String.format(
                "(%5.2f, %5.2f) (%5.2f @ %6.1f°)",
                getRightJoystickPosition().getX(), getRightJoystickPosition().getY(), getRightJoystickPosition().getMagnitude(), getRightJoystickPosition().getRotation()
        ) + "\n" +
                "dpadPosition : " + String.format(
                "(%5.2f, %5.2f) (%5.2f @ %6.1f°)",
                getDpadPosition().getX(), getDpadPosition().getY(), getDpadPosition().getMagnitude(), getDpadPosition().getRotation()
        ) + "\n" +
                "leftTriggerPosition : " + String.format("%4.2f", getLeftTriggerPosition()) + "\n" +
                "rightJoystickPosition : " + String.format("%4.2f", getRightTriggerPosition()) + "\n" +
                "leftJoystickDown : " + Boolean.toString(isLeftJoystickDown()) + "\n" +
                "rightJoystickDown : " + Boolean.toString(isRightJoystickDown()) + "\n" +
                "leftTriggerDown : " + Boolean.toString(isLeftTriggerDown()) + "\n" +
                "rightTriggerDown : " + Boolean.toString(isRightTriggerDown()) + "\n" +
                "leftBumperDown : " + Boolean.toString(isLeftBumperDown()) + "\n" +
                "rightBumperDown : " + Boolean.toString(isRightBumperDown()) + "\n" +
                "leftButtonDown : " + Boolean.toString(isLeftButtonDown()) + "\n" +
                "rightButtonDown : " + Boolean.toString(isRightButtonDown()) + "\n" +
                "downButtonDown : " + Boolean.toString(isDownButtonDown()) + "\n" +
                "upButtonDown : " + Boolean.toString(isUpButtonDown()) + "\n" +
                "aButtonDown : " + Boolean.toString(isAButtonDown()) + "\n" +
                "bButtonDown : " + Boolean.toString(isBButtonDown()) + "\n" +
                "xButtonDown : " + Boolean.toString(isXButtonDown()) + "\n" +
                "yButtonDown : " + Boolean.toString(isYButtonDown()) + "\n" +
                "startButtonDown : " + Boolean.toString(isStartButtonDown()) + "\n" +
                "leftJoystickToggleOn : " + Boolean.toString(isLeftJoystickToggleOn()) + "\n" +
                "rightJoystickToggleOn : " + Boolean.toString(isRightJoystickToggleOn()) + "\n" +
                "leftTriggerToggleOn : " + Boolean.toString(isLeftTriggerToggleOn()) + "\n" +
                "rightTriggerToggleOn : " + Boolean.toString(isRightTriggerToggleOn()) + "\n" +
                "leftBumperToggleOn : " + Boolean.toString(isLeftBumperToggleOn()) + "\n" +
                "rightBumperToggleOn : " + Boolean.toString(isRightBumperToggleOn()) + "\n" +
                "leftButtonToggleOn : " + Boolean.toString(isLeftButtonToggleOn()) + "\n" +
                "rightButtonToggleOn : " + Boolean.toString(isRightButtonToggleOn()) + "\n" +
                "downButtonToggleOn : " + Boolean.toString(isDownButtonToggleOn()) + "\n" +
                "upButtonToggleOn : " + Boolean.toString(isUpButtonToggleOn()) + "\n" +
                "aButtonToggleOn : " + Boolean.toString(isAButtonToggleOn()) + "\n" +
                "bButtonToggleOn : " + Boolean.toString(isBButtonToggleOn()) + "\n" +
                "xButtonToggleOn : " + Boolean.toString(isXButtonToggleOn()) + "\n" +
                "yButtonToggleOn : " + Boolean.toString(isYButtonToggleOn()) + "\n" +
                "startButtonToggleOn : " + Boolean.toString(isStartButtonToggleOn());
    }

    // Returns text verbosely describing state
    @Override
    public String toStringVerbose() {
        return toString() + "\n" +
                "leftJoystickPressed : " + Boolean.toString(isLeftJoystickPressed()) + "\n" +
                "rightJoystickPressed : " + Boolean.toString(isRightJoystickPressed()) + "\n" +
                "leftTriggerPressed : " + Boolean.toString(isLeftTriggerPressed()) + "\n" +
                "rightTriggerPressed : " + Boolean.toString(isRightTriggerPressed()) + "\n" +
                "leftBumperPressed : " + Boolean.toString(isLeftBumperPressed()) + "\n" +
                "rightBumperPressed : " + Boolean.toString(isRightBumperPressed()) + "\n" +
                "leftButtonPressed : " + Boolean.toString(isLeftButtonPressed()) + "\n" +
                "rightButtonPressed : " + Boolean.toString(isRightButtonPressed()) + "\n" +
                "downButtonPressed : " + Boolean.toString(isDownButtonPressed()) + "\n" +
                "upButtonPressed : " + Boolean.toString(isUpButtonPressed()) + "\n" +
                "aButtonPressed : " + Boolean.toString(isAButtonPressed()) + "\n" +
                "bButtonPressed : " + Boolean.toString(isBButtonPressed()) + "\n" +
                "xButtonPressed : " + Boolean.toString(isXButtonPressed()) + "\n" +
                "yButtonPressed : " + Boolean.toString(isYButtonPressed()) + "\n" +
                "startButtonPressed : " + Boolean.toString(isStartButtonPressed()) + "\n" +
                "leftJoystickReleased : " + Boolean.toString(isLeftJoystickReleased()) + "\n" +
                "rightJoystickReleased : " + Boolean.toString(isRightJoystickReleased()) + "\n" +
                "leftTriggerReleased : " + Boolean.toString(isLeftTriggerReleased()) + "\n" +
                "rightTriggerReleased : " + Boolean.toString(isRightTriggerReleased()) + "\n" +
                "leftBumperReleased : " + Boolean.toString(isLeftBumperReleased()) + "\n" +
                "rightBumperReleased : " + Boolean.toString(isRightBumperReleased()) + "\n" +
                "leftButtonReleased : " + Boolean.toString(isLeftButtonReleased()) + "\n" +
                "rightButtonReleased : " + Boolean.toString(isRightButtonReleased()) + "\n" +
                "downButtonReleased : " + Boolean.toString(isDownButtonReleased()) + "\n" +
                "upButtonReleased : " + Boolean.toString(isUpButtonReleased()) + "\n" +
                "aButtonReleased : " + Boolean.toString(isAButtonReleased()) + "\n" +
                "bButtonReleased : " + Boolean.toString(isBButtonReleased()) + "\n" +
                "xButtonReleased : " + Boolean.toString(isXButtonReleased()) + "\n" +
                "yButtonReleased : " + Boolean.toString(isYButtonReleased()) + "\n" +
                "startButtonReleased : " + Boolean.toString(isStartButtonReleased()) + "\n" +
                "leftJoystickToggleActivated : " + Boolean.toString(isLeftJoystickToggleActivated()) + "\n" +
                "rightJoystickToggleActivated : " + Boolean.toString(isRightJoystickToggleActivated()) + "\n" +
                "leftTriggerToggleActivated : " + Boolean.toString(isLeftTriggerToggleActivated()) + "\n" +
                "rightTriggerToggleActivated : " + Boolean.toString(isRightTriggerToggleActivated()) + "\n" +
                "leftBumperToggleActivated : " + Boolean.toString(isLeftBumperToggleActivated()) + "\n" +
                "rightBumperToggleActivated : " + Boolean.toString(isRightBumperToggleActivated()) + "\n" +
                "leftButtonToggleActivated : " + Boolean.toString(isLeftButtonToggleActivated()) + "\n" +
                "rightButtonToggleActivated : " + Boolean.toString(isRightButtonToggleActivated()) + "\n" +
                "downButtonToggleActivated : " + Boolean.toString(isDownButtonToggleActivated()) + "\n" +
                "upButtonToggleActivated : " + Boolean.toString(isUpButtonToggleActivated()) + "\n" +
                "aButtonToggleActivated : " + Boolean.toString(isAButtonToggleActivated()) + "\n" +
                "bButtonToggleActivated : " + Boolean.toString(isBButtonToggleActivated()) + "\n" +
                "xButtonToggleActivated : " + Boolean.toString(isXButtonToggleActivated()) + "\n" +
                "yButtonToggleActivated : " + Boolean.toString(isYButtonToggleActivated()) + "\n" +
                "startButtonToggleActivated : " + Boolean.toString(isStartButtonToggleActivated()) + "\n" +
                "leftJoystickToggleDeactivated : " + Boolean.toString(isLeftJoystickToggleDeactivated()) + "\n" +
                "rightJoystickToggleDeactivated : " + Boolean.toString(isRightJoystickToggleDeactivated()) + "\n" +
                "leftTriggerToggleDeactivated : " + Boolean.toString(isLeftTriggerToggleDeactivated()) + "\n" +
                "rightTriggerToggleDeactivated : " + Boolean.toString(isRightTriggerToggleDeactivated()) + "\n" +
                "leftBumperToggleDeactivated : " + Boolean.toString(isLeftBumperToggleDeactivated()) + "\n" +
                "rightBumperToggleDeactivated : " + Boolean.toString(isRightBumperToggleDeactivated()) + "\n" +
                "leftButtonToggleDeactivated : " + Boolean.toString(isLeftButtonToggleDeactivated()) + "\n" +
                "rightButtonToggleDeactivated : " + Boolean.toString(isRightButtonToggleDeactivated()) + "\n" +
                "downButtonToggleDeactivated : " + Boolean.toString(isDownButtonToggleDeactivated()) + "\n" +
                "upButtonToggleDeactivated : " + Boolean.toString(isUpButtonToggleDeactivated()) + "\n" +
                "aButtonToggleDeactivated : " + Boolean.toString(isAButtonToggleDeactivated()) + "\n" +
                "bButtonToggleDeactivated : " + Boolean.toString(isBButtonToggleDeactivated()) + "\n" +
                "xButtonToggleDeactivated : " + Boolean.toString(isXButtonToggleDeactivated()) + "\n" +
                "yButtonToggleDeactivated : " + Boolean.toString(isYButtonToggleDeactivated()) + "\n" +
                "startButtonToggleDeactivated : " + Boolean.toString(isStartButtonToggleDeactivated()) + "\n" +
                "startPlusADown : " + Boolean.toString(startPlusADown) + "\n" +
                "startPlusBDown : " + Boolean.toString(startPlusBDown);
    }
}
