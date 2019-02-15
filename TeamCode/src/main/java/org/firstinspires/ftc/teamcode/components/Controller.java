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

    // Between [-1, 1]
    private Vector2 leftJoystickPosition = Vector2.ZERO;
    private Vector2 rightJoystickPosition = Vector2.ZERO;
    private Vector2 dpadPosition = Vector2.ZERO;

    // Between [0, 1]
    private double leftTriggerPosition = 0.0;
    private double rightTriggerPosition = 0.0;

    private boolean leftStickDown = false;
    private boolean rightStickDown = false;
    private boolean leftTriggerDown = false;
    private boolean rightTriggerDown = false;
    private boolean leftBumperDown = false;
    private boolean rightBumperDown = false;
    private boolean aButtonDown = false;
    private boolean bButtonDown = false;
    private boolean xButtonDown = false;
    private boolean yButtonDown = false;
    private boolean backButtonDown = false;
    private boolean startButtonDown = false;

    private boolean leftStickPressed = false;
    private boolean rightStickPressed = false;
    private boolean leftTriggerPressed = false;
    private boolean rightTriggerPressed = false;
    private boolean leftBumperPressed = false;
    private boolean rightBumperPressed = false;
    private boolean aButtonPressed = false;
    private boolean bButtonPressed = false;
    private boolean xButtonPressed = false;
    private boolean yButtonPressed = false;
    private boolean backButtonPressed = false;
    private boolean startButtonPressed = false;

    private boolean leftStickReleased = false;
    private boolean rightStickReleased = false;
    private boolean leftTriggerReleased = false;
    private boolean rightTriggerReleased = false;
    private boolean leftBumperReleased = false;
    private boolean rightBumperReleased = false;
    private boolean aButtonReleased = false;
    private boolean bButtonReleased = false;
    private boolean xButtonReleased = false;
    private boolean yButtonReleased = false;
    private boolean backButtonReleased = false;
    private boolean startButtonReleased = false;

    private boolean leftStickToggleOn = false;
    private boolean rightStickToggleOn = false;
    private boolean leftTriggerToggleOn = false;
    private boolean rightTriggerToggleOn = false;
    private boolean leftBumperToggleOn = false;
    private boolean rightBumperToggleOn = false;
    private boolean aButtonToggleOn = false;
    private boolean bButtonToggleOn = false;
    private boolean xButtonToggleOn = false;
    private boolean yButtonToggleOn = false;
    private boolean backButtonToggleOn = false;
    private boolean startButtonToggleOn = false;

    private boolean leftStickToggleActivated = false;
    private boolean rightStickToggleActivated = false;
    private boolean leftTriggerToggleActivated = false;
    private boolean rightTriggerToggleActivated = false;
    private boolean leftBumperToggleActivated = false;
    private boolean rightBumperToggleActivated = false;
    private boolean aButtonToggleActivated = false;
    private boolean bButtonToggleActivated = false;
    private boolean xButtonToggleActivated = false;
    private boolean yButtonToggleActivated = false;
    private boolean backButtonToggleActivated = false;
    private boolean startButtonToggleActivated = false;

    private boolean leftStickToggleDeactivated = false;
    private boolean rightStickToggleDeactivated = false;
    private boolean leftTriggerToggleDeactivated = false;
    private boolean rightTriggerToggleDeactivated = false;
    private boolean leftBumperToggleDeactivated = false;
    private boolean rightBumperToggleDeactivated = false;
    private boolean aButtonToggleDeactivated = false;
    private boolean bButtonToggleDeactivated = false;
    private boolean xButtonToggleDeactivated = false;
    private boolean yButtonToggleDeactivated = false;
    private boolean backButtonToggleDeactivated = false;
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

    public boolean isLeftStickDown() {
        return leftStickDown;
    }

    public boolean isRightStickDown() {
        return rightStickDown;
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

    public boolean isBackButtonDown() {
        return backButtonDown;
    }

    public boolean isStartButtonDown() {
        return startButtonDown;
    }

    public boolean isLeftStickPressed() {
        return leftStickPressed;
    }

    public boolean isRightStickPressed() {
        return rightStickPressed;
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

    public boolean isBackButtonPressed() {
        return backButtonPressed;
    }

    public boolean isStartButtonPressed() {
        return startButtonPressed;
    }

    public boolean isLeftStickReleased() {
        return leftStickReleased;
    }

    public boolean isRightStickReleased() {
        return rightStickReleased;
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

    public boolean isBackButtonReleased() {
        return backButtonReleased;
    }

    public boolean isStartButtonReleased() {
        return startButtonReleased;
    }

    public boolean isLeftStickToggleOn() {
        return leftStickToggleOn;
    }

    public boolean isRightStickToggleOn() {
        return rightStickToggleOn;
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

    public boolean isBackButtonToggleOn() {
        return backButtonToggleOn;
    }

    public boolean isStartButtonToggleOn() {
        return startButtonToggleOn;
    }

    public boolean isLeftStickToggleActivated() {
        return leftStickToggleActivated;
    }

    public boolean isRightStickToggleActivated() {
        return rightStickToggleActivated;
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

    public boolean isBackButtonToggleActivated() {
        return backButtonToggleActivated;
    }

    public boolean isStartButtonToggleActivated() {
        return startButtonToggleActivated;
    }

    public boolean isLeftStickToggleDeactivated() {
        return leftStickToggleDeactivated;
    }

    public boolean isRightStickToggleDeactivated() {
        return rightStickToggleDeactivated;
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

    public boolean isBackButtonToggleDeactivated() {
        return backButtonToggleDeactivated;
    }

    public boolean isStartButtonToggleDeactivated() {
        return startButtonToggleDeactivated;
    }

    // Called through Component.update()
    @Override
    void updateImpl() {
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

        leftStickPressed = gamepad.left_stick_button && !leftStickDown;
        leftStickReleased = leftStickDown && !gamepad.left_stick_button;
        leftStickDown = gamepad.left_stick_button;
        if (leftStickPressed) leftStickToggleOn = !leftStickToggleOn;
        leftStickToggleActivated = leftStickPressed && leftStickToggleOn;
        leftStickToggleDeactivated = leftStickPressed && !leftStickToggleOn;

        rightStickPressed = gamepad.right_stick_button && !rightStickDown;
        rightStickReleased = rightStickDown && !gamepad.right_stick_button;
        rightStickDown = gamepad.right_stick_button;
        if (rightStickPressed) rightStickToggleOn = !rightStickToggleOn;
        rightStickToggleActivated = rightStickPressed && rightStickToggleOn;
        rightStickToggleDeactivated = rightStickPressed && !rightStickToggleOn;

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

        aButtonPressed = gamepad.a && !aButtonDown;
        aButtonReleased = aButtonDown && !gamepad.a;
        aButtonDown = gamepad.a;
        if (aButtonPressed) aButtonToggleOn = !aButtonToggleOn;
        aButtonToggleActivated = aButtonPressed && aButtonToggleOn;
        aButtonToggleDeactivated = aButtonPressed && !aButtonToggleOn;

        bButtonPressed = gamepad.b && !bButtonDown;
        bButtonReleased = bButtonDown && !gamepad.b;
        bButtonDown = gamepad.b;
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

        backButtonPressed = gamepad.back && !backButtonDown;
        backButtonReleased = backButtonDown && !gamepad.back;
        backButtonDown = gamepad.back;
        if (backButtonPressed) backButtonToggleOn = !backButtonToggleOn;
        backButtonToggleActivated = backButtonPressed && backButtonToggleOn;
        backButtonToggleDeactivated = backButtonPressed && !backButtonToggleOn;

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
                "leftStickDown : " + Boolean.toString(isLeftStickDown()) + "\n" +
                "rightStickDown : " + Boolean.toString(isRightStickDown()) + "\n" +
                "leftTriggerDown : " + Boolean.toString(isLeftTriggerDown()) + "\n" +
                "rightTriggerDown : " + Boolean.toString(isRightTriggerDown()) + "\n" +
                "leftBumperDown : " + Boolean.toString(isLeftBumperDown()) + "\n" +
                "rightBumperDown : " + Boolean.toString(isRightBumperDown()) + "\n" +
                "aButtonDown : " + Boolean.toString(isAButtonDown()) + "\n" +
                "bButtonDown : " + Boolean.toString(isBButtonDown()) + "\n" +
                "xButtonDown : " + Boolean.toString(isXButtonDown()) + "\n" +
                "yButtonDown : " + Boolean.toString(isYButtonDown()) + "\n" +
                "backButtonDown : " + Boolean.toString(isBackButtonDown()) + "\n" +
                "startButtonDown : " + Boolean.toString(isStartButtonDown()) + "\n" +
                "leftStickPressed : " + Boolean.toString(isLeftStickPressed()) + "\n" +
                "rightStickPressed : " + Boolean.toString(isRightStickPressed()) + "\n" +
                "leftTriggerPressed : " + Boolean.toString(isLeftTriggerPressed()) + "\n" +
                "rightTriggerPressed : " + Boolean.toString(isRightTriggerPressed()) + "\n" +
                "leftBumperPressed : " + Boolean.toString(isLeftBumperPressed()) + "\n" +
                "rightBumperPressed : " + Boolean.toString(isRightBumperPressed()) + "\n" +
                "aButtonPressed : " + Boolean.toString(isAButtonPressed()) + "\n" +
                "bButtonPressed : " + Boolean.toString(isBButtonPressed()) + "\n" +
                "xButtonPressed : " + Boolean.toString(isXButtonPressed()) + "\n" +
                "yButtonPressed : " + Boolean.toString(isYButtonPressed()) + "\n" +
                "backButtonPressed : " + Boolean.toString(isBackButtonPressed()) + "\n" +
                "startButtonPressed : " + Boolean.toString(isStartButtonPressed()) + "\n" +
                "leftStickReleased : " + Boolean.toString(isLeftStickReleased()) + "\n" +
                "rightStickReleased : " + Boolean.toString(isRightStickReleased()) + "\n" +
                "leftTriggerReleased : " + Boolean.toString(isLeftTriggerReleased()) + "\n" +
                "rightTriggerReleased : " + Boolean.toString(isRightTriggerReleased()) + "\n" +
                "leftBumperReleased : " + Boolean.toString(isLeftBumperReleased()) + "\n" +
                "rightBumperReleased : " + Boolean.toString(isRightBumperReleased()) + "\n" +
                "aButtonReleased : " + Boolean.toString(isAButtonReleased()) + "\n" +
                "bButtonReleased : " + Boolean.toString(isBButtonReleased()) + "\n" +
                "xButtonReleased : " + Boolean.toString(isXButtonReleased()) + "\n" +
                "yButtonReleased : " + Boolean.toString(isYButtonReleased()) + "\n" +
                "backButtonReleased : " + Boolean.toString(isBackButtonReleased()) + "\n" +
                "startButtonReleased : " + Boolean.toString(isStartButtonReleased()) + "\n" +
                "leftStickToggleOn : " + Boolean.toString(isLeftStickToggleOn()) + "\n" +
                "rightStickToggleOn : " + Boolean.toString(isRightStickToggleOn()) + "\n" +
                "leftTriggerToggleOn : " + Boolean.toString(isLeftTriggerToggleOn()) + "\n" +
                "rightTriggerToggleOn : " + Boolean.toString(isRightTriggerToggleOn()) + "\n" +
                "leftBumperToggleOn : " + Boolean.toString(isLeftBumperToggleOn()) + "\n" +
                "rightBumperToggleOn : " + Boolean.toString(isRightBumperToggleOn()) + "\n" +
                "aButtonToggleOn : " + Boolean.toString(isAButtonToggleOn()) + "\n" +
                "bButtonToggleOn : " + Boolean.toString(isBButtonToggleOn()) + "\n" +
                "xButtonToggleOn : " + Boolean.toString(isXButtonToggleOn()) + "\n" +
                "yButtonToggleOn : " + Boolean.toString(isYButtonToggleOn()) + "\n" +
                "backButtonToggleOn : " + Boolean.toString(isBackButtonToggleOn()) + "\n" +
                "startButtonToggleOn : " + Boolean.toString(isStartButtonToggleOn()) + "\n" +
                "leftStickToggleActivated : " + Boolean.toString(isLeftStickToggleActivated()) + "\n" +
                "rightStickToggleActivated : " + Boolean.toString(isRightStickToggleActivated()) + "\n" +
                "leftTriggerToggleActivated : " + Boolean.toString(isLeftTriggerToggleActivated()) + "\n" +
                "rightTriggerToggleActivated : " + Boolean.toString(isRightTriggerToggleActivated()) + "\n" +
                "leftBumperToggleActivated : " + Boolean.toString(isLeftBumperToggleActivated()) + "\n" +
                "rightBumperToggleActivated : " + Boolean.toString(isRightBumperToggleActivated()) + "\n" +
                "aButtonToggleActivated : " + Boolean.toString(isAButtonToggleActivated()) + "\n" +
                "bButtonToggleActivated : " + Boolean.toString(isBButtonToggleActivated()) + "\n" +
                "xButtonToggleActivated : " + Boolean.toString(isXButtonToggleActivated()) + "\n" +
                "yButtonToggleActivated : " + Boolean.toString(isYButtonToggleActivated()) + "\n" +
                "backButtonToggleActivated : " + Boolean.toString(isBackButtonToggleActivated()) + "\n" +
                "startButtonToggleActivated : " + Boolean.toString(isStartButtonToggleActivated()) + "\n" +
                "leftStickToggleDeactivated : " + Boolean.toString(isLeftStickToggleDeactivated()) + "\n" +
                "rightStickToggleDeactivated : " + Boolean.toString(isRightStickToggleDeactivated()) + "\n" +
                "leftTriggerToggleDeactivated : " + Boolean.toString(isLeftTriggerToggleDeactivated()) + "\n" +
                "rightTriggerToggleDeactivated : " + Boolean.toString(isRightTriggerToggleDeactivated()) + "\n" +
                "leftBumperToggleDeactivated : " + Boolean.toString(isLeftBumperToggleDeactivated()) + "\n" +
                "rightBumperToggleDeactivated : " + Boolean.toString(isRightBumperToggleDeactivated()) + "\n" +
                "aButtonToggleDeactivated : " + Boolean.toString(isAButtonToggleDeactivated()) + "\n" +
                "bButtonToggleDeactivated : " + Boolean.toString(isBButtonToggleDeactivated()) + "\n" +
                "xButtonToggleDeactivated : " + Boolean.toString(isXButtonToggleDeactivated()) + "\n" +
                "yButtonToggleDeactivated : " + Boolean.toString(isYButtonToggleDeactivated()) + "\n" +
                "backButtonToggleDeactivated : " + Boolean.toString(isBackButtonToggleDeactivated()) + "\n" +
                "startButtonToggleDeactivated : " + Boolean.toString(isStartButtonToggleDeactivated());
    }
}
