package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.Degrees;
import org.firstinspires.ftc.teamcode.util.Vector2;

// Combines Drive with a imu to track robots orientation relative to the playing field
public class Nav extends Component {
    private static final String IMU_1_I2C_NAME = "NavImu1";
    private static final String IMU_2_I2C_NAME = "NavImu2";

    // Controls drive wheels
    public final Drive drive;

    // For accurately determining robot heading
    private BNO055IMU imu;

    private final BNO055IMU imu1;
    private final BNO055IMU imu2;

    private boolean imu1InitalizationSuccessful;

    // In degrees
    private double headingOffsetFromImu;

    // See getter methods
    private static Vector2 position = Vector2.ZERO;
    private static double heading = 0.0;
    private double angularVelocity;

    // initialPosition is in inches relative to center of playing field with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    // The BNO055IMU inertial motion units can take a long time to initialize, as BNO055IMU class will try initialization five times before it gives up,
    // so Nav constructor may take close to 12 seconds
    public Nav(Telemetry telemetry, HardwareMap hardwareMap) throws InitializationFailedException {
        super(telemetry, hardwareMap);

        drive = new Drive(telemetry, hardwareMap);

        imu1 = hardwareMap.get(BNO055IMU.class, IMU_1_I2C_NAME);
        imu2 = hardwareMap.get(BNO055IMU.class, IMU_2_I2C_NAME);

        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                // The BNO055IMU inertial motion units can take a long time to initialize, as the class will try initialization five times before it gives up
                BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
                imuParameters.mode = BNO055IMU.SensorMode.NDOF;
                imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                imuParameters.calibrationDataFile = IMU_1_I2C_NAME + "Calibration.json";
                imu1InitalizationSuccessful = imu1.initialize(imuParameters);
            }
        });

        thread.run();

        // The BNO055IMU inertial motion units can take a long time to initialize, as the class will try initialization five times before it gives up
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.mode = BNO055IMU.SensorMode.NDOF;
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.calibrationDataFile = IMU_2_I2C_NAME + "Calibration.json";
        boolean imu2InitializationSuccessful = imu2.initialize(imuParameters);
        if (!imu2InitializationSuccessful) {
            throw new InitializationFailedException(IMU_2_I2C_NAME + " initialization failed!");
        }

        try {
            thread.join();
        } catch(InterruptedException e) {
            throw new InitializationFailedException(IMU_1_I2C_NAME + " initialization failed!");
        }

        if (!imu1InitalizationSuccessful) {
            throw new InitializationFailedException(IMU_1_I2C_NAME + " initialization failed!");
        }

        imu = imu2;

        // Set headingOffsetFromImu(s) to how far heading is from imu's heading reading
        headingOffsetFromImu = Degrees.normalize(heading - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

        angularVelocity = -imu.getAngularVelocity().toAngleUnit(AngleUnit.DEGREES).zRotationRate;
    }

    // Saves files that will be used to restore IMU calibration in Nav constructor after the IMU power cycles
    public void saveImuCalibration() {
        // Check to see if IMU has been initialized, as BNO055IMU.readCalibrationData() throws if BNO055IMU.getParameters().mode is null
        if (imu.getParameters().mode == BNO055IMU.SensorMode.NDOF) {
            // A all zero CalibrationData usually means the CalibrationData isn't valid, such as when the REV hubs are not connected
            BNO055IMU.CalibrationData imuCalibrationData = imu.readCalibrationData();
            if (imuCalibrationData.dxAccel != 0 || imuCalibrationData.dxGyro != 0 || imuCalibrationData.dxMag != 0 ||
                    imuCalibrationData.dyAccel != 0 || imuCalibrationData.dyGyro != 0 || imuCalibrationData.dyMag != 0 ||
                    imuCalibrationData.dzAccel != 0 || imuCalibrationData.dzGyro != 0 || imuCalibrationData.dzMag != 0 ||
                    imuCalibrationData.radiusAccel != 0 || imuCalibrationData.radiusMag != 0) {
                ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile(IMU_1_I2C_NAME + "Calibration.json"), imuCalibrationData.serialize());
            }
        }
    }

    // In inches relative to center of playing field with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Vector2 getPosition() {
        return position;
    }

    // In inches relative to center of playing field with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public void setPosition(Vector2 position) {
        Nav.position = position;
    }

    // In inches per second with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Vector2 getVelocity() {
        return drive.getVelocity().addRotation(getHeading());
    }

    // In degrees between [-180, 180] starting with robot facing positive x-axis (facing right from the the driving team's perspective) going counterclockwise
    public double getHeading() {
        return heading;
    }

    // In degrees starting with robot facing positive x-axis (facing right from the the driving team's perspective) going counterclockwise
    // If heading is outside [-180, 180] it will be looped over into this range
    public void setHeading(double heading) {
        // Adjust headingOffsetFromImu by how far new heading is from old heading
        headingOffsetFromImu = Degrees.normalize(headingOffsetFromImu + (heading - Nav.heading));

        Nav.heading = Degrees.normalize(heading);
    }

    // In degrees per second with positive going counterclockwise
    public double getAngularVelocity() {
        return angularVelocity;
    }

    // True if drive has set target velocity and angular velocity and is not running to target position and heading
    public boolean areTargetVelocitiesSet() {
        return drive.areTargetVelocitiesSet();
    }

    // In inches per second with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    public Vector2 getTargetVelocity() {
        return drive.getTargetVelocity().addRotation(getHeading());
    }

    // In degrees per second with positive counterclockwise
    public double getTargetAngularVelocity() {
        return drive.getTargetAngularVelocity();
    }

    // targetVelocity is in inches per second with positive x-axis to the right of the driving team and positive y-axis to the front of the driving team
    // targetAngularVelocity is in degrees per second with positive counterclockwise
    // If targetVelocity and targetAngularVelocity are not obtainable, both of these values will be scaled proportionally and targetVelocity's direction will be kept the same
    // limitAccelerations: should accelerations be limited to values that should not cause the wheels to slip and the robot to loose it's position?
    public void setTargetVelocities(Vector2 targetVelocity, double targetAngularVelocity) {
        drive.setTargetVelocities(targetVelocity.subRotation(getHeading()), targetAngularVelocity);
    }

    public void switchImu() {
        if (imu == imu1) {
            imu = imu2;
        } else {
            imu = imu1;
        }
    }

    // Called through Component.update()
    @Override
    void internalUpdate() {
        drive.update();

        heading = Degrees.normalize(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + headingOffsetFromImu);
        angularVelocity = -imu.getAngularVelocity().toAngleUnit(AngleUnit.DEGREES).zRotationRate;
        position = position.add(getVelocity().mul(deltaTime));
    }

    @Override
    public String toString() {
        return createStateString("position", getPosition().toString("%.2fin")) +
                createStateString("heading", "%.0f°", getHeading()) +
                createStateString("headingOffsetFromImu", "%.0f°", headingOffsetFromImu) +
                createStateString("targetVelocitiesSet", areTargetVelocitiesSet()) +
                createStateString("targetVelocity", getTargetVelocity().toString("%.2fin/s")) +
                createStateString("velocity", getVelocity().toString("%.2fin/s")) +
                createStateString("targetAngularVelocity", "%.0f°/s", getTargetAngularVelocity()) +
                createStateString("angularVelocity", "%.0f°/s", getAngularVelocity()) +
                drive.toString();
    }
}