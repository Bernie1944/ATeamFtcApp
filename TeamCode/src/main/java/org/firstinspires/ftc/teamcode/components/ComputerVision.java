package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

// For finding mineral positions relative to robot for sampling
// For finding vuMark locations relative to robot for correcting robots position and rotation
public class ComputerVision extends Component {
    // Should back or front camera be used?
    private static final VuforiaLocalizer.CameraDirection CAMERA_DIRECTION = VuforiaLocalizer.CameraDirection.BACK;

    // Should robot controller phone display Vuforia camera monitor feedback?
    private static final boolean CAMERA_MONITOR_ENABLED = true;

    // Created at https://developer.vuforia.com/targetmanager/licenseManager/licenseListing
    private static final String VUFORIA_LICENSE_KEY = "ARfo4Wj/////AAAAGdsJPEUs3kPKny+l3Yx/j90yMbhD1+gNsZN1XhABIbDvMIrZ+3g2uD98bTZ3hdxw/HzGaFDUFljoOHXLTO3Vh17TU1HABnUF2HxyacXDHvS684uOoR/WhTE96SWR1+XLGC1FMtFTnBIIplR9r10T5wZ75SaWtygfDPwsYayq0HbdXxZbqNUhLhiTF+Z82BhMoL77fR5vdo5t8rituJrfS5oKYropyDgGZ7mzLY+mdjO+k4CFZkxmJCgBBBiyc9j6rCC2ULOCcaGuCyTW8dBbFyskD9lbOA/bcQhC+QrniECnFvUUTMCiBfq+aIp4Roe/Bck5JXnAIjLFkbxYiE3BuXR5JgeEin/MzcI0HWKErygB";

    // Created at https://developer.vuforia.com/targetmanager/licenseManager/licenseListing
    private static final String VU_MARK_ASSET_NAME = "RelicRecoveryVuMark";

    // Instance of the Vuforia localization engine
    private final VuforiaLocalizer localizer;

    // Tracks the vuMark
    private final VuforiaTrackable vuMark;

    // Is Vuforia currently tracking vuMark?
    private boolean vuMarkRecognized = false;

    public ComputerVision(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);

        // Parameters for starting vuforia engine
        VuforiaLocalizer.Parameters localizerParameters = new VuforiaLocalizer.Parameters();
        localizerParameters.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        localizerParameters.useExtendedTracking = false;
        localizerParameters.cameraDirection = CAMERA_DIRECTION;

        if (CAMERA_MONITOR_ENABLED) {
            // Set a camera monitor on the robot controller phone
            localizerParameters.cameraMonitorViewIdParent = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        }

        // Create an instance of the Vuforia localization engine, which is used to identify vuMark, from localizerParameters
        // WARNING: Making multiple instances throws an exception
        localizer = ClassFactory.getInstance().createVuforia(localizerParameters);

        VuforiaTrackables trackables = localizer.loadTrackablesFromAsset(VU_MARK_ASSET_NAME);

        vuMark = trackables.get(0);
        vuMark.setName(VU_MARK_ASSET_NAME);

        trackables.activate();

        // Set camera to always be focusing
        CameraDevice.getInstance().setFocusMode(CameraDevice.FOCUS_MODE.FOCUS_MODE_CONTINUOUSAUTO);
    }
}
