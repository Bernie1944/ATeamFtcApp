package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.Degrees;

public class ComputerVision extends Component {
    private static final String WEBCAM_NAME = "Webcam";

    // Created at https://developer.vuforia.com/targetmanager/licenseManager/licenseListing
    private static final String VUFORIA_LICENSE_KEY = "AZrJNIT/////AAABmUBPE2HqLEo1hziFVtdVlJFeoMtODZ93fShJ8HZGNxCtETtBs6j2doOZLJsFSlXZQRvc4jgrMc9kWalAXzsVs7rGX8af2tNvgEpjHkR5/RAgmkJGVirjgnvpL4Qv5yomnWutC8DtsmNOdHWrJ4+sIrkuhyJ1z2I46Qa1k4g3Jud2KlYkGf/2GbEZJ2MepW0RfcmLHmBxhXI0k/pg1NpBeybsqi9c5HxA+oPQFL/B4/ZlJjpLPW3f1bPWXAdMHX5Ns/A0hdIviiN0bknU3T8sNlkT4O88edwXDMYAiSvaY+FgytwR+/OA1rscw5vTPP6Lh4W65tguI1UvnZITc2EUyU2bGqI14xR7d/rnfIhRDhgM";

    // Tensor Flow Lite .tflite model asset
    private static final String OBJECT_DETECTOR_MODEL_ASSET_NAME = "RoverRuckus.tflite";

    // 0.4 by default
    private static final double OBJECT_DETECTOR_MINIMUM_CONFIDENCE = 0.27; //0.3; //0.25; //0.3;

    // Instance of the Vuforia localization engine
    private final VuforiaLocalizer localizer;
    private final TFObjectDetector objectDetector;

    private final Nav nav;

    // See getter and setter methods
    private boolean goldMineralRecognized = false;
    private boolean goldMineralHeadingUpdatesEnabled = false;
    private double goldMineralHeading = 0.0;

    public ComputerVision(Telemetry telemetry, HardwareMap hardwareMap, Nav nav) {
        super(telemetry, hardwareMap);

        this.nav = nav;

        // Start Vuforia engine
        VuforiaLocalizer.Parameters localizerParameters = new VuforiaLocalizer.Parameters();
        localizerParameters.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        localizerParameters.cameraName = hardwareMap.get(WebcamName.class, WEBCAM_NAME);
        localizerParameters.useExtendedTracking = false;
        localizer = ClassFactory.getInstance().createVuforia(localizerParameters);

        // Start Tensor Flow engine on top of Vuforia engine
        TFObjectDetector.Parameters objectDetectorParameters = new TFObjectDetector.Parameters();
        objectDetectorParameters.minimumConfidence = OBJECT_DETECTOR_MINIMUM_CONFIDENCE;
        objectDetectorParameters.tfodMonitorViewIdParent =
                hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        objectDetector = ClassFactory.getInstance().createTFObjectDetector(objectDetectorParameters, localizer);
        objectDetector.loadModelFromAsset(OBJECT_DETECTOR_MODEL_ASSET_NAME, "Gold Mineral");
    }

    // Activates recognition and sets isGoldMineralHeadingUpdatesEnabled() to true
    public void activateGoldMineralRecognition() {
        objectDetector.activate();
        setGoldMineralHeadingUpdatesEnabled(true);
    }

    // Deactivates recognition and sets isGoldMineralHeadingUpdatesEnabled() to false
    public void deactivateGoldMineralRecognition() {
        objectDetector.deactivate();
        setGoldMineralHeadingUpdatesEnabled(false);
    }

    // Is a gold mineral currently recognized by Tensor Flow?
    public boolean isGoldMineralRecognized() {
        return goldMineralRecognized;
    }

    // Should getGoldMineralHeading() be updated when a gold mineral is recognized?
    // Disabling this can be useful when robot is rotating quickly, as fast rotations can result in inaccurate getGoldMineralHeading() values
    public boolean isGoldMineralHeadingUpdatesEnabled() {
        return goldMineralHeadingUpdatesEnabled;
    }

    // Should getGoldMineralHeading() be updated when a gold mineral is recognized?
    // Disabling this can be useful when robot is rotating quickly, as fast rotations can result in inaccurate getGoldMineralHeading() values
    public void setGoldMineralHeadingUpdatesEnabled(boolean goldMineralHeadingUpdatesEnabled) {
        this.goldMineralHeadingUpdatesEnabled = goldMineralHeadingUpdatesEnabled;
    }

    // Last known/set degrees between [-180, 180] starting with gold mineral on positive x-axis (to right of robot from the the driving team's perspective) going counterclockwise around camera
    // If heading is outside [-180, 180] it will be looped over into this range
    public double getGoldMineralHeading() {
        return goldMineralHeading;
    }

    // In degrees starting with gold mineral on positive x-axis (to right of robot from the the driving team's perspective) going counterclockwise around camera
    // If heading is outside [-180, 180] it will be looped over into this range
    public void setGoldMineralHeading(double goldMineralHeading) {
        this.goldMineralHeading = Degrees.normalize(goldMineralHeading);
    }

    // In degrees between [-180, 180] starting with gold mineral in front of camera going counterclockwise around camera as calculated using getGoldMineralHeading()
    public double getGoldMineralHeadingRelativeToCamera() {
        return Degrees.normalize(getGoldMineralHeading() - nav.getHeading() - 90.0);
    }

    // Called through Component.update()
    @Override
    void internalUpdate() {
        if (isGoldMineralHeadingUpdatesEnabled()) {
            goldMineralRecognized = false;
            double highestRecognitionConfidence = 0.0;
            for (Recognition recognition : objectDetector.getRecognitions()) {
                goldMineralRecognized = true;

                double recognitionConfidence = recognition.getConfidence();
                if (recognitionConfidence > highestRecognitionConfidence) {
                    highestRecognitionConfidence = recognitionConfidence;

                    double goldMineralHeadingRelativeToCamera = -recognition.estimateAngleToObject(AngleUnit.DEGREES);
                    setGoldMineralHeading(goldMineralHeadingRelativeToCamera + nav.getHeading() + 90.0);
                }
            }
        } else {
            goldMineralRecognized = !objectDetector.getRecognitions().isEmpty();
        }
    }

    @Override
    public String toString() {
        return createStateString("goldMineralRecognized", isGoldMineralRecognized()) +
                createStateString("goldMineralHeadingUpdatesEnabled", isGoldMineralHeadingUpdatesEnabled()) +
                createStateString("goldMineralHeading", "%.0f°", getGoldMineralHeading()) +
                createStateString("goldMineralHeadingRelativeToCamera", "%.0f°", getGoldMineralHeadingRelativeToCamera());
    }
}