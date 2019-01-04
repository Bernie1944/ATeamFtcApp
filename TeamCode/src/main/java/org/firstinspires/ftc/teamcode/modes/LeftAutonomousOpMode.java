package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.BucketLocalizer;
import org.firstinspires.ftc.teamcode.components.DriveLocalizer;
import org.firstinspires.ftc.teamcode.components.Latch;
import org.firstinspires.ftc.teamcode.components.PoseManager;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Vector2;

import java.util.Locale;

@Autonomous(name="Left")
public class LeftAutonomousOpMode extends OpMode {
    Latch latch;
    DriveLocalizer driveLocalizer;
    BucketLocalizer bucketLocalizer;
    PoseManager poseManager;

    Pose targetPose;

    @Override
    public void init() {
        latch = new Latch(telemetry, hardwareMap);
        driveLocalizer = new DriveLocalizer(telemetry, hardwareMap, new Vector2(0.0, 0.0), 0.0);
        bucketLocalizer = new BucketLocalizer(telemetry, hardwareMap, driveLocalizer);
        poseManager = new PoseManager(telemetry, hardwareMap, latch, driveLocalizer, bucketLocalizer);
    }

    @Override
    public void init_loop() {
        latch.update();
        driveLocalizer.update();
        bucketLocalizer.update();
        poseManager.update();
    }

    @Override
    public void loop() {
        init_loop();

        telemetry.addLine(toStringVerbose());
    }

    // Returns text describing state
    @Override
    public String toString() {
        return "latch {\n" +
                latch.toString() + "\n" +
                "}\n" +
                "driveLocalizer {\n" +
                driveLocalizer.toString() + "\n" +
                "}\n" +
                "bucketLocalizer {\n" +
                bucketLocalizer.toString() + "\n" +
                "}\n" +
                "poseManager {\n" +
                poseManager.toString() + "\n" +
                "}";
    }

    // Returns text verbosely describing state
    public String toStringVerbose() {
        return "latch {\n" +
                latch.toStringVerbose() + "\n" +
                "}\n" +
                "driveLocalizer {\n" +
                driveLocalizer.toStringVerbose() + "\n" +
                "}\n" +
                "bucketLocalizer {\n" +
                bucketLocalizer.toStringVerbose() + "\n" +
                "}\n" +
                "poseManager {\n" +
                poseManager.toStringVerbose() + "\n" +
                "}";
    }
}
