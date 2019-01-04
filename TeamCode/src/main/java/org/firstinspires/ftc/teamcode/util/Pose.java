package org.firstinspires.ftc.teamcode.util;

import android.support.annotation.Nullable;

import java.util.Locale;

// Each Pose represents a certain orientation of the robot
// When the robot is not fully in all of the tolerances of any pose, the robot is in between poses
public enum Pose {
    PLACING_SLIVER_IN_CARGO_HOLD(0.5, new HarvesterPosition(6.0, -6.0, 34.0, 3.0, true)),
    PLACING_GOLD_IN_CARGO_HOLD(0.5, new HarvesterPosition(6.0, 6.0, 34.0, 3.0, true)),
    MOVE_TO_PICK_UP_MINERALS(Double.POSITIVE_INFINITY, new HarvesterPosition(60.0, -60.0, 0.0, 5.0, false));

    // Minimum seconds that pose should stay at this Pose before going to the next one
    // Positive infinity indicates that there should not be a progression to the next Pose and this Pose is the end of a "program" of poses
    private double minDelay;

    // Extended to allow simpler Pose constructors
    public static abstract class ComponentPose {
    }

    public static class LatchPosition extends ComponentPose {
        public final double target;
        public final double tolerance;

        public LatchPosition(double target, double tolerance) {
            this.target = target;
            this.tolerance = tolerance;
        }

        public LatchPosition(LatchPosition other) {
            target = other.target;
            tolerance = other.tolerance;
        }

        @Override
        public String toString() {
            return String.format(
                    Locale.US,
                    "target :    %6.4f" + "%n" +
                            "tolerance : %6.4f",
                    target, tolerance
            );
        }
    }

    @Nullable
    private LatchPosition latchPosition;

    public static class NavPosition extends ComponentPose {
        // In inches relative to center of playing field with positive x-axis to the right of audience and positive y-axis to the front of audience
        public final Vector2 target;

        // In inches around target that is considered inside the pose
        public final double tolerance;

        public NavPosition(Vector2 target, double tolerance) {
            this.target = target;
            this.tolerance = tolerance;
        }

        public NavPosition(double x, double y, double tolerance) {
            this.target = new Vector2(x, y);
            this.tolerance = tolerance;
        }

        public NavPosition(NavPosition other) {
            target = other.target;
            tolerance = other.tolerance;
        }

        @Override
        public String toString() {
            return String.format(
                    Locale.US,
                    "target {" + "%n" +
                            "%s" + "%n" +
                            "}" + "%n" +
                            "tolerance : %10.4f",
                    target.toString(), tolerance
            );
        }
    }

    @Nullable
    private NavPosition navPosition;

    public static class NavRotation extends ComponentPose {
        // In degrees starting with robot facing positive x-axis (facing right from the audiences perspective) going counterclockwise
        // This is normalized, meaning it loops over into the range [-180, 180]
        public final double target;

        // In degrees around target that is considered inside the pose
        public final double tolerance;

        public NavRotation(double target, double tolerance) {
            this.target = Degrees.normalize(target);
            this.tolerance = tolerance;
        }

        public NavRotation(NavRotation other) {
            target = other.target;
            tolerance = other.tolerance;
        }

        @Override
        public String toString() {
            return String.format(
                    Locale.US,
                    "target :    %6.0f" + "%n" +
                            "tolerance : %6.0f",
                    target, tolerance
            );
        }
    }

    @Nullable
    private NavRotation navRotation;

    public static class HarvesterPosition extends ComponentPose {
        // In inches relative to center of playing field with positive x-axis to the right of audience, positive y-axis to the front of audience, and positive z-axis up
        public final Vector3 target;

        // In inches around target that is considered inside the pose
        public final double tolerance;

        // Should harvester arm be extended behind robot or in front
        public final boolean armIsBehind;

        public HarvesterPosition(Vector3 target, double tolerance, boolean armIsBehind) {
            this.target = target;
            this.tolerance = tolerance;
            this.armIsBehind = armIsBehind;
        }

        public HarvesterPosition(double x, double y, double z, double tolerance, boolean armIsBehind) {
            this.target = new Vector3(x, y, z);
            this.tolerance = tolerance;
            this.armIsBehind = armIsBehind;
        }

        public HarvesterPosition(HarvesterPosition other) {
            target = other.target;
            tolerance = other.tolerance;
            armIsBehind = other.armIsBehind;
        }

        @Override
        public String toString() {
            return String.format(
                    Locale.US,
                    "target {" + "%n" +
                            "%s" + "%n" +
                            "}" + "%n" +
                            "tolerance : %10.4f" + "%n" +
                            "armIsBehind : %b",
                    target.toString(), tolerance, armIsBehind
            );
        }
    }

    @Nullable
    private HarvesterPosition harvesterPosition;

    Pose(double minDelay) {
        this.minDelay = minDelay;
    }

    Pose(double minDelay, ComponentPose a) {
        this.minDelay = minDelay;
        setComponentPose(a);
    }

    Pose(double minDelay, ComponentPose a, ComponentPose b) {
        this.minDelay = minDelay;
        setComponentPose(a);
        setComponentPose(b);
    }

    Pose(double minDelay, ComponentPose a, ComponentPose b, ComponentPose c) {
        this.minDelay = minDelay;
        setComponentPose(a);
        setComponentPose(b);
        setComponentPose(c);
    }

    Pose(double minDelay, ComponentPose a, ComponentPose b, ComponentPose c, ComponentPose d) {
        this.minDelay = minDelay;
        setComponentPose(a);
        setComponentPose(b);
        setComponentPose(c);
        setComponentPose(d);
    }

    private void setComponentPose(ComponentPose componentPose) {
        if (componentPose instanceof LatchPosition) {
            latchPosition = (LatchPosition) componentPose;
        } else if (componentPose instanceof NavPosition) {
            navPosition = (NavPosition) componentPose;
        } else if (componentPose instanceof NavRotation) {
            navRotation = (NavRotation) componentPose;
        } else if (componentPose instanceof HarvesterPosition) {
            harvesterPosition = (HarvesterPosition) componentPose;
        }
    }

    public boolean isEndOfProgram() {
        return minDelay == Double.POSITIVE_INFINITY;
    }

    public Pose getPrevious() {
        if (ordinal() - 1 < 0) return null;

        Pose previousPose = Pose.values()[ordinal() - 1];

        if (previousPose.isEndOfProgram()) return null;
        else return previousPose;
    }

    public Pose getNext() {
        if (isEndOfProgram() || ordinal() + 1 >= Pose.values().length) return null;

        return Pose.values()[ordinal() + 1];
    }

    public double getMinDelay() {
        return minDelay;
    }

    public void setMinDelay(double minDelay) {
        this.minDelay = minDelay;
    }

    @Nullable
    public LatchPosition getLatchPosition() {
        return latchPosition;
    }

    public void setLatchPosition(@Nullable LatchPosition latchPosition) {
        this.latchPosition = latchPosition;
    }

    @Nullable
    public NavPosition getNavPosition() {
        return navPosition;
    }

    public void setNavPosition(@Nullable NavPosition navPosition) {
        this.navPosition = navPosition;
    }

    @Nullable
    public NavRotation getNavRotation() {
        return navRotation;
    }

    public void setNavRotation(@Nullable NavRotation navRotation) {
        this.navRotation = navRotation;
    }

    @Nullable
    public HarvesterPosition getHarvesterPosition() {
        return harvesterPosition;
    }

    public void setHarvesterPosition(@Nullable HarvesterPosition harvesterPosition) {
        this.harvesterPosition = harvesterPosition;
    }

    public LatchPosition getPredictiveLatchPosition() {
        if (latchPosition != null) {
            return latchPosition;
        } else {
            // If there is a next pose, return next pose's getPredictiveLatchPosition(); otherwise, return null, indicating there is no predicted nav rotation
            if (getNext() != null) return getNext().getPredictiveLatchPosition();
            else return null;
        }
    }

    public NavPosition getPredictiveNavPosition() {
        if (navPosition != null) {
            return navPosition;
        } else {
            // If there is a next pose, return next pose's getPredictiveNavPosition(); otherwise, return null, indicating there is no predicted nav rotation
            if (getNext() != null) return getNext().getPredictiveNavPosition();
            else return null;
        }
    }

    public NavRotation getPredictiveNavRotation() {
        if (navRotation != null) {
            return navRotation;
        } else {
            // If there is a next pose, return next pose's getPredictiveNavRotation(); otherwise, return null, indicating there is no predicted nav rotation
            if (getNext() != null) return getNext().getPredictiveNavRotation();
            else return null;
        }
    }

    public HarvesterPosition getPredictiveHarvesterPosition() {
        if (harvesterPosition != null) {
            return harvesterPosition;
        } else {
            // If there is a next pose, return next pose's getPredictiveHarvesterPosition(); otherwise, return null, indicating there is no predicted nav rotation
            if (getNext() != null) return getNext().getPredictiveHarvesterPosition();
            else return null;
        }
    }

    public String toStringVerbose() {
        return String.format(
                Locale.US,
                "name : %s" + "%n" +
                        "latchPosition {" + "%n" +
                        "%s" + "%n" +
                        "}" + "%n" +
                        "navPosition {" + "%n" +
                        "%s" + "%n" +
                        "}" + "%n" +
                        "navRotation {" + "%n" +
                        "%s" + "%n" +
                        "}" + "%n" +
                        "harvesterPosition {" + "%n" +
                        "%s" + "%n" +
                        "}" + "%n" +
                        "predictiveLatchPosition {" + "%n" +
                        "%s" + "%n" +
                        "}" + "%n" +
                        "predictiveNavPosition {" + "%n" +
                        "%s" + "%n" +
                        "}" + "%n" +
                        "predictiveNavRotation {" + "%n" +
                        "%s" + "%n" +
                        "}" + "%n" +
                        "predictiveHarvesterPosition {" + "%n" +
                        "%s" + "%n" +
                        "}",
                toString(),
                latchPosition != null ? latchPosition.toString() : "UNSPECIFIED",
                navPosition != null ? navPosition.toString() : "UNSPECIFIED",
                navRotation != null ? navRotation.toString() : "UNSPECIFIED",
                harvesterPosition != null ? harvesterPosition.toString() : "UNSPECIFIED",
                getPredictiveLatchPosition() != null ? getPredictiveLatchPosition().toString() : "UNSPECIFIED",
                getPredictiveNavPosition() != null ? getPredictiveNavPosition().toString() : "UNSPECIFIED",
                getPredictiveNavRotation() != null ? getPredictiveNavRotation().toString() : "UNSPECIFIED",
                getPredictiveHarvesterPosition() != null ? getPredictiveHarvesterPosition().toString() : "UNSPECIFIED"
        );
    }
}
