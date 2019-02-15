package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.Range;

import java.util.Map;
import java.util.TreeMap;

// Extends a TreeMap<Double, Double> to make get() linearly interpolate between values around key
public class DoubleMap extends TreeMap<Double, Double> {
    // Linearly interpolates between values around key
    public double get(double key) {
        Map.Entry<Double, Double> priorEntry = floorEntry(key);
        Map.Entry<Double, Double> latterEntry = ceilingEntry(key);

        if (priorEntry == null && latterEntry == null) {
            return 0.0;
        } else if (priorEntry == null) {
            return latterEntry.getValue();
        } else if (latterEntry == null) {
            return priorEntry.getValue();
        } else {
            return Range.scale(key, priorEntry.getKey(), latterEntry.getKey(), priorEntry.getValue(), latterEntry.getValue());
        }
    }
}