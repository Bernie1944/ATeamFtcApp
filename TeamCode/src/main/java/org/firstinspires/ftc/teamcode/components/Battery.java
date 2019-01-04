package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// For getting battery voltage
public class Battery extends Component {
    // See getter method
    private double voltage = 0.0;

    public Battery(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);
    }

    // Returns 0.0 if voltage is not available
    public double getVoltage() {
        return voltage;
    }

    // Called through Component.update()
    @Override
    void updateImpl() {
        // Find the smallest valid VoltageSensor.getVoltage()
        double smallestSensorVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double sensorVoltage = sensor.getVoltage();

            // If sensorVoltage is valid, update smallestSensorVoltage
            if (sensorVoltage > 0.0) {
                smallestSensorVoltage = Math.min(smallestSensorVoltage, sensorVoltage);
            }
        }

        if (smallestSensorVoltage < Double.POSITIVE_INFINITY) {
            // smallestSensorVoltage was found
            voltage = smallestSensorVoltage;
        } else {
            // 0.0 indicates voltage is not available
            voltage = 0.0;
        }
    }

    // Returns text describing state
    @Override
    public String toString() {
        return String.format("voltage : %4.2f", getVoltage());
    }
}
