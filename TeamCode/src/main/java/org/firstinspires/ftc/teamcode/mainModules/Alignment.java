package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.util.Protect;

public class Alignment {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DistanceSensor distanceSensor;

    public double TARGET = 150.0;
    public double TOLERANCE = 10.0;

    private double Kp = 0.0019;

    private final boolean protect;

    public Alignment(boolean protect, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.protect = protect;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        init();
    }

    private void init() {
        distanceSensor = Protect.map(protect, telemetry, "Distance",
                () -> hardwareMap.get(DistanceSensor.class, "Distance"));
    }

    /** True if the distance sensor is actually in the configuration. */
    public boolean isAvailable() {
        return distanceSensor != null;
    }
    public double getDistance(){
        if (distanceSensor == null) {
            return 1000.0;
        }
        // A big number on failure reads as "nothing in range", so the driver
        // keeps full manual control instead of chasing a bogus correction.
        return Protect.getDouble(protect, telemetry, "Alignment.getDistance",
                () -> distanceSensor.getDistance(DistanceUnit.MM), 1000.0);
    }

    public double alignTarget(double currentDistance) {
        double error = TARGET - currentDistance;
        telemetry.addData("Error", error);

        if (Math.abs(error) > TOLERANCE) {
            double correction = -(Kp * error); // Calculate proportional speed

            // Ensure the speed is within an acceptable range (-1.0 to 1.0)
            return Math.max(-1.0, Math.min(1.0, correction));
        } else {
            return 0.0;
        }
    }
}
