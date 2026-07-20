package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance");
    }
    public double getDistance(){
        return distanceSensor.getDistance(DistanceUnit.MM);
    }

    public double alignTarget() {
        double error = TARGET - getDistance();
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
