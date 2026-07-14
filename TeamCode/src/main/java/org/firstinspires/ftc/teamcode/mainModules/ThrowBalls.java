package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

/**
 * Module for launching balls toward the goal.
 */
public class ThrowBalls {

    private DcMotorEx wheelMotor;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final boolean protect;

    /**
     * Initializes the launcher module.
     */
    public ThrowBalls(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapMotor();
    }

    private void mapMotor() {
        wheelMotor = hardwareMap.get(DcMotorEx.class, HardwareConstants.WHEEL_MOTOR);
        wheelMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    /**
     * Spins the launcher wheel.
     * 
     * @param Spinning True to spin at full power, false to stop.
     */
    public void spin(boolean Spinning) {
        if (Spinning) {// Inside the while loop
            wheelMotor.setVelocity(100*Math.PI/3, AngleUnit.RADIANS);; // Maximum power when activated
        } else {
            wheelMotor.setVelocity(0); // No power for using flywheel intertia
        }
    }

    public void stop() {
        wheelMotor.setVelocity(0);
    }
}
