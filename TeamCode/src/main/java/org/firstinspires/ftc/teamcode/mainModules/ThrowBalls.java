package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        wheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Spins the launcher wheel.
     * 
     * @param Spinning True to spin at full power, false to stop.
     */
    public void spin(boolean Spinning) {
        if (Spinning) {// Inside the while loop
            wheelMotor.setVelocity(1940); // Set target velocity when activated
        } else {
            wheelMotor.setVelocity(0); // Stop the wheel
        }
    }

    public double throwSpeed() {
        return wheelMotor.getVelocity();
    }
    public void stop() {
        wheelMotor.setVelocity(0);
    }
}
