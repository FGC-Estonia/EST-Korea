package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

/**
 * Module for feeding collected balls into the launcher.
 */
public class +
        eedBalls {
    private DcMotorEx motor = null;
    private final boolean protect;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    /**
     * Initializes the feeder module.
     */
    public FeedBalls(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapMotors();
    }

    private void mapMotors() {
        motor = hardwareMap.get(DcMotorEx.class, HardwareConstants.FEEDER_MOTOR);

        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Activates or deactivates the feeding motor.
     * 
     * @param active True to feed balls, false to stop.
     */
    public void feed(boolean active) {
        if (active) {
            motor.setPower(1.0);
        } else {
            motor.setPower(0);
        }
    }
    public void clear(boolean active) {
        if (active) {
            motor.setPower(-1.0);
        } else {
            motor.setPower(0);
        }
    }

    public void stop() {
        motor.setPower(0);
    }
}
