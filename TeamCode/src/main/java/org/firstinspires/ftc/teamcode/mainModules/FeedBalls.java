package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.util.Protect;

/**
 * Module for feeding collected balls into the launcher.
 */
public class FeedBalls {
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
        motor = Protect.map(protect, telemetry, HardwareConstants.FEEDER_MOTOR, () -> {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, HardwareConstants.FEEDER_MOTOR);
            m.setDirection(DcMotor.Direction.REVERSE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return m;
        });
    }

    /** True if the feeder motor is actually in the configuration. */
    public boolean isAvailable() {
        return motor != null;
    }

    /**
     * Activates or deactivates the feeding motor.
     * 
     * @param active True to feed balls, false to stop.
     */
    public void feed(boolean active) {
        if (motor == null) {
            return;
        }
        final double power = active ? 1.0 : 0;
        Protect.run(protect, telemetry, "FeedBalls.feed", () -> motor.setPower(power));
    }
    public void clear(boolean active) {
        if (motor == null) {
            return;
        }
        final double power = active ? -1.0 : 0;
        Protect.run(protect, telemetry, "FeedBalls.clear", () -> motor.setPower(power));
    }

    public void stop() {
        if (motor == null) {
            return;
        }
        Protect.run(protect, telemetry, "FeedBalls.stop", () -> motor.setPower(0));
    }
}
