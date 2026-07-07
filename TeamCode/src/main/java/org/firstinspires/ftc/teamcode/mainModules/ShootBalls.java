package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

public class ShootBalls {
    private final DcMotor motor;
    private final Telemetry telemetry;
    private final boolean protect;

    /**
     * Constructor for ShootBalls module.
     * @param protect Whether to use protective measures
     * @param hardwareMap The hardware map from the OpMode
     * @param telemetry The telemetry from the OpMode
     */
    public ShootBalls(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.telemetry = telemetry;
        
        motor = hardwareMap.get(DcMotor.class, HardwareConstants.SHOOTER_MOTOR);

        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void start() {
        motor.setPower(1.0);
    }

    public void stop() {
        motor.setPower(0.0);
    }
}
