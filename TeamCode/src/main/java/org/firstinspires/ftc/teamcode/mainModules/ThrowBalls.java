package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

public class ThrowBalls {

    private DcMotorEx wheelMotor;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final boolean protect;

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

    public void spin(boolean Spinning) {
        if (Spinning) {
            wheelMotor.setPower(1); // example max position (adjust as needed)
        } else {
            wheelMotor.setPower(0); // example min position (adjust as needed)
        }
    }

    public void stop() {
        wheelMotor.setVelocity(0);
    }
}
