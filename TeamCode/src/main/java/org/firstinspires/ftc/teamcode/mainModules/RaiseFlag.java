package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

/**
 * Module for raising the flag
 */
public class RaiseFlag {

    private Servo flagServo;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    /**
     * Initializes the flag raising module.
     */
    public RaiseFlag(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapServo();
    }

    private void mapServo() {
        flagServo = hardwareMap.get(Servo.class, HardwareConstants.FLAG_SERVO_MOTOR);
        flagServo.setDirection(Servo.Direction.FORWARD);
    }

    /**
     * Sets the position of the flag servo.
     * 
     * @param pos Servo position (0.0 to 1.0).
     */
    public void setPos(double pos) {
        flagServo.setPosition(pos);
    }

}
