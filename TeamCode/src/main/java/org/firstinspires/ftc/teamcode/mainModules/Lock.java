package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

/**
 * Module for locking the climbing
 */
public class Lock {

    private Servo lockServo;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private boolean isLocked = false;

    /**
     * Initializes the locking module.
     */
    public Lock(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapServo();
    }

    private void mapServo() {
        lockServo = hardwareMap.get(Servo.class, HardwareConstants.LOCK_SERVO_MOTOR);
        lockServo.setDirection(Servo.Direction.FORWARD);
    }

    /**
     * Locks the climbing mechanism.
     */
    public void lock() {
        lockServo.setPosition(1.0);
        isLocked = true;
    }

    /**
     * Unlocks the climbing mechanism.
     */
    public void unlock() {
        lockServo.setPosition(0.0);
        isLocked = false;
    }

    public boolean isLocked() {
        return isLocked;
    }

    /**
     * Sets the position of the lock servo.
     *
     * @param pos Servo position (0.0 to 1.0).
     */
    public void setPos(double pos) {
        lockServo.setPosition(pos);
    }

}
