package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.util.Protect;

/**
 * Module for locking the climbing
 */
public class Lock {

    private Servo lockServo;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final boolean protect;

    private boolean isLocked = false;

    /**
     * Initializes the locking module.
     */
    public Lock(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapServo();
    }

    private void mapServo() {
        lockServo = Protect.map(protect, telemetry, HardwareConstants.LOCK_SERVO_MOTOR, () -> {
            Servo s = hardwareMap.get(Servo.class, HardwareConstants.LOCK_SERVO_MOTOR);
            s.setDirection(Servo.Direction.FORWARD);
            return s;
        });
    }

    /** True if this servo is actually in the configuration. */
    public boolean isAvailable() {
        return lockServo != null;
    }

    /**
     * Locks the climbing mechanism.
     */
    public void lock() {
        if (lockServo == null) {
            return;
        }
        Protect.run(protect, telemetry, "Lock.lock", () -> lockServo.setPosition(1.0));
        isLocked = true;
    }

    /**
     * Unlocks the climbing mechanism.
     */
    public void unlock() {
        if (lockServo == null) {
            return;
        }
        Protect.run(protect, telemetry, "Lock.unlock", () -> lockServo.setPosition(0.0));
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
        if (lockServo == null) {
            return;
        }
        Protect.run(protect, telemetry, "Lock.setPos", () -> lockServo.setPosition(pos));
    }

}
