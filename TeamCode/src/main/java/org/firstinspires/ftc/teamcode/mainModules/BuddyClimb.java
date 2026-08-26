package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.util.Protect;

public class BuddyClimb {
    private Servo buddyServo;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final boolean protect;

    private void mapServo() {
        buddyServo = Protect.map(protect, telemetry, HardwareConstants.BUDDY_SERVO_MOTOR, () -> {
            Servo s = hardwareMap.get(Servo.class, HardwareConstants.BUDDY_SERVO_MOTOR);
            s.setDirection(Servo.Direction.FORWARD);
            return s;
        });
    }

    /** True if this servo is actually in the configuration. */
    public boolean isAvailable() {
        return buddyServo != null;
    }

    public BuddyClimb(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapServo();
    }
    public void setPos(double pos) {
        if (buddyServo == null) {
            return;
        }
        Protect.run(protect, telemetry, "BuddyClimb.setPos", () -> buddyServo.setPosition(pos));
    }
    }

