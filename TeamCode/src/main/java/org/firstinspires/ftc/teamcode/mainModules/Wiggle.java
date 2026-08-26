package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.util.Protect;

public class Wiggle {

    private Servo wiggleServo;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final boolean protect;

    public Wiggle(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapServo();
    }

    private void mapServo() {
        wiggleServo = Protect.map(protect, telemetry, HardwareConstants.WIGGLE_SERVO_MOTOR, () -> {
            Servo s = hardwareMap.get(Servo.class, HardwareConstants.WIGGLE_SERVO_MOTOR);
            s.setDirection(Servo.Direction.FORWARD);
            return s;
        });
    }

    /** True if this servo is actually in the configuration. */
    public boolean isAvailable() {
        return wiggleServo != null;
    }

    public void setPos(double pos) {
        if (wiggleServo == null) {
            return;
        }
        Protect.run(protect, telemetry, "Wiggle.setPos", () -> wiggleServo.setPosition(pos));
    }



}

