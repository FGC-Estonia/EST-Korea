package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.util.Protect;

public class Wink {

    private Servo winkServo;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final boolean protect;

    public Wink(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapServo();
    }

    private void mapServo() {
        winkServo = Protect.map(protect, telemetry, HardwareConstants.WINK_SERVO_MOTOR, () -> {
            Servo s = hardwareMap.get(Servo.class, HardwareConstants.WINK_SERVO_MOTOR);
            s.setDirection(Servo.Direction.FORWARD);
            return s;
        });
    }

    /** True if this servo is actually in the configuration. */
    public boolean isAvailable() {
        return winkServo != null;
    }

    public void setPos(double pos) {
        if (winkServo == null) {
            return;
        }
        Protect.run(protect, telemetry, "Wink.setPos", () -> winkServo.setPosition(pos));
    }



}

