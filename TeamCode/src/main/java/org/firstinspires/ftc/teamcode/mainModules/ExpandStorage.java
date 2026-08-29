package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

public class ExpandStorage {

    private Servo expandstorageServo;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public ExpandStorage(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapServo();
    }

    private void mapServo() {
        expandstorageServo = hardwareMap.get(Servo.class, HardwareConstants.EXPAND_STORAGE_SERVO_MOTOR);
        expandstorageServo.setDirection(Servo.Direction.FORWARD);
    }

    public void setPos(double pos) {
        expandstorageServo.setPosition(pos);
    }
}

