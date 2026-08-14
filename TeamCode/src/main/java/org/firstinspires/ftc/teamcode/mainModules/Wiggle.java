package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

public class Wiggle {

    private Servo wiggleServo;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public Wiggle(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapServo();
    }

    private void mapServo() {
        wiggleServo = hardwareMap.get(Servo.class, HardwareConstants.WIGGLE_SERVO_MOTOR);
        wiggleServo.setDirection(Servo.Direction.FORWARD);
    }

    public void setPos(double pos) {
        wiggleServo.setPosition(pos);
    }



}

