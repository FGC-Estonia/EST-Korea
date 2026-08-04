package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

public class Wink {

    private Servo winkServo;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public Wink(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapServo();
    }

    private void mapServo() {
        winkServo = hardwareMap.get(Servo.class, HardwareConstants.WINK_SERVO_MOTOR);
        winkServo.setDirection(Servo.Direction.FORWARD);
    }

    public void setPos(double pos) {
        winkServo.setPosition(pos);
    }



}

