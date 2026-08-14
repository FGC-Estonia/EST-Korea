package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

public class BuddyClimb {
    private Servo buddyServo;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private void mapServo() {
        buddyServo = hardwareMap.get(Servo.class, HardwareConstants.BUDDY_SERVO_MOTOR);
        buddyServo.setDirection(Servo.Direction.FORWARD);
    }

    public BuddyClimb(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapServo();
    }
    public void setPos(double pos) {
            buddyServo.setPosition(pos);
        }
    }

