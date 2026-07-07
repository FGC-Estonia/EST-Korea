package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

public class CollectBalls {
    private DcMotorEx motor1 = null;
    private DcMotorEx motor2 = null;
    private final boolean protect;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final double LET_BALLS_OUT_RATE = 1.0;

    public CollectBalls(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapMotors();
    }

    private void mapMotors() {
        motor1 = hardwareMap.get(DcMotorEx.class, HardwareConstants.BALL_COLLECTOR_MOTOR_1);
        motor2 = hardwareMap.get(DcMotorEx.class, HardwareConstants.BALL_COLLECTOR_MOTOR_2);

        // One motor is reversed so they spin in opposite directions relative to each other
        // This is typical for dual-motor intakes (one on each side)
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void collectingBalls(int direction) {
        double power;

        if (direction == 1) {  // Suck in
            power = 1.0;
        } else if (direction == -1) {  // Let out
            power = -LET_BALLS_OUT_RATE;
        } else {
            power = 0;
        }

        motor1.setPower(power);
        motor2.setPower(power);
    }
}
