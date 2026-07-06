package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

public class CollectBalls {
    private DcMotorEx motor = null;
    private DcMotor motor1 = null;

    private DcMotor motor2 = null;

    private final boolean protect;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final double LET_BALLS_OUT_RATE = 1;

    public CollectBalls(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapMotors();
    }

    private void mapMotors() {
        motor = hardwareMap.get(DcMotorEx.class, HardwareConstants.BALL_COLLECTOR_MOTOR);
        motor1 = hardwareMap.get(DcMotor.class, "Motor_Port_1_CH");
        motor2 = hardwareMap.get(DcMotor.class, "Motor_Port_2_CH");

        motor.setDirection(DcMotor.Direction.FORWARD);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // mirror settings from your example
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    public void collectingBalls(int direction) {
        double power;

        if (direction == 1) {  // Collect Balls
            power = 1.0;
        } else if (direction == -1) {  // Let Balls out slowly
            power = -LET_BALLS_OUT_RATE;
        } else {
            power = 0;
        }

        motor.setPower(power);
        motor1.setPower(power);
        motor2.setPower(power);
    }
}