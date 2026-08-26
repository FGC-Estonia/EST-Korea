package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.util.Protect;

/**
 * Module for intake and collection of balls.
 * Uses a motor to suck balls into the robot or push them out - not used
 */
public class CollectBalls {
    private DcMotorEx motor = null;
    private Servo servoL = null;
    private Servo servoR = null;
    private final boolean protect;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final double LET_BALLS_OUT_RATE = 1.0;

    /**
     * Initializes the collection module.
     */
    public CollectBalls(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapMotors();
    }

    private void mapMotors() {
        // Each device is mapped on its own, so a missing reach servo costs us that
        // servo and nothing else - the intake motor still runs.
        motor = Protect.map(protect, telemetry, HardwareConstants.BALL_COLLECTOR_MOTOR, () -> {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, HardwareConstants.BALL_COLLECTOR_MOTOR);
            m.setDirection(DcMotor.Direction.REVERSE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return m;
        });

        servoL = Protect.map(protect, telemetry, HardwareConstants.REACHL_SERVO_MOTOR, () -> {
            Servo s = hardwareMap.get(Servo.class, HardwareConstants.REACHL_SERVO_MOTOR);
            s.setDirection(Servo.Direction.FORWARD);
            return s;
        });

        servoR = Protect.map(protect, telemetry, HardwareConstants.REACHR_SERVO_MOTOR, () -> {
            Servo s = hardwareMap.get(Servo.class, HardwareConstants.REACHR_SERVO_MOTOR);
            s.setDirection(Servo.Direction.REVERSE);
            return s;
        });
    }

    /** True if the intake motor is there. The reach servos are optional extras. */
    public boolean isAvailable() {
        return motor != null;
    }

    /**
     * Sets the collector motor power based on the desired direction.
     *
     * @param direction 1: suck in, -1: let out, 0: stop
     */
    public void collectingBalls(int direction) {
        double power;
        double servoDirection;

        if (direction == 1) {  // Suck in
            power = 1.0;
            servoDirection = 1.0;
        } else if (direction == -1) {  // Let out
            power = -LET_BALLS_OUT_RATE;
            servoDirection = 0;
        } else {
            power = 0;
            servoDirection = 0.5;
        }

        final double motorPower = power;
        final double servoPos = servoDirection;
        if (motor != null) {
            Protect.run(protect, telemetry, "CollectBalls.motor", () -> motor.setPower(motorPower));
        }
        if (servoL != null) {
            Protect.run(protect, telemetry, "CollectBalls.servoL", () -> servoL.setPosition(servoPos));
        }
        if (servoR != null) {
            Protect.run(protect, telemetry, "CollectBalls.servoR", () -> servoR.setPosition(servoPos));
        }
    }
}