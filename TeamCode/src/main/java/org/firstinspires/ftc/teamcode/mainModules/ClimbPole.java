package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.util.Protect;

/**
 * Module for controlling the robot's pole climbing mechanism.
 * Uses a single DC motor to pull the robot up or lower it down.
 */
public class ClimbPole {
    private DcMotorEx climbMotor = null;
    private final boolean protect;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final double CLIMB_UP_POWER = -1.0;
    private final double CLIMB_DOWN_POWER = 0.5;
    private final double STAY_ON_RATE = 0.1;

    public ClimbPole(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapMotors();
    }

    private void mapMotors() {
        climbMotor = Protect.map(protect, telemetry, HardwareConstants.CLIMB_MOTOR, () -> {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, HardwareConstants.CLIMB_MOTOR);
            m.setDirection(DcMotor.Direction.REVERSE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return m;
        });
    }

    /** True if the climb motor is actually in the configuration. */
    public boolean isAvailable() {
        return climbMotor != null;
    }

    /**
      Controls the climbing motor based on the given direction and joystick input.
     * 
     * @param direction 0: stop, 1: hold position, 2: up, -1: down, 3: joystick control
     * @param stick Manual joystick power adjustment (expected to be -gamepad.left_stick_y)
     */
    public void ropeClimbing(int direction, float stick) {
        double power;

        if (direction == 2) {  // Climb up
            power = CLIMB_UP_POWER;
        } else if (direction == -1) {  // Climb down
            power = CLIMB_DOWN_POWER;
        } else if (direction == 1) { // Stay on rope
            power = -STAY_ON_RATE;
        } else if (direction == 3) { // Joystick control
            power = -stick;
        } else {
            power = 0;
        }

        if (climbMotor == null) {
            return;
        }
        final double motorPower = power;
        Protect.run(protect, telemetry, "ClimbPole.ropeClimbing",
                () -> climbMotor.setPower(motorPower));
    }

}
