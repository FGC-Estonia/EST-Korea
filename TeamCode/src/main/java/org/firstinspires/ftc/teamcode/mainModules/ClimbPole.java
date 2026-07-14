package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;

/**
 * Module for controlling the robot's pole climbing mechanism.
 * Uses a single DC motor to pull the robot up or lower it down.
 */
public class ClimbPole {
    private DcMotorEx climbMotor = null;
    private final boolean protect;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final double CLIMB_DOWN_RATE = 0.2;
    private final double STAY_ON_RATE = 0.1;
    boolean resettingPosition = false;
    private int storedHomePositionTicks = 0;
    // Define constants for your climbMotor and gearing
    private static final double REV_HD_HEX_MOTOR_TICKS_PER_MOTOR_REV = 28.0; // From REV HD Hex Motor spec
    private static final double GEAR_RATIO_1 = 5.23;
    private static final double GEAR_RATIO_2 = 3.61;
    private static final double GEAR_RATIO_3 = 3.61;

    // Calculate the total ticks per revolution for the final output shaft
    private static final double TOTAL_GEAR_RATIO = GEAR_RATIO_1 * GEAR_RATIO_2 * GEAR_RATIO_3;
    private static final int TICKS_PER_OUTPUT_SHAFT_REVOLUTION = (int) (REV_HD_HEX_MOTOR_TICKS_PER_MOTOR_REV * TOTAL_GEAR_RATIO);

    public ClimbPole(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapMotors();
    }

    private void mapMotors() {
        climbMotor = hardwareMap.get(DcMotorEx.class, HardwareConstants.ROPE_MOTOR);

        climbMotor.setDirection(DcMotor.Direction.REVERSE);

        climbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        climbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    // ROTATING TO POS DOESN'T REALLY WORK
    public void rotateToHome() {
        int currentFullPosition = climbMotor.getCurrentPosition();
        int currentAngularOffset = currentFullPosition % TICKS_PER_OUTPUT_SHAFT_REVOLUTION;

        // Handle negative currentAngularOffset
        if (currentAngularOffset < 0) {
            currentAngularOffset += TICKS_PER_OUTPUT_SHAFT_REVOLUTION;
        }

        // Calculate the difference needed to reach the redDotHomeAngularOffset
        int deltaAngle = storedHomePositionTicks - currentAngularOffset;

        // Adjust deltaAngle for the shortest path around the circle
        if (deltaAngle > TICKS_PER_OUTPUT_SHAFT_REVOLUTION / 2) {
            deltaAngle -= TICKS_PER_OUTPUT_SHAFT_REVOLUTION;
        } else if (deltaAngle < -TICKS_PER_OUTPUT_SHAFT_REVOLUTION / 2) {
            deltaAngle += TICKS_PER_OUTPUT_SHAFT_REVOLUTION;
        }

        // The new target is the current full position plus this small angular adjustment
        int targetFullPosition = currentFullPosition + deltaAngle;

        rotateToPosition(targetFullPosition);
    }
    public void rotateToPosition(int targetTicks) {
        // This tells the climbMotor controller to use the encoders to reach a specific target.
        climbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // The motors will try to reach this absolute encoder count.
        climbMotor.setTargetPosition(targetTicks);

        // This power (from 0.0 to 1.0) acts as the maximum speed the motors will use
        climbMotor.setPower(1.0); // Full power for movement

        // Wait until the motors reach their target or a timeout occurs.
        long startTime = System.currentTimeMillis();
        long timeoutMillis = 10000; // 10 seconds timeout.

        // Loop while the OpMode is active, at least one motor is still moving, AND the timeout hasn't been reached.
        while ((climbMotor.isBusy()) && (System.currentTimeMillis() - startTime < timeoutMillis)) {
            // Provide real-time feedback on the Driver Station for debugging.
            telemetry.addData("Left Target", climbMotor.getTargetPosition());
            telemetry.update();
        }

        // Stop the motors once they reach the target or the loop exits (e.g., due to timeout).
        climbMotor.setPower(0);

        // Set motors back to RUN_WITHOUT_ENCODER mode.
        climbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public int rememberHomePosition(){
        storedHomePositionTicks = climbMotor.getCurrentPosition() % TICKS_PER_OUTPUT_SHAFT_REVOLUTION;
        if (storedHomePositionTicks < 0) {
            storedHomePositionTicks += TICKS_PER_OUTPUT_SHAFT_REVOLUTION;
        }
    return storedHomePositionTicks;
    }

    /**
      Controls the climbing motor based on the given direction and joystick input.
     * 
     * @param direction 0: stop, 1: hold position, 2: up, -1: down, 3: joystick control
     * @param stick Manual joystick power adjustment
     */
    public void ropeClimbing(int direction, float stick) {
        if (resettingPosition) {return;}

        double power;

        if (direction == 2) {  // Climb up
            power = 1.0;
        } else if (direction == -1) {  // Climb down slowly
            power = -CLIMB_DOWN_RATE;
        } else if (direction == 1) { // Stay on rope
            power = STAY_ON_RATE;
        } else if (direction == 3) { // Joystick control
            power = (stick + 1) * 0.6 - 0.2;
        } else {
            power = 0;
        }

        climbMotor.setPower(power);
    }

}
