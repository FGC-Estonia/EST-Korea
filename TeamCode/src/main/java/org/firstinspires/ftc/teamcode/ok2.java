/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Robot: Teleop POV", group="Robot")

public class ok2 extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public Servo    nuga    = null;

    public DcMotor  Turbiin   = null;

    private IMU imu;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime pidTimer = new ElapsedTime();

    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.5 ;                 // sets rate to move servo

    @Override
    public void runOpMode() {
        double left= 0;
        double right= 0;
        double drive;
        double turn;
        double max;
        double error;
        double last_error = 0;
        double Kp = 0.01;
        double Ki = 0.00001;
        double Kd = 0.002;
        double Yaw;
        double derivative;
        double correction;
        double integral = 0;
        double kiirusKordaja = 0.5;
        long lastLeftPos, lastRightPos;
        long leftStartTicks, rightStartTicks;
        ArrayList<Double> logTime = new ArrayList<>();
        ArrayList<Double> logDrive = new ArrayList<>();
        ArrayList<Double> logSpeed = new ArrayList<>();

        double brakePower = 0.6;        // max reverse power at full trigger press, tune this
        double brakeDeadband = 20;      // ticks/sec below which we call it "stopped", tune from telemetry

        boolean useEncoderBias = false; // leave off until you've verified the sign below
        double Ke = 0.0002;             // keep tiny — this is a trim, not a primary controller

        double targetHeading = 0;
        double turnDeadband = 0.01;

        // Define and Initialize Motors
        leftDrive  = hardwareMap.get(DcMotor.class, "Motor_Port_3_CH");
        rightDrive = hardwareMap.get(DcMotor.class, "Motor_Port_2_CH");
        Turbiin = hardwareMap.get(DcMotor.class, "Motor_Port_1_CH");
        imu = hardwareMap.get(IMU.class, "imu");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        Turbiin.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        nuga  = hardwareMap.get(Servo.class, "Servo_Port_0_CH");

        nuga.setPosition(MID_SERVO);



        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();


        imu.resetYaw();
        timer.reset();
        pidTimer.reset();

        leftStartTicks = leftDrive.getCurrentPosition();
        rightStartTicks = rightDrive.getCurrentPosition();
        lastLeftPos = leftStartTicks;
        lastRightPos = rightStartTicks;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            double dt = pidTimer.seconds();
            pidTimer.reset();

            drive = -gamepad1.left_stick_y*kiirusKordaja;
            turn  = -gamepad1.right_stick_x*0.4;

            Yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            long leftPos = leftDrive.getCurrentPosition();
            long rightPos = rightDrive.getCurrentPosition();
            double leftVelocity = (dt > 0) ? (leftPos - lastLeftPos) / dt : 0;   // ticks/sec
            double rightVelocity = (dt > 0) ? (rightPos - lastRightPos) / dt : 0;
            lastLeftPos = leftPos;
            lastRightPos = rightPos;
            double currentSpeed = (Math.abs(leftVelocity) + Math.abs(rightVelocity)) / 2.0;

            logTime.add(timer.seconds());
            logDrive.add(drive);
            logSpeed.add(currentSpeed);

            double leftTrigger = gamepad1.left_trigger;
            boolean turningNow = Math.abs(turn) > turnDeadband;
            if (turningNow) {
                targetHeading = Yaw;   // let the target follow while you're actively turning
            }

            double turnScale = Range.clip(1.0 - Math.abs(drive), 0.3, 1.0);
            double scaledTurn = turn * turnScale;


            if (leftTrigger > 0.05) {
                double leftBrake = (Math.abs(leftVelocity) > brakeDeadband)
                        ? -Math.signum(leftVelocity) * brakePower * leftTrigger : 0;
                double rightBrake = (Math.abs(rightVelocity) > brakeDeadband)
                        ? -Math.signum(rightVelocity) * brakePower * leftTrigger : 0;

                leftDrive.setPower(leftBrake);
                rightDrive.setPower(rightBrake);

                integral = 0;
                last_error = -Yaw;
            }

            else if (Math.abs(drive) > 0.03 || turningNow){
                error = targetHeading-Yaw;

                if (turningNow) {
                    correction = 0;   // don't fight an intentional turn
                    integral = 0;
                    derivative = 0;
                } else {
                    integral += error * dt;
                    integral = Range.clip(integral, -20, 20);
                    derivative = (dt > 0) ? (error - last_error) / dt : 0;
                    correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
                }

                last_error = error;



                left = drive - correction - scaledTurn;
                right = drive + correction + scaledTurn;

                // Normalize the values so neither exceed +/- 1.0
                max = Math.max(Math.abs(left), Math.abs(right));
                if (max > 1.0)
                {
                    left /= max;
                    right /= max;
                }

                // Output the safe vales to the motor drives.
                leftDrive.setPower(left);
                rightDrive.setPower(right);
            }
            else{

                leftDrive.setPower(0);
                rightDrive.setPower(0);
                integral = 0;
                last_error = targetHeading-Yaw;
            }
            // Combine drive and turn for blended motion.


            if (gamepad1.square)
                Turbiin.setPower(1);

            else if (gamepad1.circle)
                Turbiin.setPower(0);

            if (gamepad1.right_bumper) {
                kiirusKordaja += 0.1;
                while (gamepad1.right_bumper){
                    sleep(1);
                }
            }
            else if (gamepad1.left_bumper) {
                kiirusKordaja -= 0.1;
                while (gamepad1.left_bumper){
                    sleep(1);
                }
            }

            kiirusKordaja = Math.max(Math.abs(kiirusKordaja), 0.1);
            kiirusKordaja = Math.min(Math.abs(kiirusKordaja), 1);


            // Use gamepad left & right Bumpers to SWING THE KNIFE
            if (gamepad1.dpad_right) {
                Kd += 0.0005;
                while (gamepad1.dpad_right){
                    sleep(1);
                }
            }
            else if (gamepad1.dpad_left) {
                Kd -= 0.0005;
                while (gamepad1.dpad_left){
                    sleep(1);
                }
            }

            // Send telemetry message to signify robot running;
            telemetry.addData("turnScale", turnScale);
            telemetry.addData("targetHeading", targetHeading);
            telemetry.addData("brake trigger", leftTrigger);
            telemetry.addData("left vel (ticks/s)", leftVelocity);
            telemetry.addData("right vel (ticks/s)", rightVelocity);
            telemetry.addData("vasak encoder",leftDrive.getCurrentPosition());
            telemetry.addData("parem encoder", rightDrive.getCurrentPosition());
            telemetry.addData("vasak stick", drive);
            telemetry.addData("Yaw", Yaw);
            telemetry.addData("Knife", nuga);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            //sleep(10);
        }

        try {
            File logFile = new File("/sdcard/FIRST/speed_log.csv");
            FileWriter writer = new FileWriter(logFile);
            writer.write("time_s,drive_commanded,speed_ticks_per_sec\n");
            for (int i = 0; i < logTime.size(); i++) {
                writer.write(logTime.get(i) + "," + logDrive.get(i) + "," + logSpeed.get(i) + "\n");
            }
            writer.close();
        } catch (IOException e) {
            telemetry.addData("Log write failed", e.getMessage());
            telemetry.update();
        }
    }
}