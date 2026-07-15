package org.firstinspires.ftc.teamcode;  //place where the code is located

/* Damn,

There is a huge bug in our code:
    ,__                   __
    '~~****Nm_    _mZ*****~~
            _8@mm@K_
           W~@`  '@~W
          ][][    ][][
    gz    'W'W.  ,W`W`    es
  ,Wf    gZ****MA****Ns    VW.
 gA`   ,Wf     ][     VW.   'Ms
Wf    ,@`      ][      '@.    VW
M.    W`  _mm_ ][ _mm_  'W    ,A
'W   ][  i@@@@i][i@@@@i  ][   W`
 !b  @   !@@@@!][!@@@@!   @  d!
  VWmP    ~**~ ][ ~**~    YmWf
    ][         ][         ][
  ,mW[         ][         ]Wm.
 ,A` @  ,gms.  ][  ,gms.  @ 'M.
 W`  Yi W@@@W  ][  W@@@W iP  'W
d!   'W M@@@A  ][  M@@@A W`   !b
@.    !b'V*f`  ][  'V*f`d!    ,@
'Ms    VW.     ][     ,Wf    gA`
  VW.   'Ms.   ][   ,gA`   ,Wf
   'Ms    'V*mmWWmm*f`    gA`
*/

/* ======================
   Imports (external modules & utilities)
   ====================== */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.util.ImuManager;
import org.firstinspires.ftc.teamcode.mainModules.ClimbPole;
import org.firstinspires.ftc.teamcode.mainModules.MoveRobot;
import org.firstinspires.ftc.teamcode.common.util.Presses;
import org.firstinspires.ftc.teamcode.mainModules.CollectBalls;
import org.firstinspires.ftc.teamcode.mainModules.FeedBalls;
import org.firstinspires.ftc.teamcode.mainModules.RaiseFlag;
import org.firstinspires.ftc.teamcode.mainModules.ThrowBalls;

import org.firstinspires.ftc.teamcode.common.util.DriveBaseController;
import static org.firstinspires.ftc.teamcode.mainModules.MoveRobot.DriveGear;
/* ======================
   Opmode annotation + class declaration
   ====================== */
@TeleOp(name = "Main code Estonia Korea")
// allows to display the code in the driver station, comment out to remove
public class EstoniaKorea extends LinearOpMode { //file name is EstoniaKorea.java    extends the prebuilt LinearOpMode by rev to run
    /* ======================
       Fields / State
       ====================== */
    int climbingDirection = 0; // 0 - stop, 1 - stay on rope, 2 - up, -1 - down, 3 - joystick

    // --- Subsystem instances for robot modules ---
    private RaiseFlag raiseFlag = null;      // Flag raising mechanism
    private ClimbPole climbRope = null;      // Pole climbing mechanism
    private CollectBalls collectBalls = null; // Ball intake mechanism
    private FeedBalls feedBalls = null;       // Ball feeding mechanism
    private ThrowBalls throwBalls = null;     // Ball launcher mechanism
    private DriveBaseController driveBase;    // Robot drivetrain control logic

    // Attachment flags
    private boolean ropeClimbingAttached = false;
    private boolean raiseFlagAttached = false;
    private boolean collectBallsAttached = false;
    private boolean feedBallsAttached = false;
    private boolean shootBallsAttached = false;
    private boolean spinWheelAttached = false;

    int[] lastDriveMotorPositions = {0, 0, 0, 0};
    private boolean isSpinningWheel = false;

    // Robot geometry / encoder constants
    private static final double TICKS_PER_REV = 560.0; // TICKS_PER_REV: encoder ticks per motor revolution
    private static final double WHEEL_DIAMETER = 0.09; // meters, replace with your wheel diameter
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // robot geometry for kinematics: half distances (meters) - replace with your robot measurements

    boolean fieldCentric = false;

    boolean isFlagRaised = false;

    int gear = 1;

    int collectingDirection = 0;
    /* ======================
       Main opmode loop
       ====================== */
    @Override
    public void runOpMode() throws InterruptedException {
        boolean protect = true;

        // --- Drive base init ---
        driveBase = new MoveRobot(protect, hardwareMap, telemetry, true);

        // --- Core managers & modules initialization ---
        ImuManager imuManager = new ImuManager(protect, hardwareMap, telemetry, true);

        // --- Try to attach optional modules (safe to fail) ---

        try {
            climbRope = new ClimbPole(protect, hardwareMap, telemetry);
            ropeClimbingAttached = true;
        } catch (Exception e) {
            telemetry.log().add("ClimbRope hardware not found — rope climb disabled");
        }

        try {
            raiseFlag = new RaiseFlag(hardwareMap, telemetry);
            raiseFlagAttached = true;
        } catch (Exception e) {
            telemetry.log().add("RaiseFlag hardware not found — flag raising disabled");
        }

        try {
            collectBalls = new CollectBalls(protect, hardwareMap, telemetry);
            collectBallsAttached = true;
        } catch (Exception e) {
            telemetry.log().add("Collecting balls hardware not found — collecting balls disabled");
        }

        try {
            feedBalls = new FeedBalls(protect, hardwareMap, telemetry);
            feedBallsAttached = true;
        } catch (Exception e) {
            telemetry.log().add("Feeder hardware not found — feeding balls disabled");
        }

        try {
            throwBalls= new ThrowBalls(protect, hardwareMap, telemetry);
            spinWheelAttached = true;
        } catch (Exception e) {
            telemetry.log().add("SpinWheel hardware not found — spinning wheel disabled");
        }

         /* ======================
           Controls: Presses wrappers and toggles
           ====================== */

        // Unused controls reserved for future use
        Presses gamepad1_left_trigger = new Presses();
        Presses gamepad1_right_trigger = new Presses();

        Presses gamepad1_dpad_left = new Presses();
        Presses gamepad1_dpad_right = new Presses();
        Presses gamepad1_dpad_up = new Presses();
        Presses gamepad2_dpad_up = new Presses();

        Presses gamepad2_left_bumper = new Presses();
        // Drive speed toggle group (replaced by drivegear)
        Presses.ToggleGroup feedClearToggle = new Presses.ToggleGroup();
        Presses gamepad2_cross = new Presses(feedClearToggle);
        Presses gamepad2_dpad_down = new Presses(feedClearToggle);
        // Controls for rope climbing
        Presses gamepad2_triangle = new Presses();
        // Also using:
        // > gamepad2.left_bumper   - climb up
        // > gamepad2.right_bumper  - climb down
        // > gamepad2.left_stick_y  - manual joystick control (when abs > 0.05)
        // Resetting ropeclimb position:
        Presses gamepad2_dpad_left = new Presses();
        Presses gamepad2_dpad_right = new Presses();

        
        // Controls for drive gear
        Presses gamepad1_right_bumper = new Presses();
        Presses gamepad1_left_bumper = new Presses();

        // Controls for flag raising
        Presses gamepad2_share = new Presses();


        Presses gamepad2_circle = new Presses();

        // Controls for Throwing balls
        Presses gamepad2_square = new Presses();

        // Controls for fieldcentric toggle and gyro reset
        Presses gamepad1_share = new Presses();
        Presses gamepad1_options = new Presses();


        telemetry.update();
        waitForStart(); //everything has been initialized, waiting for the start button
        while (opModeIsActive()) { // main loop

            //gyro reset
            if (gamepad1.options) {
                imuManager.resetImu();
            }

            //move robot
            double imuAngle = imuManager.getYawRadians();
            double imuPitch = imuManager.getPitchRadians();
            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // FieldCentric rumble
            if (gamepad1_share.pressed(gamepad1.share)) {
                fieldCentric = !fieldCentric;
                if (fieldCentric) {
                    // One long 500 ms rumble when turning ON
                    gamepad1.rumble(1.0, 1.0, 500);
                } else {
                    // Two short 100 ms rumbles when turning OFF
                    gamepad1.rumble(0.6, 0.6, 100);
                }
            }


            // --- POLE CLIMBING LOGIC ---
            // Handles logic for holding, climbing up/down, or manual joystick control
            boolean holdingOnRope = gamepad2_triangle.toggle(gamepad2.triangle);

            if (holdingOnRope) {
                climbingDirection = 1;  // hold position
            } else if (gamepad2.left_bumper) {
                climbingDirection = 2;  // climb up
            } else if (gamepad2.right_bumper) {
                climbingDirection = -1; // climb down
            } else if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                climbingDirection = 3;
            } else {
                climbingDirection = 0;
            }
            // else: do nothing, keep previous direction (motor holds position)

            // Apply motor control
            if (ropeClimbingAttached) {
                climbRope.ropeClimbing(climbingDirection, -gamepad2.left_stick_y);
            }
            if (gamepad2_dpad_right.pressed(gamepad2.dpad_right) && ropeClimbingAttached) {
                climbRope.rememberHomePosition();
            }

            if (gamepad2_dpad_left.pressed(gamepad2.dpad_left) && ropeClimbingAttached) {
                climbRope.rotateToHome();
            }


            // --- COLLECTING BALLS LOGIC ---
            // Triggers control sucking in or letting out balls
            if (gamepad2.right_trigger > 0) {
                collectingDirection = -1;  // suck inj
            } else if (gamepad2.left_trigger > 0) {
                collectingDirection = 1; // let out
            } else {
                collectingDirection = 0;  // hold
            }

            // Apply motor control
            if (collectBallsAttached) {
                collectBalls.collectingBalls(collectingDirection);
            }


            // --- FEED BALLS LOGIC ---
            // Toggles the feeder mechanism to move balls to the launcher
            boolean isFeeding = gamepad2_cross.toggle(gamepad2.cross);
            boolean isClearing = gamepad2_dpad_down.toggle(gamepad2.dpad_down);
            telemetry.addData("isFeeding", isFeeding);
            telemetry.addData("isClearing", isClearing);
            if (feedBallsAttached) {
                if (isFeeding) {
                    feedBalls.feed(true);
                } else if (isClearing){
                    feedBalls.clear(true);
                } else {
                    feedBalls.stop();
                }
            }

            // --- THROW BALLS LOGIC ---
            // Toggles the spinning launcher wheel
            isSpinningWheel = gamepad2_square.toggle(gamepad2.square);
            if (spinWheelAttached) {
                if (isSpinningWheel) {
                    throwBalls.spin(true);
                } else {
                    throwBalls.stop();
                }
            }
            if (throwBalls.throwSpeed() > 1700) {
                gamepad2.rumble(250);
            }


            // --- FLAG RAISING LOGIC ---
            // Deploys the flag
            boolean needFlagRaised = gamepad2_share.toggle(gamepad2.share);
            if (needFlagRaised && !isFlagRaised && raiseFlagAttached) {
                raiseFlag.setPos(1);
                isFlagRaised = true;
            } else if (!needFlagRaised && isFlagRaised && raiseFlagAttached) {
                raiseFlag.setPos(0);
                isFlagRaised = false;
            }

            telemetry.addData("ViskeKeerutikiirus", throwBalls.throwSpeed());
            telemetry.addData("Field Centric", fieldCentric);
            telemetry.addData("Heading", imuAngle * 180 / 3.14159265358979323);
            /* ======================
               Drive gears: read bumpers to increment/decrement gear
               - clamps gear between 1 and 3 and maps to DriveGear enum
               ====================== */
            DriveGear currentDriveGear = DriveGear.LOW;

            if (gamepad1_left_bumper.released(gamepad1.left_bumper) && gear >= 2) {
                gear -= 1;
            } else if (gamepad1_right_bumper.released(gamepad1.right_bumper) && gear <= 2) {
                gear += 1;
            }
            telemetry.addData("Gear", gear);

            if (gear == 1) {
                currentDriveGear = DriveGear.LOW;
            } else if (gear == 2) {
                currentDriveGear = DriveGear.MEDIUM;
            } else if (gear == 3) {
                currentDriveGear = DriveGear.HIGH;
            }

            telemetry.addData("drive",drive);
            telemetry.addData("strafe", strafe);
            telemetry.addData("turn", turn);
            driveBase.drive (
                    imuAngle, imuPitch,
                    drive, turn, strafe,
                    fieldCentric, currentDriveGear
            );
            telemetry.update();
        } // This brace correctly closes the `while (opModeIsActive())` loop.
    } // This brace correctly closes the `runOpMode()` method.

    /* ======================
       Helper utilities
       ====================== */

    // helper to convert encoder ticks -> linear distance (meters) ----
    private double ticksToDistance(int ticks) {
        return ticks * (WHEEL_CIRCUMFERENCE / TICKS_PER_REV);
    }

} // This brace correctly closes the `EstoniaKorea` class.