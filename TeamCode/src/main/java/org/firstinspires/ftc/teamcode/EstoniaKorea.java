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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.mainModules.ClimbPole;
import org.firstinspires.ftc.teamcode.mainModules.MoveRobot;
import org.firstinspires.ftc.teamcode.common.util.Presses;
import org.firstinspires.ftc.teamcode.mainModules.CollectBalls;
import org.firstinspires.ftc.teamcode.mainModules.FeedBalls;
import org.firstinspires.ftc.teamcode.mainModules.Lock;
import org.firstinspires.ftc.teamcode.mainModules.ExpandStorage;
import org.firstinspires.ftc.teamcode.mainModules.Wiggle;
import org.firstinspires.ftc.teamcode.mainModules.BuddyClimb;
import org.firstinspires.ftc.teamcode.mainModules.ThrowBalls;

import org.firstinspires.ftc.teamcode.mainModules.Alignment;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.mainModules.MoveRobot.DriveGear;
/* ======================
   Op-mode annotation + class declaration
   ====================== */
@TeleOp(name = "Main code Estonia Korea")
// allows to display the code in the driver station, comment out to remove
public class EstoniaKorea extends LinearOpMode { //file name is EstoniaKorea.java    extends the prebuilt LinearOpMode by rev to run
    /* ======================
       Fields / State
       ====================== */
    int climbingDirection = 0; // 0 - stop, 1 - stay on rope, 2 - up, -1 - down, 3 - joystick

    // --- Subsystem instances for robot modules ---
    private Lock lock = null;      // Climbing lock mechanism
    private ExpandStorage expandStorage = null;
    private Wiggle wiggle = null;
    private BuddyClimb buddyClimb = null;
    private ClimbPole climbRope = null;      // Pole climbing mechanism
    private CollectBalls collectBalls = null; // Ball intake mechanism
    private FeedBalls feedBalls = null;       // Ball feeding mechanism
    private ThrowBalls throwBalls = null;     // Ball launcher mechanism
    private MoveRobot driveBase;    // Robot drivetrain control logic
    private ImuManager imuManager;
    private VoltageSensor myControlHubVoltageSensor;
    private Alignment alignment;
    private ElapsedTime wiggleTimer = new ElapsedTime();


    private boolean alignmentAttached = false;
    // Attachment flags
    private boolean ropeClimbingAttached = false;
    private boolean lockAttached = false;
    private boolean buddyClimbed = true;
    private boolean expandStoraged = false;
    private boolean wiggled = false;
    private boolean buddiesClimbed = false;
    private boolean collectBallsAttached = false;
    private boolean feedBallsAttached = false;
    private boolean shootBallsAttached = false;
    private boolean spinWheelAttached = false;
    private boolean driveBaseAttached = false;
    private boolean imuManagerAttached = false;
    private int wigglePos = 0;

    int[] lastDriveMotorPositions = {0, 0, 0, 0};
    private boolean isSpinningWheel = false;
    private boolean recordingLowest = false;


    // Robot geometry / encoder constants
    private static final double TICKS_PER_REV = 560.0; // TICKS_PER_REV: encoder ticks per motor revolution
    private static final double WHEEL_DIAMETER = 0.09; // meters, replace with your wheel diameter
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // robot geometry for kinematics: half distances (meters) - replace with your robot measurements
    boolean fieldCentric = false;

    int gear = 1;

    int collectingDirection = 0;
    /* ======================
       Main opmode loop
       ====================== */
    @Override
    public void runOpMode() throws InterruptedException {
        boolean protect = true;

        try {
            alignment = new Alignment(true, hardwareMap, telemetry, gamepad1, gamepad2);
            alignmentAttached = true;
        } catch (Exception e) {
            telemetry.log().add("Alignment hardware not found — alignment disabled");
        }

        // --- Drive base init ---
        try {
            driveBase = new MoveRobot(protect, hardwareMap, telemetry, true);
            driveBaseAttached = true;
        } catch (Exception e) {
            telemetry.log().add("DriveBase hardware not found — drive disabled");
        }

        // --- Core managers & modules initialization ---
        try {
            imuManager = new ImuManager(protect, hardwareMap, telemetry);
            imuManagerAttached = true;
        } catch (Exception e) {
            telemetry.log().add("IMU hardware not found — field centric disabled");
        }

        // --- Try to attach optional modules (safe to fail) ---

        try {
            climbRope = new ClimbPole(protect, hardwareMap, telemetry);
            ropeClimbingAttached = true;
        } catch (Exception e) {
            telemetry.log().add("ClimbRope hardware not found — rope climb disabled");
        }

        try {
            lock = new Lock(hardwareMap, telemetry);
            lockAttached = true;
        } catch (Exception e) {
            telemetry.log().add("Lock hardware not found — climbing lock disabled");
        }
        try {
            expandStorage = new ExpandStorage(hardwareMap, telemetry);
            expandStoraged = true;
        } catch (Exception e) {
            telemetry.log().add("Expanding not found — Expanding disabled");
        }
        try {
            wiggle = new Wiggle(hardwareMap, telemetry);
            wiggled = false;
        } catch (Exception e) {
            telemetry.log().add("Wiggling not found — Wiggle disabled");
        }

        try {
            buddyClimb = new BuddyClimb(hardwareMap, telemetry);
            buddiesClimbed = true;
        } catch (Exception e) {
            telemetry.log().add("Helper servo not found — Buddy climb disabled");
        }

        try {
            buddyClimb = new BuddyClimb(hardwareMap, telemetry);
            buddyClimbed = true;
        } catch (Exception e) {
            telemetry.log().add("Servo for helping not found — Buddy climb disabled");
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

        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

         /* ======================
           Controls: Presses wrappers and toggles
           ====================== */

        Presses gamepad1_dpad_left = new Presses();

        Presses.ToggleGroup feedClearToggle = new Presses.ToggleGroup();
        Presses gamepad2_dpad_down = new Presses(feedClearToggle);
        Presses gamepad2_dpad_up = new Presses(feedClearToggle);
        // Controls for rope climbing
        Presses gamepad2_triangle = new Presses();
        // Also using:
        // > gamepad2.left_bumper   - climb up
        // > gamepad2.right_bumper  - climb down
        // > gamepad2.left_stick_y  - manual joystick control (when abs > 0.05)

        
        // Controls for drive gear
        Presses gamepad1_right_bumper = new Presses();
        Presses gamepad1_left_bumper = new Presses();

        // Controls for climbing lock
        Presses gamepad2_share = new Presses();

        Presses gamepad2_dpad_left = new Presses();

        Presses gamepad2_dpad_right = new Presses();

        Presses gamepad2_touchpad = new Presses();
        // Controls for Lock
        Presses gamepad2_square = new Presses();

        // Controls for field-centric toggle and gyro reset
        Presses gamepad1_share = new Presses();
        Presses gamepad1_options = new Presses();

        // Wiggle
        Presses gamepad2_options = new Presses();

        telemetry.update();
        waitForStart(); //everything has been initialized, waiting for the start button
        while (opModeIsActive()) { // main loop

            //gyro reset
            if (gamepad1.options && imuManagerAttached) {
                imuManager.resetImu();
            }

            //move robot
            double imuAngle = 0;
            if (imuManagerAttached) {
                org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles angles = imuManager.getAngles();
                imuAngle = angles.getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);
            }
            
            double currentDistance = 1000.0;
            if (alignmentAttached) {
                currentDistance = alignment.getDistance();
            }

            double drive;
            if (gamepad1.right_trigger > 0.2 && alignmentAttached){
                drive = -gamepad1.left_stick_y + alignment.alignTarget(currentDistance);
            } else {
                drive = -gamepad1.left_stick_y;
            }
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

            if (lockAttached && lock.isLocked()) {
                climbingDirection = 0;
            } else if (holdingOnRope) {
                climbingDirection = 1;  // hold position
            } else if (gamepad2.left_bumper) {
                climbingDirection = 2;  // climb up
            } else if (gamepad2.right_bumper) {
                climbingDirection = -1; // climb down
            } else if (Math.abs(gamepad2.right_stick_y) > 0.05) {
                climbingDirection = 3;
            } else {
                climbingDirection = 0;
            }
            // else: do nothing, keep previous direction (motor holds position)

            // Apply motor control
            if (ropeClimbingAttached) {
                climbRope.ropeClimbing(climbingDirection, -gamepad2.right_stick_y);
            }

            // EXPANDING STORAGE
            boolean storageExpansion = gamepad2_dpad_right.toggle(gamepad2.dpad_right);
            if (storageExpansion && !expandStoraged) {
                expandStorage.setPos(1);
                expandStoraged = true;
            } else if (!storageExpansion && expandStoraged) {
                expandStorage.setPos(0);
                expandStoraged = false;
            }
            // WIGGLING
            boolean wiggleds = gamepad2_touchpad.toggle(gamepad2.touchpad);
            if (wiggleds && !wiggled) {
                wiggled = true;
                wiggle.setPos(1);
                wigglePos = 1;
                wiggleTimer.reset();
            } else if (!wiggleds && wiggled) {
                wiggled = false;
                wiggle.setPos(0);
            }
            if (wiggled) {
                if (wiggleTimer.milliseconds() > 690) {
                    wigglePos = (wigglePos == 0) ? 1 : 0;
                    wiggle.setPos(wigglePos);
                    wiggleTimer.reset();
                }
            }

            // BUDDY CLIMBING
            boolean buddyClimbed = gamepad1_dpad_left.toggle(gamepad1.dpad_left);
            if (buddiesClimbed && !buddyClimbed) {
                buddyClimb.setPos(0);
                buddyClimbed = true;
            } else if (!buddiesClimbed && buddyClimbed) {
                buddyClimb.setPos(1);
                buddyClimbed = false;
            }


            // --- COLLECTING BALLS LOGIC ---
            // Triggers control sucking in or letting out balls
            if (gamepad2.right_trigger > 0) {
                collectingDirection = -1;  // suck in
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
            boolean isFeeding = gamepad2_dpad_down.toggle(gamepad2.dpad_down);
            boolean isClearing = gamepad2_dpad_up.toggle(gamepad2.dpad_up);
            telemetry.addData("isFeeding", isFeeding);
            telemetry.addData("isClearing", isClearing);
            if (feedBallsAttached) {
                // Only feed if the flywheel is at the right speed
                boolean atSpeed = throwBalls.isAtSpeed();
                if (isFeeding && atSpeed) {
                    feedBalls.feed(true);
                    if (isClearing) {
                        gamepad2_dpad_down.setToggleFalse();
                        feedBalls.feed(false);
                        feedBalls.clear(true);
                    }
                } else if (isClearing){
                    feedBalls.clear(true);
                    if (isFeeding) {
                        feedBalls.clear(false);
                        gamepad2_dpad_up.setToggleFalse();
                        feedBalls.feed(true);
                    }
                } else {
                    feedBalls.stop();
                }
            }

            // --- THROW BALLS LOGIC ---
            // Toggles the spinning launcher wheel
            isSpinningWheel = gamepad2_dpad_left.toggle(gamepad2.dpad_left);
            telemetry.addData("isSpinningWheel", isSpinningWheel);
            double currentThrowSpeed = 0;
            if (spinWheelAttached) {
                if (isSpinningWheel) {
                    throwBalls.spin(true);
                } else {
                    throwBalls.stop();
                }

                currentThrowSpeed = throwBalls.throwSpeed();
                if (currentThrowSpeed > 1700) {
                    gamepad2.rumble(250);
                }
            }


            // --- CLIMBING LOCK LOGIC ---
            // Deploys the lock
            boolean needLocked = gamepad2_square.toggle(gamepad2.square);
            if (lockAttached) {
                if (needLocked && !lock.isLocked()) {
                    lock.lock();
                    gamepad2.rumble(25);
                } else if (!needLocked && lock.isLocked()) {
                    lock.unlock();
                }
            }

            telemetry.addData("ViskeKeerutikiirus", spinWheelAttached ? currentThrowSpeed : "N/A");
            telemetry.addData("Field Centric", fieldCentric);
            telemetry.addData("Heading (Deg)", Math.toDegrees(imuAngle));
            if (isSpinningWheel) {
                double lowestSpeed = Double.MAX_VALUE;
                if (!recordingLowest && currentThrowSpeed > 2675) {
                    recordingLowest = true;
                    lowestSpeed = currentThrowSpeed;
                }
                if (recordingLowest) {
                    if (currentThrowSpeed < lowestSpeed) {
                        lowestSpeed = currentThrowSpeed;
                    }
                    telemetry.addData("Lowest Speed", lowestSpeed);
                }
            } else {
                recordingLowest = false;
            }
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

            if (alignmentAttached) {
                if (currentDistance < 900) {
                    telemetry.addData("Current distance: ", currentDistance);
                    if (currentDistance < alignment.TARGET + alignment.TOLERANCE && currentDistance > alignment.TARGET - alignment.TOLERANCE) {
                        gamepad1.rumble(25);
                        gamepad2.rumble(25);
                    }
                }
            }

            double presentVoltage;
            presentVoltage = myControlHubVoltageSensor.getVoltage();

            telemetry.addData("drive",drive);
            telemetry.addData("strafe", strafe);
            telemetry.addData("turn", turn);
            if (driveBaseAttached) {

                double compensation = 12.0 / presentVoltage;
                driveBase.move(imuAngle, drive * compensation, strafe * compensation, turn * compensation, fieldCentric, currentDriveGear);
            }
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