package org.firstinspires.ftc.teamcode.mainModules;  //place where the code is located

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.util.Protect;

public class MoveRobot {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private final boolean useVelocity;
    private final boolean protect;

    double MAX_ANGULAR_VELOCITY_RADIANS = 1972.92;

    double wantedAngle = 0;

    double maxSpeed = 1;

    /* ======================
       Encoder health
       ----------------------
       In RUN_USING_ENCODER the motor controller closes a speed loop around the
       encoder. Pull that encoder cable and the controller reads zero speed
       forever, so it winds the power up to maximum trying to get there - the
       wheel runs away and the robot is undriveable.

       So each wheel watches itself: if it is being told to move and its encoder
       count has not budged for a while, the encoder is treated as dead, that one
       wheel drops to open-loop power control, and everything keeps driving. It is
       per wheel, so three good encoders keep their closed loop.
       ====================== */

    /** Below this commanded fraction we are not really asking the wheel to move. */
    public static final double ENCODER_CHECK_MIN_COMMAND = 0.25;

    /** Ticks the encoder must move within the window to be considered alive. */
    public static final int ENCODER_ALIVE_TICKS = 5;

    /** How long a wheel may be driven with a frozen encoder before we give up on it. */
    public static final double ENCODER_DEAD_MS = 400;

    /**
     * How often the encoder health check reads positions, milliseconds.
     *
     * <p>Reading a position is a round trip to the hub. Doing that for four
     * wheels on every single loop is enough to put a visible delay in the
     * driving, so the check samples on its own slow clock instead - it only
     * needs to spot an encoder that has been frozen for {@link #ENCODER_DEAD_MS},
     * which this is still far quicker than.
     */
    public static final double ENCODER_SAMPLE_MS = 50;

    /** One drive wheel plus the health of its encoder. */
    private final class Wheel {
        final String name;
        DcMotorEx motor;
        boolean encoderDead = false;

        private final ElapsedTime frozenFor = new ElapsedTime();
        private final ElapsedTime sinceSample = new ElapsedTime();
        private int lastPosition = 0;
        private boolean havePosition = false;

        Wheel(String name, DcMotorEx.Direction direction) {
            this.name = name;
            motor = Protect.map(protect, telemetry, name, () -> {
                DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
                m.setDirection(direction);
                m.setMode(useVelocity
                        ? DcMotor.RunMode.RUN_USING_ENCODER
                        : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                return m;
            });
        }

        boolean present() {
            return motor != null;
        }

        /** True while this wheel is still running its encoder speed loop. */
        boolean usingEncoder() {
            return useVelocity && !encoderDead;
        }

        int position() {
            if (motor == null) {
                return 0;
            }
            return (int) Protect.getDouble(protect, telemetry, "Encoder." + name,
                    () -> motor.getCurrentPosition(), 0);
        }

        /**
         * Watches for an encoder that has stopped counting while the wheel is
         * being driven, and drops that wheel to open loop if it has.
         *
         * @param command how hard this wheel is being driven, 0..1
         */
        void checkEncoder(double command) {
            if (motor == null || encoderDead || !useVelocity) {
                return;
            }

            // Only touch the hardware on the slow clock - see ENCODER_SAMPLE_MS.
            if (havePosition && sinceSample.milliseconds() < ENCODER_SAMPLE_MS) {
                return;
            }
            sinceSample.reset();

            int position = position();
            if (!havePosition) {
                havePosition = true;
                lastPosition = position;
                frozenFor.reset();
                return;
            }

            if (Math.abs(command) < ENCODER_CHECK_MIN_COMMAND) {
                // Not asking it to move, so a still encoder tells us nothing.
                lastPosition = position;
                frozenFor.reset();
                return;
            }

            if (Math.abs(position - lastPosition) >= ENCODER_ALIVE_TICKS) {
                lastPosition = position;
                frozenFor.reset();
                return;
            }

            if (frozenFor.milliseconds() > ENCODER_DEAD_MS) {
                dropToOpenLoop();
            }
        }

        /** Stop trusting this encoder and drive the wheel on raw power instead. */
        private void dropToOpenLoop() {
            encoderDead = true;
            Protect.run(protect, telemetry, name + ".openLoop",
                    () -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
            if (telemetry != null) {
                telemetry.log().add(name + ": encoder not counting - driving it on power instead");
            }
        }

        /**
         * @param velocity closed-loop target, ticks/sec
         * @param power    open-loop fallback, -1..1
         */
        void drive(double velocity, double power) {
            if (motor == null) {
                return;
            }
            if (usingEncoder()) {
                Protect.run(protect, telemetry, name, () -> motor.setVelocity(velocity));
            } else {
                Protect.run(protect, telemetry, name, () -> motor.setPower(power));
            }
        }
    }

    private final Wheel rightFront;
    private final Wheel leftFront;
    private final Wheel leftBack;
    private final Wheel rightBack;

    /** All four wheels, kept as a field so the per-loop checks allocate nothing. */
    private final Wheel[] wheels;

    // Defines the different drive speed gears.

    public enum DriveGear {
        LOW(0.35, 0.4, "Low"),
        MEDIUM(0.6, 0.5, "Medium"),
        HIGH(1.0, 0.8, "High");

        public final double maxSpeed;
        public final double turnSpeed; // This 'turnSpeed' from enum seems to be a direct speed cap for turning
        public final String telemetryName;

        DriveGear(double maxSpeed, double turnSpeed, String telemetryName) {
            this.maxSpeed = maxSpeed;
            this.turnSpeed = turnSpeed;
            this.telemetryName = telemetryName;
        }
    }

    public MoveRobot(boolean protect, HardwareMap hardwareMap, Telemetry telemetry, boolean useVelocity) {

        //Pass required objects and a setting to the class
        this.protect = protect;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.useVelocity = useVelocity;

        // Each wheel is mapped on its own. Losing one motor to a bad cable used to
        // throw out of here and take the whole drivetrain with it; now the other
        // three still drive and only the dead corner is skipped.
        rightFront = new Wheel(HardwareConstants.RIGHT_FRONT_MOTOR, DcMotorEx.Direction.FORWARD);
        leftFront = new Wheel(HardwareConstants.LEFT_FRONT_MOTOR, DcMotorEx.Direction.REVERSE);
        leftBack = new Wheel(HardwareConstants.LEFT_BACK_MOTOR, DcMotorEx.Direction.FORWARD);
        rightBack = new Wheel(HardwareConstants.RIGHT_BACK_MOTOR, DcMotorEx.Direction.REVERSE);
        wheels = new Wheel[]{rightFront, leftFront, leftBack, rightBack};

        // One bulk read per loop instead of a separate round trip per encoder.
        // Without this the health check alone adds four hub reads to every loop.
        enableBulkReads();
    }

    /**
     * Puts every hub in AUTO bulk-caching mode, so all the encoder values for a
     * loop arrive in a single transfer. This is what keeps the encoder health
     * check from costing anything measurable.
     */
    private void enableBulkReads() {
        Protect.run(protect, telemetry, "bulk reads", () -> {
            for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        });
    }

    /** True if at least one drive motor mapped - i.e. the robot can still move. */
    public boolean isAvailable() {
        return rightFront.present() || leftFront.present()
                || leftBack.present() || rightBack.present();
    }

    /** How many of the four drive motors are actually present. */
    public int getWorkingMotorCount() {
        int n = 0;
        if (rightFront.present()) n++;
        if (leftFront.present()) n++;
        if (leftBack.present()) n++;
        if (rightBack.present()) n++;
        return n;
    }

    /** Names of wheels whose encoder stopped counting, or "-" if all are fine. */
    public String getDeadEncoders() {
        StringBuilder sb = new StringBuilder();
        for (Wheel w : wheels) {
            if (w.present() && w.encoderDead) {
                if (sb.length() > 0) {
                    sb.append(", ");
                }
                sb.append(w.name);
            }
        }
        return sb.length() == 0 ? "-" : sb.toString();
    }

    /** True while every present wheel is still running closed loop on its encoder. */
    public boolean allEncodersOk() {
        for (Wheel w : wheels) {
            if (w.present() && w.encoderDead) {
                return false;
            }
        }
        return true;
    }

    /**
     * Controls the robot's movement based on joystick inputs and current heading.
     * Can operate in field-centric or robot-centric mode.
     *
     * @param heading      Current robot heading in radians (e.g., from IMU).
     * @param drive        Forward/backward input (-1.0 to 1.0).
     * @param strafe       Left/right strafe input (-1.0 to 1.0) - Does tank strafe though?
     * @param turn         Turning input (-1.0 to 1.0).
     * @param fieldCentric True if movement should be relative to the field, false for robot-centric.
     * @param driveGear    The selected drive gear (LOW, MEDIUM, HIGH) determining max speed.
     */
    public void move(double heading, double drive, double strafe, double turn,
                     boolean fieldCentric,
                     DriveGear driveGear
    ) {
        this.maxSpeed = driveGear.maxSpeed;
        if (driveGear == DriveGear.HIGH) {
            drive = Math.signum(drive) * Math.pow(Math.abs(drive), 1.8);
            strafe = Math.signum(strafe) * Math.pow(Math.abs(strafe), 1.8);
            turn = Math.signum(turn) * Math.pow(Math.abs(turn), 1.8);
        }
        double x;
        double y;
        double turnCompensation;

        //the robot can constantly compensate for its angle or have it be freely turning
        wantedAngle = heading; // so if switched to the other the robot wont flick to a distant angle
        turnCompensation = turn;

        // The operator can choose to move the robot relative to the field or to the robot
        x = drive;
        y = strafe;

        if (fieldCentric && protect) {
            try {
                x = drive * Math.cos(heading) - strafe * Math.sin(heading);
                y = drive * Math.sin(heading) + strafe * Math.cos(heading);
            } catch (Exception ignored) {
                // Already defaulted to drive/strafe
            }
        } else if (fieldCentric) {
            x = drive * Math.cos(heading) - strafe * Math.sin(heading);
            y = drive * Math.sin(heading) + strafe * Math.cos(heading);
        }

        // Calculates raw power to motors
        double leftFrontPowerRaw = x + y + turnCompensation;
        double leftBackPowerRaw = x - y + turnCompensation;
        double rightFrontPowerRaw = x - y - turnCompensation;
        double rightBackPowerRaw = x + y - turnCompensation;

        // Calculate the maximum absolute power value for normalization
        double maxRawPower = Math.max(
                Math.max(Math.abs(leftFrontPowerRaw), Math.abs(leftBackPowerRaw)),
                Math.max(Math.abs(rightFrontPowerRaw), Math.abs(rightBackPowerRaw))
        );
        // if the power is not over 1, the code will divide by 1, which doesn't affect the end result
        double max = Math.max(maxRawPower, 1);

        // Normalized -1..1 per wheel. This doubles as the open-loop power and as
        // "how hard are we asking this wheel to turn" for the encoder check.
        double leftFrontCommand = leftFrontPowerRaw / max * maxSpeed;
        double leftBackCommand = leftBackPowerRaw / max * maxSpeed;
        double rightFrontCommand = rightFrontPowerRaw / max * maxSpeed;
        double rightBackCommand = rightBackPowerRaw / max * maxSpeed;

        // Catch a dead encoder before handing it another velocity target.
        leftFront.checkEncoder(leftFrontCommand);
        leftBack.checkEncoder(leftBackCommand);
        rightFront.checkEncoder(rightFrontCommand);
        rightBack.checkEncoder(rightBackCommand);

        leftFront.drive(leftFrontPowerRaw / max * MAX_ANGULAR_VELOCITY_RADIANS * maxSpeed,
                leftFrontCommand);
        leftBack.drive(leftBackPowerRaw / max * MAX_ANGULAR_VELOCITY_RADIANS * maxSpeed,
                leftBackCommand);
        rightFront.drive(rightFrontPowerRaw / max * MAX_ANGULAR_VELOCITY_RADIANS * maxSpeed,
                rightFrontCommand);
        rightBack.drive(rightBackPowerRaw / max * MAX_ANGULAR_VELOCITY_RADIANS * maxSpeed,
                rightBackCommand);
    }


    public int[] getEncoderPositions() {
        return new int[]{
                rightFront.position(),
                leftFront.position(),
                leftBack.position(),
                rightBack.position()
        };
    }

}
