package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.util.HardwareConstants;
import org.firstinspires.ftc.teamcode.common.util.Protect;

/**
 * Module for launching balls toward the goal.
 */
public class ThrowBalls {

    private DcMotorEx wheelMotor;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final boolean protect;
    public static final double REQUIRED_SPEED_RPM = 2700;
    public static final double WHEEL_VELOCITY_RPM = 2785;

    public static final double REQUIRED_SPEED = REQUIRED_SPEED_RPM/60*28;
    public static final double WHEEL_VELOCITY = WHEEL_VELOCITY_RPM/60*28;

    /**
     * Initializes the launcher module.
     */
    public ThrowBalls(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        mapMotor();
    }

    private void mapMotor() {
        wheelMotor = Protect.map(protect, telemetry, HardwareConstants.THROW_MOTOR, () -> {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, HardwareConstants.THROW_MOTOR);
            m.setDirection(DcMotorEx.Direction.REVERSE);
            m.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            return m;
        });
    }

    /** True if the launcher motor is actually in the configuration. */
    public boolean isAvailable() {
        return wheelMotor != null;
    }

    /**
     * Spins the launcher wheel.
     *
     * @param Spinning True to spin at full power, false to stop.
     */
    public void spin(boolean Spinning) {
        if (wheelMotor == null) {
            return;
        }
        final double velocity = Spinning ? WHEEL_VELOCITY : 0;
        Protect.run(protect, telemetry, "ThrowBalls.spin",
                () -> wheelMotor.setVelocity(velocity));
    }

    public double throwSpeed() {
        if (wheelMotor == null) {
            return 0;
        }
        // 0 on failure: reads as "not up to speed", so the feeder holds off.
        return Protect.getDouble(protect, telemetry, "ThrowBalls.throwSpeed",
                () -> wheelMotor.getVelocity()/28*60, 0);
    }

    public boolean isAtSpeed() {
        if (wheelMotor == null) {
            return false;
        }
        // false on failure, so we never feed a ball into a wheel we cannot read.
        return Protect.getBoolean(protect, telemetry, "ThrowBalls.isAtSpeed",
                () -> wheelMotor.getVelocity() > REQUIRED_SPEED, false);
    }

    public void stop() {
        if (wheelMotor == null) {
            return;
        }
        Protect.run(protect, telemetry, "ThrowBalls.stop",
                () -> wheelMotor.setVelocity(0));
    }

    /* ======================
       Shot measurement
       ----------------------
       Every ball pulls the flywheel RPM down and the motor pulls it back. This
       only MEASURES that dip - it reports what the encoder saw and nothing more:
       the speed the wheel was holding, the lowest speed it reached, and the
       difference between them.

       Nothing here assumes how deep a dip "should" be. Run the wheel, shoot, and
       read the numbers off the driver station. Once there is real data on what
       one ball costs, that number is what tells a single feed from a double -
       until then the drops are reported raw and nothing is inferred from them.
       ====================== */

    /**
     * RPM that ONE ball actually costs, measured on this robot.
     *
     * <p>Leave at 0 until it has been measured: at 0 nothing is inferred and the
     * telemetry reports raw drops only. To calibrate, spin the wheel up, feed
     * single balls, and read the "Recent drops" line - the typical value goes
     * here. From then on the drop is divided by it, so a shot that digs twice as
     * deep is reported as 2 balls and a double feed gets counted properly.
     *
     * <p>ball_calc.xlsx predicts about 69 RPM at 2785, but that is theory. Put
     * the measured number here, not that one.
     */
    public static double dropPerBallRpm = 0;

    /** Sanity cap - more balls than this in one dip is a misread, not a feed. */
    public static final int MAX_BALLS_PER_SHOT = 3;

    /** How far below the steady speed the wheel must fall to count as a shot, RPM. */
    public static final double DIP_ENTER_RPM = 25;

    /** How close to the steady speed it must climb back for the shot to be over, RPM. */
    public static final double DIP_EXIT_RPM = 12;

    /** A dip shorter than this is encoder noise, not a ball. Milliseconds. */
    public static final double MIN_DIP_MS = 60;

    /** A dip may not last longer than this - past it, something else is wrong. */
    public static final double MAX_DIP_MS = 800;

    /** How many recent drops to keep, so the real spread can be read off. */
    public static final int RECENT_DROPS = 8;

    /** Samples in the moving average that feeds the detector. */
    private static final int FILTER_SAMPLES = 3;

    /**
     * Recent raw readings kept so the true bottom of a dip is not lost.
     * The moving average needs a couple of samples to fall far enough to trigger,
     * and a shallow dip is already climbing back by then - so when a dip starts
     * we look back at the readings just before it and take the lowest one that
     * was already below the steady speed. These are real encoder samples, just
     * ones that arrived before the detector was sure a shot was happening.
     */
    private static final int HISTORY_SAMPLES = 6;

    /** Time constant of the steady-speed tracker, seconds. Slow, so a dip cannot drag it down. */
    private static final double BASELINE_TAU_SEC = 1.0;

    private final ElapsedTime sampleTimer = new ElapsedTime();
    private final ElapsedTime dipTimer = new ElapsedTime();

    private final double[] filterBuffer = new double[FILTER_SAMPLES];
    private int filterFill = 0;
    private int filterIndex = 0;

    private final double[] historyRpm = new double[HISTORY_SAMPLES];
    private int historyFill = 0;
    private int historyIndex = 0;

    private boolean trackingArmed = false;  // wheel reached speed, measuring for real
    private boolean inDip = false;          // currently inside a shot

    private double baselineRpm = 0;         // steady speed the dip is measured against
    private double filteredRpm = 0;         // moving average of the raw readings
    private double dipLowestRpm = 0;        // lowest raw reading inside this dip

    private int shotCount = 0;
    private int ballCount = 0;              // only counted once dropPerBallRpm is set
    private int lastShotBalls = 0;
    private double lastDropRpm = 0;
    private double lastLowestRpm = 0;
    private double lastBaselineRpm = 0;
    private double lastDipMs = 0;
    private double minDropRpm = 0;
    private double maxDropRpm = 0;
    private double totalDropRpm = 0;

    private final double[] recentDrops = new double[RECENT_DROPS];
    private int recentCount = 0;
    private int recentIndex = 0;

    /**
     * Feed the current flywheel RPM in once per loop while the wheel is spinning.
     * Measures the dip each ball makes.
     *
     * @param currentRpm the flywheel speed this loop, from {@link #throwSpeed()}
     */
    public void updateShotTracking(double currentRpm) {
        double dt = sampleTimer.seconds();
        sampleTimer.reset();
        if (dt <= 0 || dt > 0.5) {
            // First call, or the loop stalled - no usable time slice.
            return;
        }

        // --- keep the recent raw readings so a dip's bottom is not missed ---
        historyRpm[historyIndex] = currentRpm;
        historyIndex = (historyIndex + 1) % HISTORY_SAMPLES;
        if (historyFill < HISTORY_SAMPLES) {
            historyFill++;
        }

        // --- moving average, so one bad reading cannot start or end a dip ---
        filterBuffer[filterIndex] = currentRpm;
        filterIndex = (filterIndex + 1) % FILTER_SAMPLES;
        if (filterFill < FILTER_SAMPLES) {
            filterFill++;
        }
        double sum = 0;
        for (int i = 0; i < filterFill; i++) {
            sum += filterBuffer[i];
        }
        filteredRpm = sum / filterFill;

        // Wait until the wheel has actually spun up before measuring anything.
        if (!trackingArmed) {
            if (filteredRpm >= REQUIRED_SPEED_RPM) {
                trackingArmed = true;
                baselineRpm = filteredRpm;
            }
            return;
        }

        if (!inDip) {
            // Track the steady speed slowly. A dip is far too brief to move it,
            // so a draining battery follows along but a ball does not.
            double alpha = Math.min(1.0, dt / BASELINE_TAU_SEC);
            baselineRpm += (filteredRpm - baselineRpm) * alpha;

            if (filteredRpm < baselineRpm - DIP_ENTER_RPM) {
                inDip = true;
                dipLowestRpm = lowestRecentBelowBaseline();
                dipTimer.reset();
            }
        } else {
            // Baseline is frozen here on purpose - it is the reference we measure against.
            if (currentRpm < dipLowestRpm) {
                dipLowestRpm = currentRpm;
            }

            boolean recovered = filteredRpm > baselineRpm - DIP_EXIT_RPM;
            boolean tooLong = dipTimer.milliseconds() > MAX_DIP_MS;

            if (recovered || tooLong) {
                closeDip(tooLong);
            }
        }
    }

    /**
     * Lowest of the recent raw readings that were already below the steady speed.
     * This is where the bottom of a short dip actually is - by the time the
     * moving average has dropped far enough to trigger, the wheel is climbing again.
     */
    private double lowestRecentBelowBaseline() {
        double lowest = Double.MAX_VALUE;
        for (int back = 0; back < historyFill; back++) {
            int i = (historyIndex - 1 - back + HISTORY_SAMPLES * 2) % HISTORY_SAMPLES;
            double rpm = historyRpm[i];
            if (rpm >= baselineRpm) {
                break;   // reached back to where the wheel was still at speed
            }
            if (rpm < lowest) {
                lowest = rpm;
            }
        }
        return lowest == Double.MAX_VALUE ? baselineRpm : lowest;
    }

    /** A dip ended - record what the encoder saw, unless it was too brief to trust. */
    private void closeDip(boolean tooLong) {
        double durationMs = dipTimer.milliseconds();
        double drop = baselineRpm - dipLowestRpm;

        if (!tooLong && durationMs >= MIN_DIP_MS && drop > 0) {
            shotCount++;

            // Only turn the drop into a ball count once someone has measured what
            // a ball is worth. Until then this stays 0 rather than guessing.
            if (isCalibrated()) {
                int balls = (int) Math.round(drop / dropPerBallRpm);
                balls = Math.max(1, Math.min(MAX_BALLS_PER_SHOT, balls));
                lastShotBalls = balls;
                ballCount += balls;
            }

            lastBaselineRpm = baselineRpm;
            lastLowestRpm = dipLowestRpm;
            lastDropRpm = drop;
            lastDipMs = durationMs;
            totalDropRpm += drop;
            if (drop > maxDropRpm) {
                maxDropRpm = drop;
            }
            if (minDropRpm == 0 || drop < minDropRpm) {
                minDropRpm = drop;
            }
            recentDrops[recentIndex] = drop;
            recentIndex = (recentIndex + 1) % RECENT_DROPS;
            if (recentCount < RECENT_DROPS) {
                recentCount++;
            }
        }

        inDip = false;
    }

    /** Clears all measurements. Call when the flywheel is switched off. */
    public void resetShotTracking() {
        trackingArmed = false;
        inDip = false;
        filterFill = 0;
        filterIndex = 0;
        historyFill = 0;
        historyIndex = 0;
        baselineRpm = 0;
        filteredRpm = 0;
        dipLowestRpm = 0;
        shotCount = 0;
        ballCount = 0;
        lastShotBalls = 0;
        lastDropRpm = 0;
        lastLowestRpm = 0;
        lastBaselineRpm = 0;
        lastDipMs = 0;
        minDropRpm = 0;
        maxDropRpm = 0;
        totalDropRpm = 0;
        recentCount = 0;
        recentIndex = 0;
    }

    /** True while a shot is being measured right now. */
    public boolean isInShot() {
        return inDip;
    }

    /** True once dropPerBallRpm has been set, i.e. ball counts mean something. */
    public static boolean isCalibrated() {
        return dropPerBallRpm > 0;
    }

    /** Balls counted so far. Always 0 until dropPerBallRpm has been measured. */
    public int getBallCount() {
        return ballCount;
    }

    /** Balls in the most recent shot - 2 means a double feed. 0 if not calibrated. */
    public int getLastShotBalls() {
        return lastShotBalls;
    }

    /** Dips measured since the flywheel was switched on. */
    public int getShotCount() {
        return shotCount;
    }

    /** RPM the wheel lost on the most recent shot - measured, not estimated. */
    public double getLastDropRpm() {
        return lastDropRpm;
    }

    /** Speed the wheel was holding before the most recent shot. */
    public double getLastBaselineRpm() {
        return lastBaselineRpm;
    }

    /** Lowest speed the encoder reported during the most recent shot. */
    public double getLastLowestRpm() {
        return lastLowestRpm;
    }

    /** How long the most recent dip lasted, milliseconds. */
    public double getLastDipMs() {
        return lastDipMs;
    }

    /** Smallest drop measured so far. */
    public double getMinDropRpm() {
        return minDropRpm;
    }

    /** Biggest drop measured so far. */
    public double getMaxDropRpm() {
        return maxDropRpm;
    }

    /** Average drop across all shots so far, 0 if none. */
    public double getAverageDropRpm() {
        return shotCount == 0 ? 0 : totalDropRpm / shotCount;
    }

    /** The steady speed dips are currently being measured against. */
    public double getBaselineRpm() {
        return baselineRpm;
    }

    /**
     * The last few drops, newest first, so the real spread can be read straight
     * off the driver station instead of guessed at.
     */
    public String getRecentDropsText() {
        if (recentCount == 0) {
            return "-";
        }
        StringBuilder sb = new StringBuilder();
        for (int back = 1; back <= recentCount; back++) {
            int i = (recentIndex - back + RECENT_DROPS * 2) % RECENT_DROPS;
            if (sb.length() > 0) {
                sb.append(" ");
            }
            sb.append(Math.round(recentDrops[i]));
        }
        return sb.toString();
    }
}
