package org.firstinspires.ftc.teamcode.common.util;

import java.util.HashSet;
import java.util.Set;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Helpers for the "protect" flag every module takes in its constructor.
 *
 * <p>protect = true means a hardware problem must not take the whole op-mode down
 * with it: the failure is logged once and the robot carries on with everything
 * that still works. protect = false lets the exception through so it is loud and
 * obvious while testing in the pit.
 *
 * <p>This covers both ends of a device's life:
 * <ul>
 *   <li>{@link #map} for looking a device up at init. A device that is not in the
 *       configuration comes back null instead of throwing, so ONE missing servo
 *       disables that servo and nothing else - the rest of its module, and every
 *       other module, still runs. Modules must null-check what map returned.
 *   <li>{@link #run}, {@link #getDouble}, {@link #getBoolean} for calls in the
 *       main loop, so a device that dies mid-match is skipped rather than fatal.
 * </ul>
 *
 * <p>Failures are logged once per device, not once per loop, so a broken motor
 * cannot flood the driver station. Call {@link #resetReports()} when an op-mode
 * starts so a new run reports afresh.
 */
public final class Protect {

    private Protect() {}

    /** Devices already complained about, so the log is not spammed every loop. */
    private static final Set<String> reported = new HashSet<>();

    /** Forget what has already been reported. Call this when an op-mode starts. */
    public static void resetReports() {
        reported.clear();
    }

    /** Looks a device up in the hardware map. */
    public interface Mapper<T> {
        T get();
    }

    /** A hardware call that returns nothing. */
    public interface Action {
        void run();
    }

    /** A hardware call that returns a number. */
    public interface DoubleAction {
        double get();
    }

    /** A hardware call that returns a flag. */
    public interface BooleanAction {
        boolean get();
    }

    /**
     * Looks up a device at init time.
     *
     * <p>When protected, a device missing from the configuration is logged and
     * comes back null rather than throwing - so the caller loses that one device
     * and keeps everything else. Callers MUST null-check the result before using
     * it. When not protected the lookup throws as normal.
     *
     * @param what the configuration name, used in the log
     * @return the device, or null when it could not be found and protect is on
     */
    public static <T> T map(boolean protect, Telemetry telemetry, String what, Mapper<T> mapper) {
        if (!protect) {
            return mapper.get();
        }
        try {
            return mapper.get();
        } catch (RuntimeException e) {
            reportOnce(telemetry, "hardware not found: " + what);
            return null;
        }
    }

    /**
     * Runs a hardware call, swallowing and logging failures when protected.
     *
     * @param protect   the module's protect flag
     * @param telemetry where to log a failure, may be null
     * @param what      short name of the action, shown in the log
     * @param action    the call to make
     */
    public static void run(boolean protect, Telemetry telemetry, String what, Action action) {
        if (!protect) {
            action.run();
            return;
        }
        try {
            action.run();
        } catch (RuntimeException e) {
            report(telemetry, what, e);
        }
    }

    /**
     * Reads a number from the hardware, falling back to a safe value when protected.
     *
     * @param fallback what to return if the read fails - pick a value that makes
     *                 the caller behave safely, not just zero
     */
    public static double getDouble(boolean protect, Telemetry telemetry, String what,
                                   DoubleAction action, double fallback) {
        if (!protect) {
            return action.get();
        }
        try {
            return action.get();
        } catch (RuntimeException e) {
            report(telemetry, what, e);
            return fallback;
        }
    }

    /**
     * Reads a flag from the hardware, falling back to a safe value when protected.
     */
    public static boolean getBoolean(boolean protect, Telemetry telemetry, String what,
                                     BooleanAction action, boolean fallback) {
        if (!protect) {
            return action.get();
        }
        try {
            return action.get();
        } catch (RuntimeException e) {
            report(telemetry, what, e);
            return fallback;
        }
    }

    private static void report(Telemetry telemetry, String what, RuntimeException e) {
        reportOnce(telemetry, what + " failed: " + e.getMessage());
    }

    /** Logs a message the first time it is seen and stays quiet after that. */
    private static void reportOnce(Telemetry telemetry, String message) {
        if (telemetry != null && reported.add(message)) {
            telemetry.log().add(message);
        }
    }
}
