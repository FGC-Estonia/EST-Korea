package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ImuManager {

    //These are used in multiple places so they need to be defined here
    private IMU imu;

    private boolean imuErrorBoolean = false;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private final boolean protect;

    public void initImu(){
        if (protect) {
            try {
                // Initializing imu to avoid errors
                imu = hardwareMap.get(IMU.class, "imu");

                RevHubOrientationOnRobot.LogoFacingDirection logoDirection;
                RevHubOrientationOnRobot.UsbFacingDirection usbDirection;

                    logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
                    usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

                RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

                imu.initialize(new IMU.Parameters(orientationOnRobot));

                imu.resetYaw();
                imuErrorBoolean = false;
            } catch (Exception errorInitIMU) {
                imuErrorBoolean = true;
                telemetry.addData("IMU error", errorInitIMU.getMessage());
            }
        }else {
            imu = hardwareMap.get(IMU.class, "imu");

            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            imu.initialize(new IMU.Parameters(orientationOnRobot));

            imu.resetYaw();
            imuErrorBoolean = false;
        }
    }

    public ImuManager(boolean protect, HardwareMap hardwareMap, Telemetry telemetry) {
        this.protect = protect;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        initImu();
    }

    public void resetImu(){
        if (protect) {
            try {
                imu.resetYaw();
            } catch (Exception resetException) {
                telemetry.addLine(resetException.getMessage());
            }
        }
        else {
            imu.resetYaw();
        }
    }

    public org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles getAngles() {
        if (imuErrorBoolean) {
            // Don't re-init every loop if there is an error, it's too slow.
            // Just return zeros.
            return new org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0);
        }

        try {
            return imu.getRobotYawPitchRollAngles();
        } catch (Exception e) {
            imuErrorBoolean = true;
            telemetry.addData("IMU ERROR", e.getMessage());
            return new org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0);
        }
    }

    public double getYawRadians(){
        return getAngles().getYaw(AngleUnit.RADIANS);
    }

    private double smoothedYaw = 0;
    private final double alpha = 0.1; // Smoothing factor (0.0 = very smooth, 1.0 = no smoothing)

    public double getSmoothedYawRadians() {
        double rawYaw = getYawRadians(); // Assume this reads from IMU
        smoothedYaw = alpha * rawYaw + (1 - alpha) * smoothedYaw;
        return smoothedYaw;
    }

    public double getPitchRadians(){
        return getAngles().getPitch(AngleUnit.RADIANS);
    }
}
