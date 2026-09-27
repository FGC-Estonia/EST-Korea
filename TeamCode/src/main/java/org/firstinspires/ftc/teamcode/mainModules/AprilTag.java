package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

public class AprilTag {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    public double TARGET_DISTANCE = 6.0;  // Until X inches from tag
    public int TARGET_TAG_ID = 103;

    public AprilTag(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        initAprilTag();
    }

    private void initAprilTag() {
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
    }

    public List<AprilTagDetection> getDetections() {
        return aprilTagProcessor.getDetections();
    }

    private AprilTagDetection getDetectionById(int id) {
        for (AprilTagDetection detection : getDetections()) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public double[] getCommand() {
        AprilTagDetection target = getDetectionById(TARGET_TAG_ID);
        if (target == null || target.metadata == null) {
            return null;
        }

        double rangeError = target.ftcPose.range - TARGET_DISTANCE;
        double turn = 0.02 * target.ftcPose.bearing;
        double drive = 0.02 * rangeError;

        drive = Math.max(-1.0, Math.min(1.0, drive));
        turn = Math.max(-1.0, Math.min(1.0, turn));

        return new double[]{drive, turn};
    }

    public void sendTelemetry() {
        List<AprilTagDetection> detections = getDetections();
        telemetry.addData("# AprilTags Detected", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addData("Tag ID", detection.id);
                telemetry.addData("RBE", "%.1f %.1f %.1f (in, deg, deg)",
                        detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation);
            }
        }
    }

    public void close() {
        visionPortal.close();
    }
}