package org.firstinspires.ftc.teamcode.mainModules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;
import static org.firstinspires.ftc.teamcode.mainModules.MoveRobot.DriveGear;
@TeleOp(name = "AprilTag", group = "Concept")
public class AprilTag extends LinearOpMode {
    private MoveRobot driveBase;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final double TARGET_DISTANCE = 6.0; // How far to stop (in inches)
    // private static final int TARGET_ID = 103; // future thing idk yet


    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initAprilTag();

        initDriveBase();


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            telemetryAprilTag();
            if (gamepad1.right_bumper) {
                driveToAprilTag();
            } else {
                driveManually();
            }

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            // if (gamepad1.dpad_down) {
            //     visionPortal.stopStreaming();
            // } else if (gamepad1.dpad_up) {
            //     visionPortal.resumeStreaming();
            // }

            // Share the CPU.
            sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

    private void initDriveBase() {
        driveBase = new MoveRobot(true, hardwareMap, telemetry, true);
    }

    //driveBase.move(imuAngle, drive, strafe, turn, fieldCentric, currentDriveGear);


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format(Locale.UK, "\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format(Locale.UK, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format(Locale.UK, "\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format(Locale.UK, "Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
    }   // end method telemetryAprilTag()
    private void driveManually() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        driveBase.move(0, drive, strafe, turn, false, DriveGear.HIGH);
    }
    private void driveToAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        double drive = 0;
        double turn = 0;

        if (!detections.isEmpty()) {
            AprilTagDetection detection = detections.get(0); // just use the first tag seen

            if (detection.metadata != null) {
                double rangeError = detection.ftcPose.range - TARGET_DISTANCE;
                turn = 0.02 * detection.ftcPose.bearing;
                drive = 0.02 * rangeError;
                drive = Math.max(-1.0, Math.min(1.0, drive));
                turn = Math.max(-1.0, Math.min(1.0, turn));
            }
        }
        driveBase.move(0, drive, 0, turn, false, DriveGear.HIGH);
    }
}   // end class