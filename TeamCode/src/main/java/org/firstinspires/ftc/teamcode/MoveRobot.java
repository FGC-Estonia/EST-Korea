package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "X-Drive TeleOp")
public class MoveRobot extends LinearOpMode {

    // ---- Simple speed presets, replaces an external DriveGear enum ----
    public enum DriveGear {
        LOW(0.4),
        MEDIUM(0.7),
        HIGH(1.0);

        public final double maxSpeed;

        DriveGear(double maxSpeed) {
            this.maxSpeed = maxSpeed;
        }
    }

    // ---- Drive base, nested so the whole OpMode stays in one file ----
    public static class DriveBase {

        // Hardware-map names follow the motor's wired port on the Control Hub
        private static final String LEFT_FRONT  = "Motor_Port_1_CH";
        private static final String RIGHT_FRONT = "Motor_Port_0_CH";
        private static final String LEFT_BACK    = "Motor_Port_2_CH";
        private static final String RIGHT_BACK   = "Motor_Port_3_CH";

        private final DcMotor leftFront;
        private final DcMotor rightFront;
        private final DcMotor leftBack;
        private final DcMotor rightBack;

        public DriveBase(HardwareMap hardwareMap) {
            leftFront  = hardwareMap.get(DcMotor.class, LEFT_FRONT);
            rightFront = hardwareMap.get(DcMotor.class, RIGHT_FRONT);
            leftBack   = hardwareMap.get(DcMotor.class, LEFT_BACK);
            rightBack  = hardwareMap.get(DcMotor.class, RIGHT_BACK);

            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);

            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        /**
         * X-drive (mecanum) movement.
         *
         * @param imuAngle         Robot heading in radians, used only if fieldCentric is true
         * @param imuPitch         Unused, kept for parity with the call signature
         * @param strafe           Left/right input, -1.0 to 1.0
         * @param drive            Forward/back input, -1.0 to 1.0
         * @param turn             Rotation input, -1.0 to 1.0
         * @param fieldCentric     True = field-relative driving, false = robot-relative
         * @param currentDriveGear Speed cap for this loop
         */
        public void drive(double imuAngle, double imuPitch,
                          double strafe, double drive, double turn,
                          boolean fieldCentric, DriveGear currentDriveGear) {

            double x = drive;
            double y = strafe;

            if (fieldCentric) {
                x = drive * Math.cos(imuAngle) - strafe * Math.sin(imuAngle);
                y = drive * Math.sin(imuAngle) + strafe * Math.cos(imuAngle);
            }

            double leftFrontPower  = x + y + turn;
            double leftBackPower   = x - y + turn;
            double rightFrontPower = x - y - turn;
            double rightBackPower  = x + y - turn;

            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                    Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))
            ));

            double speed = currentDriveGear.maxSpeed;

            leftFront.setPower(leftFrontPower / max * speed);
            leftBack.setPower(leftBackPower / max * speed);
            rightFront.setPower(rightFrontPower / max * speed);
            rightBack.setPower(rightBackPower / max * speed);
        }
    }

    @Override
    public void runOpMode() {

        DriveBase driveBase = new DriveBase(hardwareMap);

        // Only needed for field-centric mode; remove these two lines if you don't use it
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        telemetry.addLine("Ready - press start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double turn   =  gamepad1.right_stick_x;

            boolean fieldCentric = gamepad1.right_bumper;

            DriveGear currentDriveGear = gamepad1.left_bumper ? DriveGear.LOW : DriveGear.HIGH;

            double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double imuPitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);

            driveBase.drive(
                    imuAngle, imuPitch,
                    strafe, drive, turn,
                    fieldCentric, currentDriveGear
            );

            telemetry.addData("Gear", currentDriveGear);
            telemetry.addData("Field Centric", fieldCentric);
            telemetry.addData("Heading (rad)", imuAngle);
            telemetry.update();
        }
    }
}