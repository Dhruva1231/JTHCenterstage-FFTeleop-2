package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {

    private DcMotorEx leftRear, rightRear, rightFront;
    private Encoder leftEncoder, rightEncoder, frontEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        leftRear = hardwareMap.get(DcMotorEx.class, "2");
        rightRear = hardwareMap.get(DcMotorEx.class, "11");
        rightFront = hardwareMap.get(DcMotorEx.class, "00");

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "3"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "2"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "0"));

        leftEncoder.setDirection(Encoder.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("left", leftEncoder.getCurrentPosition());
            telemetry.addData("right", rightEncoder.getCurrentPosition());
            telemetry.addData("front", frontEncoder.getCurrentPosition());
            telemetry.addData("left1", leftRear.getCurrentPosition());
            telemetry.addData("right1", rightFront.getCurrentPosition());
            telemetry.addData("front1", rightRear.getCurrentPosition());
            telemetry.update();
        }
    }
}
