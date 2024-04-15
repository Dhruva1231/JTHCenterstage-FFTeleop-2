package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE1_DEPOSIT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE1_INTAKE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE1_INTER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE1_PRE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE1_PRE2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE1_TRANSFER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE1_TRANSFER1_5;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE1_TRANSFER3;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE1_TRANSFER4;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE1_TRANSFERINTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE2_DEPOSIT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE2_INTAKE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE2_INTER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE2_INTER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE2_INTER_5;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE2_TRANSFER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE2_TRANSFER1_5;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE2_TRANSFER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE2_TRANSFER3;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE2_TRANSFERINTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE3_DEPOSIT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE3_INTAKE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE3_INTER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE3_INTER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE3_TRANSFER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE3_TRANSFER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE3_TRANSFER3;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE3_TRANSFERINTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE3_TRANSFERINTER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.PRELOAD_YELLOW;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.PRELOAD_YELLOW_DROP;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.PRELOAD_YELLOW_INTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.SPIT10;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.SPIT11;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.SPIT12;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.SPIT13;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.SPIT14;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.SPIT15;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.SPIT16;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.SPIT2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.SPIT3;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.SPIT4;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.SPIT5;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.SPIT6;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.SPIT7;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.SPIT8;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.SPIT9;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.flip.betweenflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.flip.initializeflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.flip.intake2flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.flip.intake3flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.flip.intake4flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.flip.intake5flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.flip.transferflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.RedTrussAuto.State.CYCLE1_TRANSFER2;

import android.bluetooth.le.ScanSettings;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.Barcode;
import org.firstinspires.ftc.teamcode.vision.BlueScanner;
import org.firstinspires.ftc.teamcode.vision.RedScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import kotlin.LateinitKt;

@Autonomous(name="Red Truss Side Auto")
public class RedTrussAuto extends LinearOpMode {
    OpenCvWebcam webcam;

    private DcMotorEx intakeMotor;
    public DcMotorEx outtakeMotor;
    public static double servotransferpos = 0.88;
    public static double servohold = 0.65;
    public static double servointake5pos = 0.41 + 0.01; //0.41
    public static double servointake4pos = 0.395 + 0.0075; //0.395
    public static double servointake3pos = 0.38 + 0.01; //0.38
    public static double servointake2pos = 0.355 + 0.01; //0.355
    public static double servointake1pos = 0.33 + 0.01;//0.33

    private PIDController Ocontroller;
    public static int Otarget = -25;
    public static double Op = 0.012, Oi = 0, Od = 0.0002;
    public static double Of = -0.08;


    enum flip{
        initializeflip,
        intake5flip,
        intake4flip,
        intake3flip,
        intake2flip,
        intake1flip,
        transferflip,
        transferflip1,
        betweenflip
    }


    public static double servoPosition = servohold;
    flip flip = initializeflip;
    public static double maxvel = 15;
    public static double maxaccel = 15;

    public static double maxvel2 = 30;
    public static double maxaccel2 = 30;
    private ElapsedTime timer = new ElapsedTime();


    enum State {
        PRELOAD_YELLOW,            // Drop yellow preload on backdrop
        PRELOAD_YELLOW_INTER,
        PRELOAD_YELLOW_DROP,           // Drop yellow preload on backdrop
        PRELOAD_PURPLE,    // Drop purple preload on ground
        PRELOAD_PURPLE_INTER,    // Drop purple preload on ground
        PRELOAD_PURPLE_DROP,   // Drop purple preload on ground
        CYCLE1_PRE,        // Move servos into proper position and drive center
        CYCLE1_PRE2,
        CYCLE1_INTAKE,     // Extend intake and intake pixels
        CYCLE1_INTER1,      // Start path to backdrop
        SPIT1,
        SPIT2,
        SPIT3,
        SPIT4,
        SPIT5,
        SPIT6,
        SPIT7,
        SPIT8,
        SPIT9,
        SPIT10,
        SPIT11,
        SPIT12,
        SPIT13,
        SPIT14,
        SPIT15,
        SPIT16,
        CYCLE1_INTER2,
        CYCLE1_TRANSFER1,   // Retract intake and transfer pixels
        CYCLE1_TRANSFERINTER,
        CYCLE1_TRANSFER1_5,
        CYCLE1_TRANSFER2,   // Retract intake and transfer pixels
        CYCLE1_TRANSFER3,
        CYCLE1_TRANSFER4,
        CYCLE1_DEPOSIT,    // Drop pixels on backdrop
        CYCLE2_INTER_5,
        CYCLE2_INTER1,
        CYCLE2_INTAKE,
        CYCLE2_INTER2,
        CYCLE2_TRANSFER1,
        CYCLE2_TRANSFERINTER,
        CYCLE2_TRANSFER1_5,
        CYCLE2_TRANSFER2,
        CYCLE2_TRANSFER3,
        CYCLE2_DEPOSIT,
        CYCLE3_INTER_5,
        CYCLE3_INTER1,
        CYCLE3_INTAKE,
        CYCLE3_INTER2,
        CYCLE3_TRANSFER1,
        CYCLE3_TRANSFERINTER,
        CYCLE3_TRANSFERINTER2,
        CYCLE3_TRANSFER2,
        CYCLE3_TRANSFER3,
        CYCLE3_DEPOSIT,
        CYCLE1_REATTEMPT1,
        CYCLE1_REATTEMPT2,
        CYCLE1_REATTEMPT3,
        PARK,

        IDLE,              // Our bot will enter the IDLE state when done
    }

    private boolean left = true;
    private PIDController Lcontroller;
    public static double Lp = 0.006, Li = 0, Ld = 0.0001;
    private DcMotorEx intakeLeftExt;
    private DcMotorEx intakeRightExt;


    //Ltarget Max 750, Min -75
    public static int Ltarget;
    private IMU imu;


    State state = PRELOAD_YELLOW;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    private Servo switchservo;

    public boolean intake1 = false;
    public boolean intake2 = false;

    private Servo pivot1;
    private Servo pivot2;
    private Servo flip1;
    private Servo flip2;
    private Servo pan;
    private Servo wrist;
    private Servo claw1;
    private Servo claw2;
    private Servo latch;

    public static double pivotpos;
    public static double flip1pos = 0.26;
    public static double wristpos = 0.87;
    public static double claw1pos = 0.9;
    public static double claw2pos = 0.1;
    public static double panpos = 0.47;

    public static double intakePower = 0;
    public double counter = 0;
    public double panpositioncalc;


    public TouchSensor intSensor1;
    public TouchSensor intSensor2;


    @Override
    public void runOpMode() throws InterruptedException {
        Lcontroller = new PIDController(Lp,Li,Ld);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        intakeLeftExt = hardwareMap.get(DcMotorEx.class, "22");
        intakeRightExt = hardwareMap.get(DcMotorEx.class, "1");

        switchservo = hardwareMap.get(Servo.class, "switch");
        //switch is 5ss

        intakeMotor = hardwareMap.get(DcMotorEx.class, "0");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "33");


        intSensor1 = hardwareMap.get(TouchSensor.class, "0t");
        intSensor2 = hardwareMap.get(TouchSensor.class, "1t");

        pivot1 = hardwareMap.get(Servo.class, "3s");
        pivot2 = hardwareMap.get(Servo.class, "0ss");
        flip1 = hardwareMap.get(Servo.class, "4s");
        flip2 = hardwareMap.get(Servo.class, "5s");

        wrist = hardwareMap.get(Servo.class, "4ss");
        claw1 = hardwareMap.get(Servo.class, "0s");
        claw2 = hardwareMap.get(Servo.class, "1s");
        pan = hardwareMap.get(Servo.class, "2ss");

        intakeLeftExt.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        latch = hardwareMap.get(Servo.class, "1ss");

        intakeLeftExt.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        switchservo.setPosition(0.35);

        Lcontroller = new PIDController(Lp,Li,Ld);
        Ocontroller = new PIDController(Op,Oi,Od);

        wrist.setPosition(0.87);
        pan.setPosition(0.47);
        wrist.setPosition(0.87);
        pan.setPosition(0.47);
        latch.setPosition(0.7);

        pivot1.setPosition(0.65);
        pivot2.setPosition(1-0.65);
        flip1.setPosition(0.26);
        flip2.setPosition(1-0.26);
        TrajectorySequence preload1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(5, 0), Math.toRadians(0))
                .splineTo(new Vector2d(21.5, 8), Math.toRadians(50))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(35, -38.75, Math.toRadians(90)))
                .build();

        TrajectorySequence preload2 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(27, -7, Math.toRadians(55)))
                .setReversed(true)
                .back(2)
                .splineTo(new Vector2d(27.5, -38.75), Math.toRadians(-90))
                .build();

        TrajectorySequence preload3 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(23, -13, Math.toRadians(45)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(23, -38, Math.toRadians(90)))
                .build();

        //cycles
        TrajectorySequence preload2_v1_5 = drive.trajectorySequenceBuilder(preload1.end())
                .setReversed(false)
                .splineTo(new Vector2d(-1, 18), Math.toRadians(93))
                .lineTo(new Vector2d(-1, 40))
                .splineTo(new Vector2d(15.5, 69), Math.toRadians(80))
                .build();

        //cycle2
        TrajectorySequence preload2_v2_5 = drive.trajectorySequenceBuilder(preload2.end())
                .setReversed(false)
                .splineTo(new Vector2d(-1, -18), Math.toRadians(93))
                .lineTo(new Vector2d(-1, 40))
                .splineTo(new Vector2d(16.5, 69), Math.toRadians(80))
                .build();

        //cycle 3
        TrajectorySequence preload2_v3_5 = drive.trajectorySequenceBuilder(preload3.end())
                .setReversed(false)
                .splineTo(new Vector2d(-1, -18), Math.toRadians(93))
                .lineTo(new Vector2d(-1, 40))
                .splineTo(new Vector2d(15.5, 69), Math.toRadians(80))

                .build();


        TrajectorySequence deposit_v1 = drive.trajectorySequenceBuilder(preload2_v1_5.end())
                .setReversed(true)
                .splineTo(new Vector2d(-1, 40), Math.toRadians(-87))
                .splineTo(new Vector2d(-1, 0), Math.toRadians(-87))
                .splineTo(new Vector2d(22, -37.75), Math.toRadians(-87))
                .build();

        TrajectorySequence deposit_v2 = drive.trajectorySequenceBuilder(preload2_v2_5.end())
                .setReversed(true)
                .splineTo(new Vector2d(-1, 40), Math.toRadians(-87))
                .splineTo(new Vector2d(-1, 0), Math.toRadians(-87))
                .splineTo(new Vector2d(22, -37.75), Math.toRadians(-87))
                .build();

        TrajectorySequence deposit_v3 = drive.trajectorySequenceBuilder(preload2_v3_5.end())
                .setReversed(true)
                .splineTo(new Vector2d(-1, 40), Math.toRadians(-87))
                .splineTo(new Vector2d(-1, 0), Math.toRadians(-87))
                .splineTo(new Vector2d(22, -37), Math.toRadians(-87))
                .build();


        TrajectorySequence cycle2_v1 = drive.trajectorySequenceBuilder(deposit_v1.end())
                .setReversed(false)
                .splineTo(new Vector2d(-1.5, -18), Math.toRadians(90))
                .lineTo(new Vector2d(-1.5, 40))
                .splineTo(new Vector2d(13, 68.25), Math.toRadians(80))
                .build();

        TrajectorySequence cycle2_v2 = drive.trajectorySequenceBuilder(deposit_v2.end())
                .setReversed(false)
                .splineTo(new Vector2d(-1.5, -18), Math.toRadians(90))
                .lineTo(new Vector2d(-1.5, 40))
                .splineTo(new Vector2d(13.5, 68.25), Math.toRadians(80))
                .build();

        TrajectorySequence cycle2_v3 = drive.trajectorySequenceBuilder(deposit_v3.end())
                .setReversed(false)
                .splineTo(new Vector2d(-1.5, -18), Math.toRadians(90))
                .lineTo(new Vector2d(-1.5, 40))
                .splineTo(new Vector2d(13, 68.75), Math.toRadians(80))
                .build();


        TrajectorySequence deposit_v1_2 = drive.trajectorySequenceBuilder(cycle2_v1.end())
                .setReversed(true)
                .splineTo(new Vector2d(-3, 40), Math.toRadians(-87))
                .splineTo(new Vector2d(-3, 0), Math.toRadians(-87))
                .splineTo(new Vector2d(23, -38.5), Math.toRadians(-87))
                .waitSeconds(0.25)
                .build();

        TrajectorySequence deposit_v2_2 = drive.trajectorySequenceBuilder(cycle2_v2.end())
                .setReversed(true)
                .splineTo(new Vector2d(-3, 40), Math.toRadians(-87))
                .splineTo(new Vector2d(-3, 0), Math.toRadians(-87))
                .splineTo(new Vector2d(23, -37.75), Math.toRadians(-87))
                .waitSeconds(0.25)
//                .splineTo(new Vector2d(2.5, 43.5), Math.toRadians(45))
                .build();

        TrajectorySequence deposit_v3_2 = drive.trajectorySequenceBuilder(cycle2_v3.end())
                .setReversed(true)
                .splineTo(new Vector2d(-3, 40), Math.toRadians(-87))
                .splineTo(new Vector2d(-3, 0), Math.toRadians(-87))
                .splineTo(new Vector2d(23, -38), Math.toRadians(-87))
                .waitSeconds(0.25)
                .build();


        TrajectorySequence cycle3_v1 = drive.trajectorySequenceBuilder(deposit_v1_2.end())
                .lineTo(new Vector2d(0, -34))
                .build();



        telemetry.addData("Status", "Initialized");
        telemetry.update();
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        BlueScanner scanner = new BlueScanner(telemetry);
//        webcam.setPipeline(scanner);
//
//        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
//
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//            }
//        });

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        RedScanner scanner = new RedScanner(telemetry);
        webcam.setPipeline(scanner);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode)
            {
            }
        });

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        telemetry.setMsTransmissionInterval(50);
        wristpos = 0.87;

        waitForStart();
//        BlueScanner scanner = null;
//        Barcode result = scanner.getResult();
//        switch (result) {
//            case LEFT:
//                telemetry.addData("Detected", "Left");
//                break;
//            case MIDDLE:
//                telemetry.addData("Detected", "Middle");
//                break;
//            case RIGHT:
//                telemetry.addData("Detected", "Right");
//                break;
//        }
        Barcode result = scanner.getResult();
        switch (result) {
            case LEFT:
                telemetry.addData("Detected", "Left");
                break;
            case MIDDLE:
                telemetry.addData("Detected", "Middle");
                break;
            case RIGHT:
                telemetry.addData("Detected", "Right");
                break;
        }



        wristpos = 0.87;
        claw1pos = 0.9;
        claw2pos = 0.12;

        if (isStopRequested()) return;

        state = PRELOAD_YELLOW;
        timer.reset();
//        result = Barcode.LEFT;

        while (opModeIsActive() && !isStopRequested()) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double angle = Math.toDegrees(currentHeading);

            pivot1.setPosition(servoPosition);
            pivot2.setPosition(1-servoPosition);
            intakeMotor.setPower(intakePower);
            pan.setPosition(panpos);
            wrist.setPosition(wristpos);
            flip1.setPosition(flip1pos);
            flip2.setPosition(1-flip1pos);

            claw1.setPosition(claw2pos);

            claw2.setPosition(claw1pos);

            switch(state){
                case PRELOAD_YELLOW:
                    latch.setPosition(0.7);
                    if(result == Barcode.LEFT){
                        drive.followTrajectorySequenceAsync(preload1);
                        timer.reset();
                        state = PRELOAD_YELLOW_INTER;
                    }
                    if(result == Barcode.MIDDLE){
                        drive.followTrajectorySequenceAsync(preload2);
                        timer.reset();
                        state = PRELOAD_YELLOW_INTER;
                    }
                    if(result == Barcode.RIGHT){
                        drive.followTrajectorySequenceAsync(preload3);
                        timer.reset();
                        state = PRELOAD_YELLOW_INTER;
                    }
                    break;

                case PRELOAD_YELLOW_INTER:
                    if(timer.seconds() > 2.5){
                        servoPosition = 0.6;
                    }
                    if(timer.seconds() > 2.7){
                        flip1pos = 0.81;
                    }
                    if(timer.seconds() > 2.9){
                        wristpos = 0.53;
                    }
                    if(!drive.isBusy() && timer.seconds() > 3){
                        claw1pos = 0.4;
                        claw2pos = 0.6;
                        timer.reset();
                        state = PRELOAD_YELLOW_DROP;
                    }
                    break;

                case PRELOAD_YELLOW_DROP:
                    if(timer.seconds() > 0.25){
                        if(result == Barcode.LEFT){
                            drive.followTrajectorySequenceAsync(preload2_v1_5);
                            timer.reset();
                            state = CYCLE1_PRE;
                        }
                        if(result == Barcode.MIDDLE){
                            drive.followTrajectorySequenceAsync(preload2_v2_5);
                            timer.reset();
                            state = CYCLE1_PRE;
                        }
                        if(result ==  Barcode.RIGHT){
                            drive.followTrajectorySequenceAsync(preload2_v3_5);
                            timer.reset();
                            state = CYCLE1_PRE;
                        }
                    }
                    break;

                case CYCLE1_PRE:
                    if(timer.seconds() > 0.5){
                        wristpos = 0.87;
                    }
                    if(timer.seconds() > 1){
                        flip1pos = 0.4;
                    }
                    if(timer.seconds() > 1.25){
                        flip1pos = 0.26;
                    }
                    if(timer.seconds() > 2){
                        servoPosition = servointake5pos;
                    }
                    if(timer.seconds() > 3) {
                        intakePower = -0.75;
                        timer.reset();
                        state = CYCLE1_PRE2;
                    }

                    break;

                case CYCLE1_PRE2:
                    intake1 = intSensor2.isPressed();
                    intake2 = intSensor1.isPressed();

                    if((intake1 || intake2)) {
                        servoPosition = servointake4pos;
                    }

                    if(intake1 && intake2 && !drive.isBusy()) {
                        intakePower = 0.25;
                        intake1 = false;
                        intake2 = false;
                        timer.reset();
                        counter = 2;
                        if(result == Barcode.LEFT){
                            drive.followTrajectorySequenceAsync(deposit_v1);
                            timer.reset();
                            state = CYCLE1_TRANSFER1;
                        }
                        if(result == Barcode.MIDDLE){
                            drive.followTrajectorySequenceAsync(deposit_v2);
                            timer.reset();
                            state = CYCLE1_TRANSFER1;
                        }
                        if(result == Barcode.RIGHT){
                            drive.followTrajectorySequenceAsync(deposit_v3);
                            timer.reset();
                            state = CYCLE1_TRANSFER1;
                        }
                    }
                    break;

                case CYCLE1_TRANSFER1:
                    if(timer.seconds() > 0.2){
                        intakePower = -1;
                    }
                    if(timer.seconds() > 0.7){
                        latch.setPosition(0.7);
                        flip = transferflip;
                        timer.reset();
                        state = CYCLE1_TRANSFER1_5;
                    }
                    break;

                case CYCLE1_TRANSFER1_5:
                    if(timer.seconds() > 0.7){
                        Otarget = 35;
                        claw1pos = 0.9;
                        claw2pos = 0.12;
                        timer.reset();
                        state = CYCLE1_TRANSFER2;
                    }
                    break;

                case CYCLE1_TRANSFER2:
                    if(timer.seconds() > 0.35){
                        servoPosition = servointake3pos;
                        flip = intake3flip;
                        timer.reset();
                        if(counter == 2){
                            state = CYCLE1_INTAKE;
                        }
                        if (counter == 4){
                            state = SPIT7;
                        }
                        if (counter == 7){
                            state = SPIT13;
                        }
                        //here
                    }
                    break;

                case CYCLE1_INTAKE:
                    if(timer.seconds() > 1.1){
                        Otarget = 115;
                        flip1pos = 0.78;
                        timer.reset();
                        state = SPIT2;
                    }
                    break;

                case SPIT2:
                    panpositioncalc = -90.67756-5.256742*angle-0.1199626*Math.pow(angle, 2)-0.001355961*Math.pow(angle, 3)-0.000007597125*Math.pow(angle, 4)-1.68825*Math.pow(10, -8)*Math.pow(angle, 5);
                    if(timer.seconds() > 0.25){
                        panpos = 0.47;
                        wristpos = 0.53;
//                        panpos = Range.clip(panpositioncalc, 0.395, 0.54);
                        timer.reset();
                        state = SPIT3;
                    }
                    break;

                case SPIT3:
                    if(!drive.isBusy() && timer.seconds() > 0.3){
                        claw1pos = 0.4;
                        claw2pos = 0.6;
                        flip1pos = 0.81;
                        timer.reset();
                        state = SPIT4;
                    }
                    break;

                case SPIT4:
                    if(timer.seconds() > 0.25){
                        flip1pos = 0.78;
                        panpos = 0.47;
                        if(result == Barcode.LEFT){
                            drive.followTrajectorySequenceAsync(cycle2_v1);
                            timer.reset();
                            state = SPIT5;
                        }
                        if(result == Barcode.MIDDLE){
                            drive.followTrajectorySequenceAsync(cycle2_v2);
                            timer.reset();
                            state = SPIT5;
                        }
                        if(result == Barcode.RIGHT){
                            drive.followTrajectorySequenceAsync(cycle2_v3);
                            timer.reset();
                            state = SPIT5;
                        }
                    }
                    break;

                case SPIT5:
                    if(timer.seconds() > 0.25){
                        Otarget = 0;
                        flip1pos = 0.4;
                        timer.reset();
                        state = SPIT6;
                    }
                    break;

                case SPIT6:
                    if(timer.seconds() > 0.25){
                        flip1pos = 0.26;
                    }
                    if(timer.seconds() > 0.5){
                        wristpos = 0.87;
                    }
                    if(timer.seconds() > 1.5){
                        intakePower = -1;
                    }
                    intake1 = intSensor2.isPressed();
                    intake2 = intSensor1.isPressed();
                    if((intake1 || intake2)) {
                        servoPosition = servointake2pos;
                    }
                    if(timer.seconds() > 3 && (intake1 || intake2) && !(intake1 && intake2)){
//                        servoPosition = servointake1pos;
                    }

                    if(intake1 && intake2 && !drive.isBusy()) {
                        intakePower = 0.4;
                        servoPosition = servointake3pos;
                        intake1 = false;
                        intake2 = false;
                        timer.reset();
                        counter = 4;
                        if(result == Barcode.LEFT){
                            drive.followTrajectorySequenceAsync(deposit_v1_2);
                            timer.reset();
                            state = CYCLE1_TRANSFER1;
                        }
                        if(result == Barcode.MIDDLE){
                            drive.followTrajectorySequenceAsync(deposit_v2_2);
                            timer.reset();
                            state = CYCLE1_TRANSFER1;
                        }
                        if(result == Barcode.RIGHT){
                            drive.followTrajectorySequenceAsync(deposit_v3_2);
                            timer.reset();
                            state = CYCLE1_TRANSFER1;
                        }

                    }
                    break;

                case SPIT7:
                    if(timer.seconds() > 1.5){
                        Otarget = 175;
                        flip1pos = 0.78;
                        timer.reset();
                        state = SPIT8;
                    }
                    break;

                case SPIT8:
                    panpositioncalc = -90.67756-5.256742*angle-0.1199626*Math.pow(angle, 2)-0.001355961*Math.pow(angle, 3)-0.000007597125*Math.pow(angle, 4)-1.68825*Math.pow(10, -8)*Math.pow(angle, 5);
                    if(timer.seconds() > 0.25){
                        panpos = 0.47;
                        wristpos = 0.53;
//                        panpos = Range.clip(panpositioncalc, 0.395, 0.54);
                        timer.reset();
                        state = SPIT9;
                    }
                    break;

                case SPIT9:
                    if(!drive.isBusy() && timer.seconds() > 0.3){
                        claw1pos = 0.4;
                        claw2pos = 0.6;
                        flip1pos = 0.81;
                        timer.reset();
                        state = SPIT10;
                    }
                    break;

                case SPIT10:
                    if(timer.seconds() > 0.25){
                        flip1pos = 0.78;
                        panpos = 0.47;
                        drive.followTrajectorySequenceAsync(cycle3_v1);
                        timer.reset();
                        state = SPIT11;
                    }
                    break;

                case SPIT11:
                    if(timer.seconds() > 0.25){
                        wristpos = 0.87;
                        Otarget = 0;
                        flip1pos = 0.4;
                        timer.reset();
                        state = SPIT12;
                        servoPosition = servointake5pos;
                    }
                    break;

                case SPIT12:
                    if(timer.seconds() > 0.25){
                        flip1pos = 0.26;
                    }
                    break;

                case SPIT13:
                    if(timer.seconds() > 2.75){
                        Otarget = 400;
                        flip1pos = 0.81;
                        timer.reset();
                        state = SPIT14;
                    }
                    break;

                case SPIT14:
                    panpositioncalc = -90.67756-5.256742*angle-0.1199626*Math.pow(angle, 2)-0.001355961*Math.pow(angle, 3)-0.000007597125*Math.pow(angle, 4)-1.68825*Math.pow(10, -8)*Math.pow(angle, 5);
                    if(timer.seconds() > 0.25){
                        panpos = Range.clip(panpositioncalc, 0.395, 0.54);
                        timer.reset();
                        state = SPIT15;
                    }
                    break;

                case SPIT15:
                    if(!drive.isBusy() && timer.seconds() > 0.3){
                        claw1pos = 0.4;
                        claw2pos = 0.6;
                        timer.reset();
                        state = SPIT16;
                    }
                    break;
            }

            Lcontroller.setPID(Lp, Li, Ld);

            int armPos = intakeLeftExt.getCurrentPosition();

            double Lpid = Lcontroller.calculate(armPos, Ltarget);

            double Lpower = Lpid;

            intakeLeftExt.setPower(Lpower);
            intakeRightExt.setPower(Lpower);

            Ocontroller.setPID(Op, Oi, Od);

            int outPos = outtakeMotor.getCurrentPosition();

            double Opid = Ocontroller.calculate(outPos, -Otarget);
            double Off = Of;
            double Opower = Opid + Off;

            outtakeMotor.setPower(Opower);


            switch (flip) {
                case initializeflip:
                    flip = betweenflip;
                    break;

                case transferflip:
                    servoPosition = servotransferpos;
//                    generateMotionProfile(pivot1.getPosition(), servotransferpos, maxvel2, maxaccel2);
//                    startTime = getRuntime();
                    flip = betweenflip;
                    break;

                case intake1flip:
                    servoPosition = servointake1pos;
//                    generateMotionProfile(pivot1.getPosition(), servointake1pos, maxvel, maxaccel);
//                    startTime = getRuntime();
                    flip = betweenflip;
                    break;

                case intake2flip:
                    servoPosition = servointake2pos;
//                    generateMotionProfile(pivot1.getPosition(), servointake2pos, maxvel, maxaccel);
//                    startTime = getRuntime();
                    flip = betweenflip;
                    break;

                case intake3flip:
                    servoPosition = servointake3pos;
//                    generateMotionProfile(pivot1.getPosition(), servointake3pos, maxvel, maxaccel);
//                    startTime = getRuntime();
                    flip = betweenflip;
                    break;

                case intake4flip:
                    servoPosition = servointake4pos;
//                    generateMotionProfile(pivot1.getPosition(), servointake4pos, maxvel, maxaccel);
//                    startTime = getRuntime();
                    flip = betweenflip;
                    break;

                case intake5flip:
                    servoPosition = servointake5pos;
//                    generateMotionProfile(pivot1.getPosition(), servointake5pos, maxvel, maxaccel);
//                    startTime = getRuntime();
                    flip = betweenflip;
                    break;

                case betweenflip:

                    break;
            }

            drive.update();

        }
    }
}