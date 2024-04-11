package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.CYCLE1_DEPOSIT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.CYCLE1_PRE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.CYCLE1_TRANSFER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.CYCLE1_TRANSFER1_5;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.CYCLE1_TRANSFER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.CYCLE1_TRANSFER4;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.PRELOAD_YELLOW_DROP;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.PRELOAD_YELLOW_INTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.inter2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.inter3;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.inter4;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.inter5;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload10;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload11;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload12;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload13;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload14;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload15;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload16;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload17;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload18;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload19;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload20;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload3;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload4;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload5;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload55;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload6;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload9;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload9_5;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.newpreload9_75;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.stack1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.stack3;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.stack4;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.State.stack5;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.flip.betweenflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.flip.initializeflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.flip.intake3flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.flip.intake5flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.redpegging.flip.transferflip;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.Barcode;
import org.firstinspires.ftc.teamcode.vision.BlueScanner;
import org.firstinspires.ftc.teamcode.vision.RedScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="red auto")
public class redpegging extends LinearOpMode {
    OpenCvWebcam webcam;

    private DcMotorEx intakeMotor;
    public DcMotorEx outtakeMotor;
    public static double servotransferpos = 0.88;
    public static double servohold = 0.65;
    public static double flipdrive = 0.42;
    public static double servointake5pos = 0.395;
    public static double servointake4pos = 0.38;
    public static double servointake3pos = 0.355;
    public static double servointake2pos = 0.345;
    public static double servointake1pos = 0.32;

    private FtcDashboard dashboard = FtcDashboard.getInstance();


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

    public static double counter = 0;
    private ElapsedTime timer = new ElapsedTime();
    enum State {
        newpreload1,
        newpreload2,
        newpreload3,
        newpreload4,
        newpreload5,
        newpreload55,
        newpreload6,
        newpreload7,
        newpreload8,
        newpreload9,
        newpreload9_5,
        newpreload9_75,
        newpreload10,
        newpreload11,
        newpreload12,
        newpreload13,
        newpreload14,
        newpreload15,
        newpreload16,
        newpreload17,
        newpreload18,
        newpreload19,
        newpreload20,
        stack5,
        inter5,
        stack4,
        inter4,
        stack3,
        inter3,
        stack2,
        inter2,
        stack1,
        inter1,


        PRELOAD_YELLOW,            // Drop yellow preload on backdrop
        PRELOAD_YELLOW_INTER,
        PRELOAD_YELLOW_DROP,           // Drop yellow preload on backdrop
        PRELOAD_PURPLE,    // Drop purple preload on ground
        PRELOAD_PURPLE_INTER,    // Drop purple preload on ground
        PRELOAD_PURPLE_DROP,   // Drop purple preload on ground
        CYCLE1_PRE,        // Move servos into proper position and drive center
        CYCLE1_INTAKE,     // Extend intake and intake pixels
        CYCLE1_INTER1,      // Start path to backdrop
        SPIT1,
        SPIT2,
        SPIT3,
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
    private PIDController Lcontroller;
    public static double Lp = 0.006, Li = 0, Ld = 0.0001;
    private DcMotorEx intakeLeftExt;
    private DcMotorEx intakeRightExt;


    public static int intakeextendposition = 930;
    public static int intakeretractposition = 700;
    //Ltarget Max 750, Min -75
    public static int Ltarget;
    private IMU imu;


    State state = newpreload1;
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

    public static boolean intakepreload = false;
    public static boolean intakecycle1 = false;
    public static boolean intakecycle2 = false;

    public static double pivotpos;
    public static double flip1pos = 0.5;
    public static double wristpos = 0.87;
    public static double claw1pos = 0.4;
    public static double claw2pos = 0.1;
    public static double panpos = 0.47;

    public static double intakePower = 0;

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

        intakeMotor = hardwareMap.get(DcMotorEx.class, "0");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "33");


        intSensor1 = hardwareMap.get(TouchSensor.class, "0t");
        intSensor2 = hardwareMap.get(TouchSensor.class, "1t");

        pivot1 = hardwareMap.get(Servo.class, "3s");
        pivot2 = hardwareMap.get(Servo.class, "0ss");
        flip1 = hardwareMap.get(Servo.class, "4s");
        flip2 = hardwareMap.get(Servo.class, "5s");
        latch = hardwareMap.get(Servo.class, "1ss");

        wrist = hardwareMap.get(Servo.class, "4ss");
        claw1 = hardwareMap.get(Servo.class, "0s");
        claw2 = hardwareMap.get(Servo.class, "1s");
        pan = hardwareMap.get(Servo.class, "2ss");

        intakeLeftExt.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        switchservo.setPosition(0.35);

        Lcontroller = new PIDController(Lp,Li,Ld);
        Ocontroller = new PIDController(Op,Oi,Od);

        claw1.setPosition(0.1);
        claw2.setPosition(0.4);
        wrist.setPosition(0.87);
        pan.setPosition(0.47);
        latch.setPosition(0.7);

        flip1.setPosition(0.5);
        flip2.setPosition(1-0.5);

        pivot1.setPosition(0.65);
        pivot2.setPosition(1-0.65);

        TrajectorySequence preload1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(30, 0), Math.toRadians(0))
                .splineTo(new Vector2d(39.5, 20), Math.toRadians(51))
//                .splineToLinearHeading(new Pose2d(40, -23.5, Math.toRadians(-71.5)), Math.toRadians(0))
                .build();

        TrajectorySequence preload2 = drive.trajectorySequenceBuilder(startPose)
//                .splineTo(new Vector2d(5, 0), Math.toRadians(0))
//                .splineTo(new Vector2d(25, -10), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(42.5, 18, Math.toRadians(66)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(44, -23, Math.toRadians(-71.5)), Math.toRadians(0))
//                .addSpatialMarker(new Vector2d(42, -19.5), () -> {
//                    intakepreload = true;
//                    // This marker runs at the point that gets
//                    // closest to the (20, 20) coordinate
//
//                    // Run your action in here!
//                })

                .build();


        TrajectorySequence preload3 = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(35, 3.5, Math.toRadians(65.5)), Math.toRadians(0))
                .addSpatialMarker(new Vector2d(34, 3), () -> {
                    intakepreload = true;
                    // This marker runs at the point that gets
                    // closest to the (20, 20) coordinate; 16,6

                    // Run your action in here!
                })
                .build();


        //cycles
        TrajectorySequence preload2_v1_5 = drive.trajectorySequenceBuilder(preload1.end())
//                .lineToLinearHeading(new Pose2d(50, -8, Math.toRadians(-90)))
                .splineToLinearHeading(new Pose2d(50, 20, Math.toRadians(90)), Math.toRadians(0))
                .waitSeconds(0.01)
                .setReversed(true)
                .splineTo(new Vector2d(50, -50), Math.toRadians(-95))
                .splineToConstantHeading(new Vector2d(37, -85.25), Math.toRadians(-95))
                .build();

        TrajectorySequence preload2_v2_5 = drive.trajectorySequenceBuilder(preload2.end())
                .splineToLinearHeading(new Pose2d(50, 15, Math.toRadians(90)), Math.toRadians(0))
//                .lineToLinearHeading(new Pose2d(50, -8, Math.toRadians(-90)))
                .waitSeconds(0.01)
                .setReversed(true)
                .splineTo(new Vector2d(50, -50), Math.toRadians(-95))
                .splineToConstantHeading(new Vector2d(33, -84.75), Math.toRadians(-95))
                .build();

        TrajectorySequence preload2_v3_5 = drive.trajectorySequenceBuilder(preload3.end())
                .splineToLinearHeading(new Pose2d(50, 8, Math.toRadians(90)), Math.toRadians(0))
                .waitSeconds(0.01)
                .setReversed(true)
                .splineTo(new Vector2d(50, -50), Math.toRadians(-95))
                .splineToConstantHeading(new Vector2d(29, -84.75), Math.toRadians(-95))

                .build();


        TrajectorySequence preloadinter1 = drive.trajectorySequenceBuilder(preload2_v1_5.end())
                .lineToLinearHeading(new Pose2d(53, -60, Math.toRadians(90)))
                .build();

        TrajectorySequence preloadinter2 = drive.trajectorySequenceBuilder(preload2_v2_5.end())
                .lineToLinearHeading(new Pose2d(53, -60, Math.toRadians(90)))
                .build();

        TrajectorySequence preloadinter3 = drive.trajectorySequenceBuilder(preload2_v3_5.end())
                .lineToLinearHeading(new Pose2d(53, -60, Math.toRadians(90)))
                .build();


        TrajectorySequence cycle2_v1 = drive.trajectorySequenceBuilder(preloadinter1.end())
                /*.splineTo(new Vector2d(48.5, 60), Math.toRadians(-92))
                .splineTo(new Vector2d(48.5, 23), Math.toRadians(-92))
                .splineTo(new Vector2d(48.5, 15), Math.toRadians(-92))*/

//                .lineToLinearHeading(new Pose2d(48, 60, Math.toRadians(-93)))
                .lineToLinearHeading(new Pose2d(53, -15, Math.toRadians(89)))
  //              .lineToLinearHeading(new Pose2d(48,15, Math.toRadians(-93)))
                .addSpatialMarker(new Vector2d(53, -18), () -> {
                    intakecycle1 = true;
                    // This marker runs at the point that gets
                    // closest to the (20, 20) coordinate

                    // Run your action in here!
                })
                .build();


        TrajectorySequence deposit_v1_2 = drive.trajectorySequenceBuilder(cycle2_v1.end())
                .setReversed(true)
                .splineTo(new Vector2d(53, -50), Math.toRadians(-89))
                .splineTo(new Vector2d(48, -87.5), Math.toRadians(235))
                .build();


        TrajectorySequence cycle3_v1 = drive.trajectorySequenceBuilder(deposit_v1_2.end())
                .setReversed(false)
                .splineTo(new Vector2d(53, -60), Math.toRadians(88))
                .splineTo(new Vector2d(53, -23), Math.toRadians(88))
                .splineTo(new Vector2d(53, -14), Math.toRadians(88))
                .addSpatialMarker(new Vector2d(52, -17), () -> {
                    intakecycle2 = true;
                    // This marker runs at the point that gets
                    // closest to the (20, 20) coordinate

                    // Run your action in here!
                })
                .build();

        TrajectorySequence deposit_v1_3 = drive.trajectorySequenceBuilder(cycle3_v1.end())
                .setReversed(true)
                .splineTo(new Vector2d(54, -50), Math.toRadians(-88))
                .splineTo(new Vector2d(48, -87.5), Math.toRadians(235))
                .build();



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


        if (isStopRequested()) return;

        state = newpreload1;
        timer.reset();

        intakepreload = false;

        while (opModeIsActive() && !isStopRequested()) {
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

                case newpreload1:
                    latch.setPosition(0.3);
                    if(result == Barcode.LEFT){
                        drive.followTrajectorySequenceAsync(preload1);
                        timer.reset();
                        state = newpreload2;
                    }
                    if(result == Barcode.MIDDLE){
                        drive.followTrajectorySequenceAsync(preload2);
                        timer.reset();
                        state = newpreload2;
                    }
                    if(result == Barcode.RIGHT){
                        drive.followTrajectorySequenceAsync(preload3);
                        timer.reset();
                        state = newpreload2;
                    }
                    break;

                case newpreload2:
                    if(timer.seconds() > 0.25){
                        servoPosition = servointake5pos;
                        Otarget = 150;
                    }
                    if(timer.seconds() > 0.35){
                        flip1pos = 1;
                    }
                    if(timer.seconds() > 0.45 && result == Barcode.LEFT){
                        wristpos = 0;
                    }
                    if(!drive.isBusy() && (result == Barcode.MIDDLE||result == Barcode.LEFT)){
                        timer.reset();
                        state = newpreload3;
                    }
                    if(intakepreload && timer.seconds() > 0.5){
                        timer.reset();
                        state = newpreload3;
                    }
                    break;

                case newpreload3:
                    if(timer.seconds() > 0){
                        claw1pos = 0.4;
                        claw2pos = 0.6;
                        intakePower = -1;
                        if(result == Barcode.RIGHT){
                            Ltarget = 530;
                        }
                        if(result == Barcode.MIDDLE){
                            Ltarget = 135;
                        }
                        if(result == Barcode.LEFT){
                            Ltarget = 130;
                        }
                        timer.reset();
                        state = newpreload4;
                    }
                    break;

                case newpreload4:
                    if(timer.seconds() > 0.1){
                        wristpos = 0.87;
                        Otarget = -25;
                    }
                    intake1 = intSensor2.isPressed();
                    intake2 = intSensor1.isPressed();

                    if(intake1 && intake2 && timer.seconds() > 0.15){
                        flip1pos = 0.4;
                        servoPosition = flipdrive;
                        intake1 = false;
                        intake2 = false;
                        Otarget = -25;
                        intakePower = -1;
                        Ltarget = -50;
                        timer.reset();
                        state = newpreload5;
                    }
                    break;


                case newpreload5:
                    flip1pos = 0.26;
                    if(Math.abs(intakeLeftExt.getCurrentPosition() - Ltarget) < 100){
                        if(result ==Barcode.LEFT){
                            drive.followTrajectorySequenceAsync(preload2_v1_5);
                            timer.reset();
                            counter = 5;
                            state = newpreload55;
                        }
                        if(result ==Barcode.MIDDLE){
                            drive.followTrajectorySequenceAsync(preload2_v2_5);
                            timer.reset();
                            counter = 5;
                            state = newpreload55;
                        }
                        if(result ==Barcode.RIGHT){
                            drive.followTrajectorySequenceAsync(preload2_v3_5);
                            timer.reset();
                            counter = 5;
                            state = newpreload55;
                        }
                    }
                    break;


                case newpreload55:
                    if(timer.seconds() > 0.5){
                        timer.reset();
                        counter = 5;
                        state = CYCLE1_TRANSFER1;
                    }
                    break;

                case newpreload6:
                    if(timer.seconds() > 2){
                        flip1pos = 0.81;
                    }

                    if(timer.seconds() > 2.25){
                        wristpos = 0.53;
                    }

                    if(!drive.isBusy() && timer.seconds() > 2.25){
                        claw1pos = 0.4;
                        claw2pos = 0.6;
                        timer.reset();
                        state = newpreload9_5;
                    }
                    break;

                case newpreload9_5:
                    if(timer.seconds() > 0.1){
                        if(result == Barcode.LEFT){
                            drive.followTrajectorySequenceAsync(preloadinter1);
                            timer.reset();
                            state = newpreload9_75;
                        }
                        if(result == Barcode.MIDDLE){
                            drive.followTrajectorySequenceAsync(preloadinter2);
                            timer.reset();
                            state = newpreload9_75;
                        }
                        if(result == Barcode.RIGHT){
                            drive.followTrajectorySequenceAsync(preloadinter3);
                            timer.reset();
                            state = newpreload9_75;
                        }
                    }
                    break;

                case newpreload9_75:
                    if(!drive.isBusy()){
                        timer.reset();
                        state = newpreload9;
                    }
                    break;

                case newpreload9:
                    drive.followTrajectorySequenceAsync(cycle2_v1);
                    timer.reset();
                    state = newpreload10;
                    break;

                case newpreload10:
                    //copy
                    if(timer.seconds() > 0.35){
                        Otarget = -25;
                        flip1pos = 0.4;
                        timer.reset();
                        state = newpreload11;
                    }
                    break;

                case newpreload11:
                    if(timer.seconds() > 0.3){
                        flip1pos = 0.26;
                        wristpos = 0.87;
                    }
                    if(intakecycle1 && timer.seconds() > 0.35){
                        timer.reset();
                        state = stack5;
                    }
                    break;

                case stack5:
                    servoPosition = servointake4pos;
                    panpos = 0.47;
                    Otarget = -25;
                    flip1pos = 0.26;
                    wristpos = 0.87;
                    claw1pos = 0.4;
                    claw2pos = 0.6;
                    if(timer.seconds() > 0){
                        intakePower = -1;
                        Ltarget = intakeextendposition;
                        intake1 = intSensor2.isPressed();
                        intake2 = intSensor1.isPressed();
                        if((intake1 || intake2)) {
                            intake1 = false;
                            intake2 = false;
                            timer.reset();
                            state = inter5;
                        }

                    }
                    break;

                case inter5:
//                    Ltarget = intakeretractposition;
//                    servoPosition = servointake3pos;
//                    intake1 = intSensor2.isPressed();
//                    intake2 = intSensor1.isPressed();
                    if(timer.seconds() > 0.05){
                        timer.reset();
                        state = stack4;
                    }
                    break;

                case stack4:
                    servoPosition = servointake3pos;
                    if(timer.seconds() > 0.05){
                        intakePower = -1;
//                        Ltarget = intakeextendposition;
                        intake1 = intSensor2.isPressed();
                        intake2 = intSensor1.isPressed();
                        if((intake1 && intake2)) {
                            drive.followTrajectorySequenceAsync(deposit_v1_2);
                            intake1 = false;
                            intake2 = false;
                            timer.reset();
                            state = inter4;
                        }

                    }
                    break;

                case inter4:
                    Ltarget = -50;
                    //transfer
                    if(timer.seconds() > 0.5){
                        timer.reset();
                        counter = 0;
                        state = CYCLE1_TRANSFER1;
                    }
                    break;

                case stack3:
                    if(timer.seconds() > 0){
                        timer.reset();
                        state = inter3;
                    }
                    break;

                case inter3:
                    if(timer.seconds()>0){
                        Otarget = 600;
                        flip1pos = .81;
                    }
                    if(timer.seconds()>0.1){
                        panpos = 0.34;
                    }
                    if(!drive.isBusy()&&timer.seconds()>0.15){
                        claw1pos = 0.4;
                        claw2pos = 0.6;
                        timer.reset();
                        state = newpreload12;
                    }
                    break;

                case newpreload12:
                    if(timer.seconds() > 0.35){
                        drive.followTrajectorySequenceAsync(cycle3_v1);
                        Otarget = -25;
                        flip1pos = 0.4;
                        timer.reset();
                        state = newpreload13;
                    }
                    break;

                case newpreload13:
                    if(timer.seconds() > 0.3){
                        flip1pos = 0.26;
                    }
                    if(intakecycle2 && timer.seconds() > 0.4){
                        timer.reset();
                        state = newpreload14;
                    }
                    break;

                case newpreload14:
                    servoPosition = servointake2pos;
                    panpos = 0.47;
                    Otarget = -25;
                    flip1pos = 0.26;
                    wristpos = 0.87;
                    claw1pos = 0.4;
                    claw2pos = 0.6;
                    if(timer.seconds() > 0){
                        intakePower = -1;
                        Ltarget = intakeextendposition - 35;
                        intake1 = intSensor2.isPressed();
                        intake2 = intSensor1.isPressed();
                        if((intake1 || intake2)) {
                            intake1 = false;
                            intake2 = false;
                            timer.reset();
                            state = newpreload15;
                        }

                    }
                    break;

                case newpreload15:
//                    Ltarget = intakeretractposition;
                    servoPosition = servointake3pos;
                    intake1 = intSensor2.isPressed();
                    intake2 = intSensor1.isPressed();
                    if(timer.seconds() > 0.05){
                        timer.reset();
                        state = newpreload16;
                    }
                    break;

                case newpreload16:
                    if(timer.seconds() > 0.05){
                        intakePower = -1;
//                        Ltarget = intakeextendposition;
//                        if(intakeLeftExt.getCurrentPosition() > (intakeextendposition - 150)){
//                            servoPosition = servointake1pos;
//                        }
                        servoPosition = servointake1pos;
                        intake1 = intSensor2.isPressed();
                        intake2 = intSensor1.isPressed();
                        if((intake1 && intake2)) {
                            drive.followTrajectorySequenceAsync(deposit_v1_3);
                            intake1 = false;
                            intake2 = false;
                            timer.reset();
                            state = newpreload17;
                        }

                    }
                    break;

                case newpreload17:
                    Ltarget = -50;
                    //transfer
                    if(timer.seconds() > 0.25){
                        timer.reset();
                        counter = 2;
                        state = CYCLE1_TRANSFER1;
                    }
                    break;

                case newpreload18:
                    if(timer.seconds() > 0){
                        timer.reset();
                        state = newpreload19;
                    }
                    break;

                case newpreload19:
                    if(timer.seconds()>0.25){
                        Otarget = 600;
                        flip1pos = .81;
                    }
                    if(timer.seconds()>0.5){
                        panpos = 0.34;
                    }
                    if(!drive.isBusy()&&timer.seconds()>0.65){
                        claw1pos = 0.4;
                        claw2pos = 0.6;
                        timer.reset();
                        state = newpreload20;
                    }
                    break;


                case stack2:
                    if(timer.seconds() > 0.5){
                        intakePower = -1;
                        servoPosition = servointake2pos;
                        Ltarget = intakeextendposition;
                        intake1 = intSensor2.isPressed();
                        intake2 = intSensor1.isPressed();
                        if((intake1 && intake2)) {
                            servoPosition = servointake4pos;
                            intake1 = false;
                            intake2 = false;
                            timer.reset();
                            state = inter2;
                        }

                    }
                    break;

                case inter2:
                    if(timer.seconds() < 0.1){
                        intakePower = 0.25;
                    }else{
                        intakePower = -0.5;
                    }
                    if(timer.seconds() > 0){
                        Otarget = -25;
                        Ltarget = -50;
                    }
                    if (timer.seconds() > 1) {
                        counter = 1;
                        timer.reset();
                        state = CYCLE1_TRANSFER1;
                    }
                    break;

                case PRELOAD_YELLOW:
                    servoPosition = servohold;
                    panpos = 0.47;
                    Otarget = -25;
                    Ltarget = -50;
                    flip1pos = 0.26;
                    wristpos = 0.87;
                    claw1pos = 0.9;
                    claw2pos = 0.1;
                    if(result == Barcode.LEFT){
                        drive.followTrajectorySequenceAsync(preload1);
                        timer.reset();
                        state = PRELOAD_YELLOW_INTER;
                    }else if(result == Barcode.MIDDLE){
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
                    if(timer.seconds() > 1){
                        flip1pos = 0.81;
                    }
                    if(timer.seconds() > 1.25){
                        wristpos = 0.51;
                    }
                    if(!drive.isBusy() && timer.seconds() > 1.5){
                        timer.reset();
                        state = PRELOAD_YELLOW_DROP;
                    }
                    break;

                case PRELOAD_YELLOW_DROP:
                    if(timer.seconds() > 0){
                        claw1pos = 0.4;
                        claw2pos = 0.6;
                        flip = intake5flip;
                        timer.reset();
                        state = CYCLE1_PRE;
                    }
                    break;



                case CYCLE1_TRANSFER1:
                    intakePower = -0.5;
                    if(timer.seconds() > 0.4){
                        flip = transferflip;
                        timer.reset();
                        state = CYCLE1_TRANSFER1_5;
                    }
                    break;

                case CYCLE1_TRANSFER1_5:
                    if(timer.seconds() > 0.7){
                        Otarget = 35;
                        claw1pos = 0.9;
                        claw2pos = 0.1;
                        timer.reset();
                        state = CYCLE1_TRANSFER2;
                    }
                    break;

                case CYCLE1_TRANSFER2:
                    if(timer.seconds() > 0.35){
                        servoPosition = servointake3pos;
                        flip = intake3flip;
                        timer.reset();
                        if(counter == 5){
                            state = newpreload6;
                        }
                        if(counter == 0){
                            state = stack3;
                        }
                        if(counter == 2){
                            state = newpreload18;
                        }

                        if(counter == 1){
                            state = stack1;
                        }
                        //here
                    }
                    break;



                case CYCLE1_TRANSFER3:
                    if(timer.seconds() > 0.3) {
                        flip = intake3flip;
                        Otarget = 350;
                        flip1pos = 0.81;
                        timer.reset();
                        state = CYCLE1_TRANSFER4;
                    }
                    break;

                case CYCLE1_TRANSFER4:
                    if(timer.seconds() > 0.2){
                        panpos = 0.34;
                    }
                    if(!drive.isBusy() && timer.seconds() > 0.51){
                        timer.reset();
                        state = CYCLE1_DEPOSIT;
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

            Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
            telemetry.update();

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