package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE1_DEPOSIT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE1_INTAKE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE1_INTER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE1_PRE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE1_TRANSFER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE1_TRANSFER1_5;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE1_TRANSFER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE1_TRANSFER3;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE1_TRANSFER4;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE1_TRANSFERINTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE2_DEPOSIT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE2_INTAKE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE2_INTER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE2_INTER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE2_INTER_5;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE2_TRANSFER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE2_TRANSFER1_5;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE2_TRANSFER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE2_TRANSFER3;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE2_TRANSFERINTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE3_DEPOSIT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE3_INTAKE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE3_INTER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE3_INTER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE3_TRANSFER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE3_TRANSFER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE3_TRANSFER3;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE3_TRANSFERINTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.CYCLE3_TRANSFERINTER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.PRELOAD_YELLOW;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.PRELOAD_YELLOW_DROP;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.PRELOAD_YELLOW_INTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.SPIT2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.State.SPIT3;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.flip.betweenflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.flip.initializeflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.flip.intake2flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.flip.intake3flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.flip.intake4flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.flip.intake5flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.BlueAuto2plus7.flip.transferflip;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.Barcode;
import org.firstinspires.ftc.teamcode.vision.BlueScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="autoautoatuaasdasd ")
public class BlueAuto2plus7 extends LinearOpMode {
    OpenCvWebcam webcam;

    private DcMotorEx intakeMotor;
    public DcMotorEx outtakeMotor;
    public static double servotransferpos = 0.88;
    public static double servohold = 0.65;
    public static double servointake5pos = 0.4;
    public static double servointake4pos = 0.385;
    public static double servointake3pos = 0.37;
    public static double servointake2pos = 0.355;
    public static double servointake1pos = 0.34;


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


    public static double servoPosition = servointake5pos;
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

        pivot1 = hardwareMap.get(Servo.class, "2s");
        pivot2 = hardwareMap.get(Servo.class, "3s");
        flip1 = hardwareMap.get(Servo.class, "4s");
        flip2 = hardwareMap.get(Servo.class, "5s");
        latch = hardwareMap.get(Servo.class, "1ss");

        wrist = hardwareMap.get(Servo.class, "4ss");
        claw1 = hardwareMap.get(Servo.class, "0s");
        claw2 = hardwareMap.get(Servo.class, "1s");
        pan = hardwareMap.get(Servo.class, "2ss");

        intakeLeftExt.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        switchservo.setPosition(0.35);

        Lcontroller = new PIDController(Lp,Li,Ld);
        Ocontroller = new PIDController(Op,Oi,Od);

//        claw1.setPosition(0.9);
//        claw2.setPosition(0.1);
//        wrist.setPosition(0.87);
//        pan.setPosition(0.47);
//        latch.setPosition(0.3);
//        claw1.setPosition(0.9);
//        claw2.setPosition(0.1);
//        wrist.setPosition(0.87);
//        pan.setPosition(0.47);
//        latch.setPosition(0.3);


        TrajectorySequence preload1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(23, 12, Math.toRadians(-40)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(22, 39, Math.toRadians(-95)))
                .build();


        TrajectorySequence preload2 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(27, 6, Math.toRadians(-40)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(29, 39.5, Math.toRadians(-95)))
                .build();


        TrajectorySequence preload3 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(10, 0), Math.toRadians(0))
                .splineTo(new Vector2d(25, -4), Math.toRadians(-60))
                .waitSeconds(0.1)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(34.5, 40, Math.toRadians(-95)))
                .build();

        //cycles
        TrajectorySequence preload2_v1_5 = drive.trajectorySequenceBuilder(preload1.end())
                .setReversed(false)
                .splineTo(new Vector2d(48, 18), Math.toRadians(-93))
                .splineTo(new Vector2d(48, -20.5), Math.toRadians(-93))
                .build();

        TrajectorySequence deposit_v1 = drive.trajectorySequenceBuilder(preload2_v1_5.end())
                .setReversed(true)
                .splineTo(new Vector2d(48, 18), Math.toRadians(93))
                .splineTo(new Vector2d(42, 40), Math.toRadians(-235))
                .build();

        TrajectorySequence cycle2_v1 = drive.trajectorySequenceBuilder(deposit_v1.end())
                .setReversed(false)
                .splineTo(new Vector2d(47.5, 18), Math.toRadians(-93))
                .splineTo(new Vector2d(47.5, -20), Math.toRadians(-93))
                .build();


        TrajectorySequence deposit_v1_2 = drive.trajectorySequenceBuilder(cycle2_v1.end())
                .setReversed(true)
                .splineTo(new Vector2d(46, 18), Math.toRadians(95))
                .splineTo(new Vector2d(41, 40), Math.toRadians(-235))
                .build();


        TrajectorySequence cycle3_v1 = drive.trajectorySequenceBuilder(deposit_v1_2.end())
                .setReversed(false)
                .splineTo(new Vector2d(47, 18), Math.toRadians(-95))
                .splineTo(new Vector2d(47, -10), Math.toRadians(-95))
                .splineTo(new Vector2d(47, -22.5), Math.toRadians(-105))
                .build();

        TrajectorySequence deposit_v1_3 = drive.trajectorySequenceBuilder(cycle3_v1.end())
                .setReversed(true)
                .turn(Math.toRadians(16.5))
                .splineTo(new Vector2d(46, 18), Math.toRadians(95))
                .splineTo(new Vector2d(40, 40), Math.toRadians(-235))
                .build();


        //cycle2
        TrajectorySequence preload2_v2_5 = drive.trajectorySequenceBuilder(preload2.end())
                .setReversed(false)
                .splineTo(new Vector2d(47, 18), Math.toRadians(-93))
                .splineTo(new Vector2d(47, -20.5), Math.toRadians(-93))
                .build();

        TrajectorySequence deposit_v2 = drive.trajectorySequenceBuilder(preload2_v2_5.end())
                .setReversed(true)
                .splineTo(new Vector2d(47, 18), Math.toRadians(93))
                .splineTo(new Vector2d(42, 40), Math.toRadians(-235))
                .build();

        TrajectorySequence cycle2_v2 = drive.trajectorySequenceBuilder(deposit_v2.end())
                .setReversed(false)
                .splineTo(new Vector2d(47, 18), Math.toRadians(-93))
                .splineTo(new Vector2d(47, -20), Math.toRadians(-93))
                .build();

        TrajectorySequence deposit_v2_2 = drive.trajectorySequenceBuilder(cycle2_v2.end())
                .setReversed(true)
                .splineTo(new Vector2d(46, 18), Math.toRadians(95))
                .splineTo(new Vector2d(41, 40), Math.toRadians(-235))
                .build();

        TrajectorySequence cycle3_v2 = drive.trajectorySequenceBuilder(deposit_v2_2.end())
                .setReversed(false)
                .splineTo(new Vector2d(47, 18), Math.toRadians(-95))
                .splineTo(new Vector2d(47, -10), Math.toRadians(-95))
                .splineTo(new Vector2d(47, -22.5), Math.toRadians(-105))
                .build();

        TrajectorySequence deposit_v2_3 = drive.trajectorySequenceBuilder(cycle3_v2.end())
                .setReversed(true)
                .turn(Math.toRadians(16.5))
                .splineTo(new Vector2d(46, 18), Math.toRadians(95))
                .splineTo(new Vector2d(40, 40), Math.toRadians(-235))
                .build();


        //cycle 3
        TrajectorySequence preload2_v3_5 = drive.trajectorySequenceBuilder(preload3.end())
                .setReversed(false)
                .splineTo(new Vector2d(47, 18), Math.toRadians(-93))
                .splineTo(new Vector2d(47, -20.5), Math.toRadians(-93))
                .build();

        TrajectorySequence deposit_v3 = drive.trajectorySequenceBuilder(preload2_v3_5.end())
                .setReversed(true)
                .splineTo(new Vector2d(47.5, 18), Math.toRadians(93))
                .splineTo(new Vector2d(42, 40), Math.toRadians(-235))
                .build();

        TrajectorySequence cycle2_v3 = drive.trajectorySequenceBuilder(deposit_v3.end())
                .setReversed(false)
                .splineTo(new Vector2d(48.5, 18), Math.toRadians(-94))
                .splineTo(new Vector2d(48.5, -20), Math.toRadians(-94))
                .build();


        TrajectorySequence deposit_v3_2 = drive.trajectorySequenceBuilder(cycle2_v3.end())
                .setReversed(true)
                .splineTo(new Vector2d(46, 18), Math.toRadians(95))
                .splineTo(new Vector2d(41, 40), Math.toRadians(-235))
                .build();


        TrajectorySequence cycle3_v3 = drive.trajectorySequenceBuilder(deposit_v3_2.end())
                .setReversed(false)
                .splineTo(new Vector2d(47, 18), Math.toRadians(-95))
                .splineTo(new Vector2d(47, -10), Math.toRadians(-95))
                .splineTo(new Vector2d(47, -22.5), Math.toRadians(-105))
                .build();

        TrajectorySequence deposit_v3_3 = drive.trajectorySequenceBuilder(cycle3_v3.end())
                .setReversed(true)
                .turn(Math.toRadians(16.5))
                .splineTo(new Vector2d(46, 18), Math.toRadians(95))
                .splineTo(new Vector2d(40, 40), Math.toRadians(-235))
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



        if (isStopRequested()) return;

        state = PRELOAD_YELLOW;
        timer.reset();
//        result = Barcode.LEFT;

        while (opModeIsActive() && !isStopRequested()) {
//            pivot1.setPosition(servoPosition);
//            pivot2.setPosition(1-servoPosition);
//            intakeMotor.setPower(intakePower);
//            pan.setPosition(panpos);
//            wrist.setPosition(wristpos);
//            flip1.setPosition(flip1pos);
//            flip2.setPosition(1-flip1pos);
//
//
//            claw1.setPosition(claw1pos);
//            claw2.setPosition(claw2pos);

            switch(state){
                case PRELOAD_YELLOW:
                    if(left){
                        drive.followTrajectorySequenceAsync(preload1);
                        timer.reset();
                        state = PRELOAD_YELLOW_INTER;
                    }
                    break;

                case PRELOAD_YELLOW_INTER:
                    if(!drive.isBusy()){
                        timer.reset();
                        state = PRELOAD_YELLOW_DROP;
                    }
                    break;

                case PRELOAD_YELLOW_DROP:
                    if(left){
                        drive.followTrajectorySequenceAsync(preload2_v1_5);
                        timer.reset();
                        state = CYCLE1_PRE;
                    }
                    break;

                case CYCLE1_PRE:
                    if(!drive.isBusy()){
                        timer.reset();
                        state = CYCLE1_INTAKE;
                    }
                    break;

                case CYCLE1_INTAKE:
                    if(left){
                        drive.followTrajectorySequenceAsync(deposit_v1);
                        timer.reset();
                        state = SPIT2;
                    }
                    break;

                case SPIT2:
                    if(!drive.isBusy()){
                        timer.reset();
                        state = SPIT3;
                    }
                    break;

                case SPIT3:
                    if(left){
                        drive.followTrajectorySequenceAsync(cycle2_v1);
                        timer.reset();
                        state = CYCLE1_INTER2;
                    }
                    break;

                case CYCLE1_INTER2:
                    if(!drive.isBusy()){
                        timer.reset();
                        state = CYCLE1_TRANSFERINTER;
                    }
                    break;

                case CYCLE1_TRANSFERINTER:
                    if(timer.seconds() > 0.25){
                        timer.reset();
                        state = CYCLE1_TRANSFER1;
                    }
                    break;

                case CYCLE1_TRANSFER1:
                    if(left){
                        drive.followTrajectorySequenceAsync(deposit_v1_2);
                        timer.reset();
                        state = CYCLE1_TRANSFER1_5;
                    }
                    break;

                case CYCLE1_TRANSFER1_5:

                    break;

                case CYCLE1_TRANSFER2:
                    if(timer.seconds() > 0.5){
//                        latch.setPosition(0.3);
                        servoPosition = servointake3pos;
                        flip = intake3flip;
                        timer.reset();
                        state = CYCLE1_TRANSFER3;
                    }
                    break;

//                case CYCLE1_REATTEMPT1:
//                    if(timer.seconds()>0.25){
//                        servoPosition = servointake2pos;
//                    }
//                    if(timer.seconds()>0.35) {
//                        intakePower = -1;
//                        timer.reset();
//                        state = CYCLE1_REATTEMPT2;
//                    }
//                    break;
//
//                case CYCLE1_REATTEMPT2:
//                    if(!drive.isBusy()){
//                        Ltarget = 400;
//                        timer.reset();
//                        state = CYCLE1_REATTEMPT3;
//                    }
//                    break;
//
//                case CYCLE1_REATTEMPT3:
//                    if(timer.seconds()>0.5){
//                        Ltarget = -10;
//                        timer.reset();
//                        state = CYCLE1_TRANSFER1;
//                    }
//                    break;

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
                        panpos = 0.6;
                    }
                    if(!drive.isBusy() && timer.seconds() > 0.51){
                        timer.reset();
                        state = CYCLE1_DEPOSIT;
                    }
                    break;

                case CYCLE1_DEPOSIT:
                    if(timer.seconds() > 0.5){
                        claw1pos = 0.4;
                        claw2pos = 0.6;
                    }
                    if(timer.seconds() > 0.75){
                        panpos = 0.47;
                        flip1pos = 0.32;
                        Otarget = -25;
                        if(left){
                            drive.followTrajectorySequenceAsync(cycle2_v1);
                            timer.reset();
                            state = CYCLE2_INTER_5;
                        }
//                        if(result == Barcode.MIDDLE){
//                            drive.followTrajectorySequenceAsync(cycle2_v2);
//                            timer.reset();
//                            state = CYCLE2_INTER_5;
//                        }
//                        if(result == Barcode.RIGHT){
//                            drive.followTrajectorySequenceAsync(cycle2_v3);
//                            timer.reset();
//                            state = CYCLE2_INTER_5;
//                        }
                    }
                    break;

                case CYCLE2_INTER_5:
                    if(timer.seconds() > 2){
                        flip1pos = 0.26;
                        intakePower = -1;
//                        Ltarget = 1255;
                    }

                    if(!drive.isBusy() && timer.seconds() > 2.6){
                        timer.reset();
                        state = CYCLE2_INTER1;
                    }
                    break;

                case CYCLE2_INTER1:
                    if(timer.seconds() > 0.1){
                        flip = intake2flip;
                        timer.reset();
                        state = CYCLE2_INTAKE;
                    }
                    break;

                case CYCLE2_INTAKE:
                    intake1 = intSensor2.isPressed();
                    intake2 = intSensor1.isPressed();
                    if(timer.seconds() > 1){
                        servoPosition = servointake1pos;
                    }
                    if((intake1 && intake2)){
                        intakePower = 0.1;
//                        intake1 = false;
//                        intake2 = false;
                        timer.reset();
                        state = CYCLE2_INTER2;
                    }
                    break;

                case CYCLE2_INTER2:
                    if(timer.seconds() > 0.1){
//                        Ltarget = -25;
                        intakePower = -0.5;
                    }

                    if(timer.seconds() > 0.25){
                        if(left){
                            drive.followTrajectorySequenceAsync(deposit_v1_2);
                            timer.reset();
                            state = CYCLE2_TRANSFERINTER;
                        }
//                        if(result == Barcode.MIDDLE){
//                            drive.followTrajectorySequenceAsync(deposit_v2_2);
//                            timer.reset();
//                            state = CYCLE2_TRANSFERINTER;
//                        }
//                        if(result == Barcode.RIGHT){
//                            drive.followTrajectorySequenceAsync(deposit_v3_2);
//                            timer.reset();
//                            state = CYCLE2_TRANSFERINTER;
//                        }
                    }
                    break;

                case CYCLE2_TRANSFERINTER:
                    if(timer.seconds() > 0.5){
                        flip = transferflip;
                        timer.reset();
                        state = CYCLE2_TRANSFER1;
                    }
                    break;

                case CYCLE2_TRANSFER1:
                    if(timer.seconds() > 0.5){
                        timer.reset();
                        state = CYCLE2_TRANSFER1_5;
                    }
                    break;

                case CYCLE2_TRANSFER1_5:
                    if(timer.seconds() > 0.25){
                        Otarget = 25;
                    }
                    if(timer.seconds() > 0.75){
                        claw1pos = 0.9;
                        claw2pos = 0.1;
                        timer.reset();
                        state = CYCLE2_TRANSFER2;
                    }
                    break;

                case CYCLE2_TRANSFER2:
                    if(timer.seconds() > 0.5){
//                        latch.setPosition(0.3);
                        flip = intake3flip;
                        timer.reset();
                        state = CYCLE2_TRANSFER3;
                    }
                    break;

                case CYCLE2_TRANSFER3:
                    if(timer.seconds() > 0.5){
                        flip = intake5flip;
                        flip1pos = 0.81;
                        Otarget = 450;
                    }

                    if(timer.seconds() > 1){
                        panpos = 0.6;
                    }

                    if(!drive.isBusy() && timer.seconds() > 1.1){
                        timer.reset();
                        state = CYCLE2_DEPOSIT;
                    }
                    break;

                case CYCLE2_DEPOSIT:
                    if(timer.seconds() > 0.5){
                        claw1pos = 0.4;
                        claw2pos = 0.6;
                    }
                    if(timer.seconds() > 1){
                        panpos = 0.47;
                    }
                    if(timer.seconds() > 1.25){
                        flip1pos = 0.28;
                        Otarget = -25;
                    }
                    if(timer.seconds() > 1.5){
                        flip1pos = 0.26;
                    }
//
//                    if(timer.seconds() > 0.2){
//                        panpos = 0.47;
//                        flip1pos = 0.28;
//                        Otarget = -25;
//                        if(result == Barcode.LEFT){
//                            drive.followTrajectorySequenceAsync(cycle3_v1);
//                            timer.reset();
//                            state = CYCLE3_INTER_5;
//                        }
//                        if(result == Barcode.MIDDLE){
//                            drive.followTrajectorySequenceAsync(cycle3_v2);
//                            timer.reset();
//                            state = CYCLE3_INTER_5;
//                        }
//                        if(result == Barcode.RIGHT){
//                            drive.followTrajectorySequenceAsync(cycle3_v3);
//                            timer.reset();
//                            state = CYCLE3_INTER_5;
//                        }
//                    }
                    break;

                case CYCLE3_INTER_5:
                    if(timer.seconds() > 2.5){
//                        Ltarget = 1225;
                    }
                    if(!drive.isBusy()){
//                        Ltarget = 1225;
                        flip1pos = 0.26;
                        timer.reset();
                        state = CYCLE3_INTER1;
                    }
                    break;

                case CYCLE3_INTER1:
                    if((Math.abs(intakeLeftExt.getCurrentPosition() - Ltarget) < 50) && timer.seconds() > 0.2){
                        flip = intake4flip;
                        timer.reset();
                        state = CYCLE3_INTAKE;
                    }
                    break;

                case CYCLE3_INTAKE:
                    intake1 = intSensor2.isPressed();
                    intake2 = intSensor1.isPressed();
                    if(timer.seconds() > 1){
                        servoPosition = servointake3pos;
//                        generateMotionProfile(pivot1.getPosition(), servointake3pos, maxvel, maxaccel);
                    }
                    if((intake1 && intake2)){
                        intakePower = 0.1;
//                        intake1 = false;
//                        intake2 = false;
                        timer.reset();
                        state = CYCLE3_INTER2;
                    }
                    break;

                case CYCLE3_INTER2:
                    if(timer.seconds() > 0){
//                        Ltarget = -40;
                    }

                    if(timer.seconds() > 0.1 && Math.abs(intakeLeftExt.getCurrentPosition() - Ltarget) < 650){
                        intakePower = -0.5;
                        if(left){
                            drive.followTrajectorySequenceAsync(deposit_v1_3);
                            timer.reset();
                            state = CYCLE3_TRANSFERINTER;
                        }
//                        if(result == Barcode.MIDDLE){
//                            drive.followTrajectorySequenceAsync(deposit_v2_3);
//                            timer.reset();
//                            state = CYCLE3_TRANSFERINTER;
//                        }
//                        if(result == Barcode.RIGHT){
//                            drive.followTrajectorySequenceAsync(deposit_v3_3);
//                            timer.reset();
//                            state = CYCLE3_TRANSFERINTER;
//                        }
                    }
                    break;



                case CYCLE3_TRANSFERINTER:
                    if(timer.seconds() > 0.25 && Math.abs(intakeLeftExt.getCurrentPosition() - Ltarget) < 400) {
                        timer.reset();
                        state = CYCLE3_TRANSFERINTER2;
                    }
                    break;

                case CYCLE3_TRANSFERINTER2:
                    if(timer.seconds() > 0.5){
                        flip = transferflip;
                        timer.reset();
                        state = CYCLE3_TRANSFER1;
                    }
                    break;


                case CYCLE3_TRANSFER1:
                    if(timer.seconds() > 0.25){
                        Otarget = 25;
                    }
                    if(timer.seconds() > 0.75){
                        claw1pos = 0.9;
                        claw2pos = 0.1;
                        timer.reset();
                        state = CYCLE3_TRANSFER2;
                    }

                    break;

                case CYCLE3_TRANSFER2:
                    if(timer.seconds() > 0.25){
//                        latch.setPosition(0.3);
                        flip = intake2flip;
                        timer.reset();
                        state = CYCLE3_TRANSFER3;
                    }
                    break;

                case CYCLE3_TRANSFER3:
                    if(timer.seconds() > 0.5){
                        Otarget = 450;
                        flip = intake5flip;
                        flip1pos = 0.81;
                    }

                    if(timer.seconds() > 0.4){
                        panpos = 0.6;
                    }

                    if(!drive.isBusy() && timer.seconds() > 0.51){
                        timer.reset();
                        state = CYCLE3_DEPOSIT;
                    }

                    break;
                case CYCLE3_DEPOSIT:
                    if(timer.seconds() > 0){
                        claw1pos = 0.4;
                        claw2pos = 0.6;
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

//            outtakeMotor.setPower(Opower);


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