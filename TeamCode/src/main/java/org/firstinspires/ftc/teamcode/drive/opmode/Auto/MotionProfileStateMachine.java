package org.firstinspires.ftc.teamcode.drive.opmode.Auto;


import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.State.CYCLE1_INTAKE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.State.CYCLE1_INTER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.State.CYCLE1_INTER2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.State.CYCLE1_PRE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.State.PRELOAD_YELLOW;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.State.PRELOAD_YELLOW_DROP;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.State.PRELOAD_YELLOW_INTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.State.SPIT1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.State.SPIT2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.flip.betweenflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.flip.initializeflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.flip.intake1flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.flip.intake2flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.flip.intake3flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.flip.intake4flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.flip.intake5flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.MotionProfileStateMachine.flip.transferflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.clawdepositpos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.clawelevatorpos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.clawinitializepos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.clawouttakepos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.clawparkpos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.clawtransferpos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Teleop.MTITeleop;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="motion profiling fsm")
public class MotionProfileStateMachine extends LinearOpMode {
    OpenCvWebcam webcam;

    private MotionProfile profile;
    private double startTime;
    private DcMotorEx intakeMotor;
    public static double servotransferpos = 0.89;
    public static double servointake5pos = 0.395;
    public static double servointake4pos = 0.385;
    public static double servointake3pos = 0.37;
    public static double servointake2pos = 0.355;
    public static double servointake1pos = 0.34;
    public static double maxvel = 5;
    public static double maxaccel = 5;
    public static double maxvel2 = 11;
    public static double maxaccel2 = 11;
    private ElapsedTime timer = new ElapsedTime();

    private FtcDashboard dashboard = FtcDashboard.getInstance();

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
        CYCLE1_INTER2,
        SPIT1,
        SPIT2,
        CYCLE1_TRANSFER1,   // Retract intake and transfer pixels
        CYCLE1_TRANSFER2,   // Retract intake and transfer pixels
        CYCLE1_DEPOSIT,    // Drop pixels on backdrop
        CYCLE2_INTER_5,
        CYCLE2_INTER1,
        CYCLE2_INTAKE,
        CYCLE2_INTER2,
        CYCLE2_TRANSFER1,
        CYCLE2_TRANSFER2,
        CYCLE2_DEPOSIT,
        IDLE,              // Our bot will enter the IDLE state when done
    }

    enum flip{
        initializeflip,
        intake5flip,
        intake4flip,
        intake3flip,
        intake2flip,
        intake1flip,
        transferflip,
        betweenflip
    }

    flip flip = initializeflip;

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

    public static double pivotpos;
    public static double flip1pos = 0.25;
    public static double wristpos = 0.87;
    public static double claw1pos = 0.4;
    public static double claw2pos = 0.6;
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


        intSensor1 = hardwareMap.get(TouchSensor.class, "0t");
        intSensor2 = hardwareMap.get(TouchSensor.class, "1t");

        pivot1 = hardwareMap.get(Servo.class, "2s");
        pivot2 = hardwareMap.get(Servo.class, "3s");
        flip1 = hardwareMap.get(Servo.class, "4s");
        flip2 = hardwareMap.get(Servo.class, "5s");

        wrist = hardwareMap.get(Servo.class, "4ss");
        claw1 = hardwareMap.get(Servo.class, "0s");
        claw2 = hardwareMap.get(Servo.class, "1s");
        pan = hardwareMap.get(Servo.class, "2ss");

        intakeLeftExt.setDirection(DcMotorSimple.Direction.REVERSE);

        switchservo.setPosition(0.13);

        Lcontroller = new PIDController(Lp,Li,Ld);

        pivot1.setPosition(0.5);
        pivot2.setPosition(0.5);

        claw1.setPosition(0.4);
        claw2.setPosition(0.6);

        generateMotionProfile(pivot1.getPosition(), pivot1.getPosition(), 5, 5);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
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
//                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
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

        waitForStart();

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

        startTime = getRuntime();
        if (isStopRequested()) return;

        state = PRELOAD_YELLOW;
        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {

            double elapsedTime = getRuntime() - startTime;
            double servoPosition = profile.get(elapsedTime).getX();
            pivot1.setPosition(servoPosition);
            pivot2.setPosition(1-servoPosition);
            intakeMotor.setPower(intakePower);
            pan.setPosition(panpos);
            wrist.setPosition(wristpos);
            flip1.setPosition(flip1pos);
            flip2.setPosition(1-flip1pos);

            claw1.setPosition(claw1pos);
            claw2.setPosition(claw2pos);

            switch(state){
                case PRELOAD_YELLOW:
                    flip1pos = 0.26;
                    Ltarget = -50;
                    claw1pos = 0.4;
                    claw2pos = 0.6;
                    timer.reset();
                    state = PRELOAD_YELLOW_INTER;
                    break;

                case PRELOAD_YELLOW_INTER:
                    if(timer.seconds() > 0.25){
                        intakePower = -1;
                        flip = intake1flip;
                        timer.reset();
                        state = PRELOAD_YELLOW_DROP;
                    }
                    break;

                case PRELOAD_YELLOW_DROP:
                    intake1 = intSensor2.isPressed();
                    intake2 = intSensor1.isPressed();
                    if((intake1 && intake2)){
                        intake1 = false;
                        intake2 = false;
                        timer.reset();
                        state = CYCLE1_PRE;
                    }
                    break;

                case CYCLE1_PRE:
                    if(timer.seconds() > 0.1){
                        flip = transferflip;
                        timer.reset();
                        state = CYCLE1_INTAKE;
                    }
                    break;

                case CYCLE1_INTAKE:
                    if(pivot1.getPosition() > 0.85){
                        timer.reset();
                        state = CYCLE1_INTER1;
                    }
                    break;

                case CYCLE1_INTER1:
                    if(timer.seconds() > 0.2){
                        claw1pos = 0.9;
                        claw2pos = 0.1;
                        timer.reset();
                        state = CYCLE1_INTER2;
                    }
                    break;

                case CYCLE1_INTER2:
                    if(timer.seconds() > 0.2){
                        flip = intake1flip;
                        timer.reset();
                        state = SPIT1;
                    }
                    break;

                case SPIT1:
                    if(pivot1.getPosition() < 0.7 && timer.seconds() > 0.2){
                        flip1pos = 0.81;
                    }
                    if(timer.seconds() > 1.5){
                        claw1pos = 0.9;
                        claw2pos = 0.1;
                        timer.reset();
                        state = SPIT2;
                    }
                    break;

                case SPIT2:
                    if(timer.seconds() > 0.25){
                        flip1pos = 0.26;
                        timer.reset();
                        state = PRELOAD_YELLOW;
                    }
                    break;

            }

            Lcontroller.setPID(Lp, Li, Ld);

            int armPos = intakeLeftExt.getCurrentPosition();

            double Lpid = Lcontroller.calculate(armPos, Ltarget);

            double Lpower = Lpid;

            intakeLeftExt.setPower(Lpower);
            intakeRightExt.setPower(Lpower);


            drive.update();

            switch (flip) {
                case initializeflip:
                    startTime = getRuntime();
                    flip = betweenflip;
                    break;

                case transferflip:
                    generateMotionProfile(pivot1.getPosition(), servotransferpos, maxvel2, maxaccel2);
                    startTime = getRuntime();
                    flip = betweenflip;
                    break;

                case intake1flip:
                    generateMotionProfile(pivot1.getPosition(), servointake1pos, maxvel, maxaccel);
                    startTime = getRuntime();
                    flip = betweenflip;
                    break;

                case intake2flip:
                    generateMotionProfile(pivot1.getPosition(), servointake2pos, maxvel, maxaccel);
                    startTime = getRuntime();
                    flip = betweenflip;
                    break;

                case intake3flip:
                    generateMotionProfile(pivot1.getPosition(), servointake3pos, maxvel, maxaccel);
                    startTime = getRuntime();
                    flip = betweenflip;
                    break;

                case intake4flip:
                    generateMotionProfile(pivot1.getPosition(), servointake4pos, maxvel, maxaccel);
                    startTime = getRuntime();
                    flip = betweenflip;
                    break;

                case intake5flip:
                    generateMotionProfile(pivot1.getPosition(), servointake5pos, maxvel, maxaccel);
                    startTime = getRuntime();
                    flip = betweenflip;
                    break;

                case betweenflip:

                    break;
            }



        }
    }
    private void generateMotionProfile(double initialPosition, double finalPosition,
                                       double maxVelocity, double maxAcceleration) {
        // Generate the motion profile
        double maxJerk = 0;

        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(initialPosition, 0, 0),
                new MotionState(finalPosition, 0, 0),
                maxVelocity,
                maxAcceleration,
                maxJerk
        );

        // Reset the start time
        startTime = getRuntime();
    }
}