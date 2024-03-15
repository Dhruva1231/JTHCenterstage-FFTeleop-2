package org.firstinspires.ftc.teamcode.drive.opmode.Auto;


import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.SlidesMotionProfileStateMachine.State.CYCLE1_INTAKE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.SlidesMotionProfileStateMachine.State.CYCLE1_INTER1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.SlidesMotionProfileStateMachine.State.CYCLE1_PRE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.SlidesMotionProfileStateMachine.State.PRELOAD_YELLOW;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.SlidesMotionProfileStateMachine.State.PRELOAD_YELLOW_DROP;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.SlidesMotionProfileStateMachine.State.PRELOAD_YELLOW_INTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.SlidesMotionProfileStateMachine.State.SPIT1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.SlidesMotionProfileStateMachine.State.SPIT2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.SlidesMotionProfileStateMachine.flip.betweenflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.SlidesMotionProfileStateMachine.flip.initializeflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.SlidesMotionProfileStateMachine.flip.intake1flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.SlidesMotionProfileStateMachine.flip.transferflip;

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
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="slides motion profiling fsm")
public class SlidesMotionProfileStateMachine extends LinearOpMode {
    OpenCvWebcam webcam;

    private MotionProfile profile;
    private double startTime;
    private DcMotorEx intakeMotor;
    public static double maxvel = 100;
    public static double maxaccel = 100;
    public static double maxvel2 = 100;
    public static double maxaccel2 = 100;
    private ElapsedTime timer = new ElapsedTime();

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

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
        CYCLE1_INTER2,
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

    public static double extendotransferpos = -50;
    public static double extendointake = 700;
    public static double servointake4pos = 0.43;
    public static double servointake3pos = 0.39;
    public static double servointake2pos = 0.37;
    public static double servointake1pos = 0.35;

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

        generateMotionProfile(intakeLeftExt.getCurrentPosition(), intakeLeftExt.getCurrentPosition(), 5, 5);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        startTime = getRuntime();
        if (isStopRequested()) return;

        state = PRELOAD_YELLOW;
        timer.reset();

        while (opModeIsActive() && !isStopRequested()) {

            double elapsedTime = getRuntime() - startTime;
            Ltarget = (int) profile.get(elapsedTime).getX();

            pivot1.setPosition(0.5);
            pivot2.setPosition(0.5);
            intakeMotor.setPower(intakePower);
            pan.setPosition(panpos);
            wrist.setPosition(wristpos);
            flip1.setPosition(flip1pos);
            flip2.setPosition(1-flip1pos);

            claw1.setPosition(claw1pos);
            claw2.setPosition(claw2pos);

//            pivot1.setPosition(servoPosition);
//            pivot2.setPosition(1-servoPosition);

            switch(state){
                case PRELOAD_YELLOW:
                    flip = intake1flip;
                    timer.reset();
                    state = PRELOAD_YELLOW_INTER;
                    break;

                case PRELOAD_YELLOW_INTER:
                    if(timer.seconds() > 5){
                        flip = transferflip;
                        timer.reset();
                        state = PRELOAD_YELLOW_DROP;
                    }
                    break;

                case PRELOAD_YELLOW_DROP:
                    if(timer.seconds() > 5){
                        flip = intake1flip;
                        timer.reset();
                        state = CYCLE1_PRE;
                    }
                    break;

                case CYCLE1_PRE:

                    break;

                case CYCLE1_INTAKE:
                    break;

                case CYCLE1_INTER1:
                    break;

                case SPIT1:

                    break;

                case SPIT2:
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
                    generateMotionProfile(Ltarget, extendotransferpos, maxvel2, maxaccel2);
                    startTime = getRuntime();
                    flip = betweenflip;
                    break;

                case intake1flip:
                    generateMotionProfile(Ltarget, extendointake, maxvel, maxaccel);
                    startTime = getRuntime();
                    flip = betweenflip;
                    break;

                case betweenflip:

                    break;
            }

            Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());


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