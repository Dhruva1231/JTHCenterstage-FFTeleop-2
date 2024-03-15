package org.firstinspires.ftc.teamcode.drive.opmode.Teleop;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.NewStatesAuto.servointake5pos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3red.servotransferpos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.flip.betweenflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.flip.initializeflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.flip.intake1flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.flip.intake2flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.flip.intake5flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.flip.intakeflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.flip.transferflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.stackpos.bottom;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.stackpos.normal;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.stackpos.top;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.barrier;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.barrierinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.barrierpreinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.base;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.cancel;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.cancelinter1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.cancelinter2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.deposit;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.idle;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.initialization1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.initialization2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.initialize;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.intake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.intakeinter1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.outtake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.outtakeinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.outtakepre;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.pre;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.redo;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.shoot;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.transfer;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.TransferTest.state.transferinter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.commands.core.LynxGetMotorTargetVelocityCommand;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.Auto.NewStatesAuto;

@Config
@TeleOp(name="Transfer Test")
public class TransferTest extends OpMode {
    public static double servointakepos = 0.32;
    enum flip{
        initializeflip,
        intake5flip,
        intake4flip,
        intake3flip,
        intake2flip,
        intake1flip,
        transferflip,
        intakeflip,
        betweenflip
    }

    flip flip = initializeflip;

    private MotionProfile profile;

    private double startTime;

    private IMU imu;
    private double slow = 1.0;
    private double turnslow = 1.0;
    private double progSlowmode = 1.0;

    private boolean left = false;
    private boolean right = false;
    boolean onOff = false;
    boolean onOff2 = false;
    boolean extendInt = false;
    boolean intaking = false;
    public DcMotorEx intakeLeftExt;
    public DcMotorEx outtakeMotor;
    public DcMotorEx intakeRightExt;
    private PIDController Ocontroller;

    public static double intakePower;

    //right claw grab = 0.5, retract = 0
    //left claw grab = 0.44, retract = 0.98
    //Otarget Max 800, Min 25
    public static int Otarget = -25;
    public static double Op = 0.012, Oi = 0, Od = 0.0002;
    public static double Of = -0.08;

    public DcMotorEx intakeMotor;

    public static double wristpos = 0.87;
    public static double panpos = 0.47;
    public static double c1pos = 0;
    public static double c2pos = 0.98;

    public Servo pivotleft;
    public Servo pivotright;
    public Servo elbowleft;
    public Servo elbowright;
    public Servo pan;
    public Servo tilt;
    public Servo wrist;
    public Servo claw1;
    public Servo claw2;

    public static double maxvel1 = 15;
    public static double maxaccel1 = 15;

    public static double maxvel2 = 30;
    public static double maxaccel2 = 30;


    private int counter = 0;

    private DcMotorEx leftFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private boolean posLockOut = false;
    private boolean posLockInt = false;

    public enum stackpos{
        top,
        bottom,
        normal
    }
    public enum state {
        initialization1,
        initialization2,
        cancel,
        cancelinter1,
        cancelinter2,
        pre,
        initialize,
        intslides,
        base,
        intake,
        intakeinter1,
        transfer,
        transferinter,
        outtake,
        outtakepre,
        outtakeinter,
        barrier,
        barrierpreinter,
        barrierinter,
        redo,
        deposit,
        shoot,
        idle,
        specialsensor
    }

    public TouchSensor intSensor1;
    public TouchSensor funnelsensor;
    public TouchSensor intSensor2;

    public static double finalconvert;

    public double targetAngleOut = Math.toRadians(90);
    public double targetAngleInt = Math.toRadians(90);

    state New = initialization1;
    stackpos stackposition = normal;
    ElapsedTime timer  = new ElapsedTime();
    ElapsedTime holdtimer  = new ElapsedTime();
    ElapsedTime holdtimer2  = new ElapsedTime();
    public static double claw1pos = 0.4;
    public static double claw2pos = 0.6;


    public static double flip1pos = 0.26;
    public static double servoPosition;

    public Servo flip1;

    public static double time1 = 1;
    public static double time2 = 0.2;
    public static double time3 = 0.2;
    public static double time4 = 0.6;
    public Servo latch;
    public Servo flip2;
    public Servo pivot1;
    public Servo pivot2;
    public Servo airplane;

    public Servo switchservo;
    public AnalogInput distance1;
    public static double Hp = 0.03;
    public static double one = 0.12;
    public static double two = 0.3;
    public static double plane = 0.5;

    public static double slowmode;
    public static double strafeslowmode;

    @Override
    public void init(){

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        Ocontroller = new PIDController(Op,Oi,Od);

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
        airplane = hardwareMap.get(Servo.class, "3ss");

        wrist = hardwareMap.get(Servo.class, "4ss");
        claw1 = hardwareMap.get(Servo.class, "0s");
        claw2 = hardwareMap.get(Servo.class, "1s");
        pan = hardwareMap.get(Servo.class, "2ss");

        intakeLeftExt.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        latch = hardwareMap.get(Servo.class, "1ss");

        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "3");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "2");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "00");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "11");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        switchservo.setPosition(0.12);
        latch.setPosition(0.7);
        airplane.setPosition(0.5);
//        generateMotionProfile(pivot1.getPosition(), pivot1.getPosition(), 15, 15);


    }

    @Override
    public void start() {
        startTime = getRuntime();
        timer.reset();
        // Get the current time
    }

    @Override
    public void loop(){
        double max;
        double axial   = -gamepad1.left_stick_y * slowmode;
        double lateral =  gamepad1.left_stick_x * slowmode * strafeslowmode;
        double yaw     =  gamepad1.right_stick_x * slowmode;


        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        switchservo.setPosition(one);

//        latch.setPosition(two);
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

//        double elapsedTime = getRuntime() - startTime;
//        double servoPosition = profile.get(elapsedTime).getX();
        pivot1.setPosition(servoPosition);
        pivot2.setPosition(1-servoPosition);

        airplane.setPosition(plane);
        intakeMotor.setPower(intakePower);
        pan.setPosition(panpos);
        wrist.setPosition(wristpos);
        flip1.setPosition(flip1pos);
        flip2.setPosition(1-flip1pos);

        claw1.setPosition(claw1pos);
        claw2.setPosition(claw2pos);

        switch (New) {
            case initialization1:
                Otarget = -25;
                wristpos = 0.87;
                flip1pos = 0.26;
                timer.reset();
                New = initialization2;
                break;

            case initialization2:
                timer.reset();
                New = pre;
                break;

            case pre:
                //move to intake
                if(timer.seconds() > 0.25){
                    flip1pos = 0.26;
                    if(counter == 0){
                        flip = intakeflip;
                    }else{
                        flip = intake1flip;
                    }
                    timer.reset();
                    New = initialize;
                }
                break;

            case initialize:
                timer.reset();
                New = base;
                break;

            case base:
                timer.reset();
                New = intake;
                break;

            case intake:
                servoPosition = servointakepos;
                intakePower = -1;

                if((gamepad1.dpad_up)||(intSensor1.isPressed() && intSensor2.isPressed())){
                    intaking = false;
                    slow = 1;
                    turnslow = 1;
//                    intakeMotor.setPower(-0.5);
                    intakePower = -0.5;
                    timer.reset();
                    New = intakeinter1;
                }
                break;

            case intakeinter1:
                if(timer.seconds() > time4){
//                    latch.setPosition(0.7);
                    timer.reset();
                    New = transfer;
                }
                break;

            case transfer:
                servoPosition = servotransferpos;
                timer.reset();
                New = transferinter;
                break;

            case transferinter:
                if(timer.seconds() > time1){
                    timer.reset();
                    New = outtake;
                }
                break;

            case outtake:
                if(timer.seconds() > 0){
                    Otarget = 35;
                    claw1pos = 0.9;
                    claw2pos = 0.1;
                    timer.reset();
                    New = outtakepre;
                }
                break;

            case outtakepre:
                if(timer.seconds() > time2){
                    servoPosition = servointake5pos;
                    timer.reset();
                    New = outtakeinter;
                }
                break;

            case outtakeinter:
                if(timer.seconds() > time3){
                    Otarget = 200;
                    flip1pos = 0.81;
                    intakePower = 0;
                    timer.reset();
                    New = barrier;
                }
                break;

            case barrier:
                if(timer.seconds() > 0.15){
                    wristpos = 0.53;
                    timer.reset();
                    New = barrierpreinter;
                }
                break;

            case barrierpreinter:
                timer.reset();
                New = barrierinter;
                break;

            case barrierinter:
                if(timer.seconds() > 0.5){
                claw1pos = 0.4;
                claw2pos = 0.6;
                timer.reset();
                New = deposit;
                }
                break;

            case deposit:
                c1pos = 0.1;
                c2pos = 0.9;
                wristpos = 0.87;
                flip1pos = 0.35;
                Otarget = -25;
                timer.reset();
                New = pre;
                break;


            case idle:

                break;

            case specialsensor:

                break;
        }


        if(gamepad1.triangle){
            New = cancel;
        }


        Ocontroller.setPID(Op, Oi, Od);

        int outPos = outtakeMotor.getCurrentPosition();

        double Opid = Ocontroller.calculate(outPos, -Otarget);
        double Off = Of;


        double Opower = Opid + Off;

        outtakeMotor.setPower(Opower);


        telemetry.addData("outtake power", Opower);


        telemetry.addData("testinginging", intakeLeftExt.getCurrentPosition());
        telemetry.addData("axon pos", intakeMotor.getCurrentPosition());


        telemetry.addData("left", intakeLeftExt.getCurrentPosition());
        telemetry.addData("right", intakeRightExt.getCurrentPosition());
        telemetry.addData("out", outtakeMotor.getCurrentPosition());
        telemetry.addData("intake 1 ", intSensor1.isPressed());
        telemetry.addData("intake 2", intSensor2.isPressed());

        telemetry.addData("pitch", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS));

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        telemetry.update();
    }

    public double modify(double heading){
        //if it is negative
        if(heading < 0){
            heading += 360;
        }
        return heading % 360;

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

