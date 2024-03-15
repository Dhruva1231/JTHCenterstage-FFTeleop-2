package org.firstinspires.ftc.teamcode.drive.opmode.Teleop;

import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.Constants.clawinitializepos;

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

@Config
@TeleOp(name="Red Teleop")
public class NewTeleop3red extends OpMode {

    private MotionProfile profile;
    private double startTime;

    private PIDController Lcontroller;
    private PIDController Acontroller;

    public static double Lp = 0.006, Li = 0, Ld = 0.0001;
    public static double Lf = 0.01;//0-770
    public static int Ltarget = -50;//350 - grab

    private DcMotorEx intakeLeftExt;
    private DcMotorEx intakeRightExt;


    private DcMotorEx leftFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;
    private DcMotorEx intakeMotor;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double one = 0.15;

    //0.3
    //1.0
    public static double servotransferpos = 0.88;
    public static double servointakepos = 0.4;
    public static double maxvel = 5;
    public static double maxaccel = 5;

    public static double maxvel2 = 5;
    public static double maxaccel2 = 5;

    private Servo switchservo;
    private Servo pivot1;
    private Servo pivot2;
    private Servo flip1;
    private Servo flip2;
    private Servo pan;
    private Servo wrist;
    private Servo claw1;
    private Servo claw2;

    public static double flip1pos = 0.5;
    public static double wristpos = 0.87;
    public static double claw1pos = 0.9;
    public static double claw2pos = 0.1;
    public static double panpos = 0.48;

    public static double intakePower = 0;

    public DcMotorEx extensionencoder;

    public TouchSensor intSensor1;
    public TouchSensor intSensor2;


    @Override
    public void init(){
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "3");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "2");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "00");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "11");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "0");

        extensionencoder = hardwareMap.get(DcMotorEx.class, "2");

        intSensor1 = hardwareMap.get(TouchSensor.class, "0t");
        intSensor2 = hardwareMap.get(TouchSensor.class, "1t");

        intakeLeftExt = hardwareMap.get(DcMotorEx.class, "22");
        intakeRightExt = hardwareMap.get(DcMotorEx.class, "1");
        switchservo = hardwareMap.get(Servo.class, "switch");

        //claw 2 grab 0.1
        //claw 2 open 0.6
        //claw 2 is 1s

        //claw 1 grab 0.9
        //claw 1 open 0.4
        //claw 1 is 0s

        //4s didnt go to anything

        //5s goes to one deposit servo
        //3s goes to one of the intake
        //2s goes to othjer intake

        //2ss is pan
        //4ss is wrist

        //0ss goes to nothing

        //0.55 is transfer
        //0 is intake


        //0.24 is deposit tranfer


        //i think 0.47 was pan or wris

        //0.85 wrist, 0.51, 0.17, 0.27

        pivot1 = hardwareMap.get(Servo.class, "2s");
        pivot2 = hardwareMap.get(Servo.class, "3s");
        flip1 = hardwareMap.get(Servo.class, "4s");
        flip2 = hardwareMap.get(Servo.class, "5s");

        wrist = hardwareMap.get(Servo.class, "4ss");
        claw1 = hardwareMap.get(Servo.class, "0s");
        claw2 = hardwareMap.get(Servo.class, "1s");
        pan = hardwareMap.get(Servo.class, "2ss");

        intakeLeftExt.setDirection(DcMotorSimple.Direction.REVERSE);

        Lcontroller = new PIDController(Lp,Li,Ld);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        pivot1.setPosition(0.5);
        pivot2.setPosition(0.5);
        generateMotionProfile(pivot1.getPosition(), pivot1.getPosition(), 5, 5);

    }

    @Override
    public void start() {
        // Get the current time
        startTime = getRuntime();
    }

    @Override
    public void loop(){

        double elapsedTime = getRuntime() - startTime;
        double servoPosition = profile.get(elapsedTime).getX();
        pivot1.setPosition(servoPosition);
        pivot2.setPosition(1-servoPosition);


        double max;
        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;


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


        Lcontroller.setPID(Lp, Li, Ld);

        int armPos = intakeLeftExt.getCurrentPosition();

        double Lpid = Lcontroller.calculate(armPos, Ltarget);

        double Lpower = Lpid;

        intakeLeftExt.setPower(Lpower);
        intakeRightExt.setPower(Lpower);

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        intakeMotor.setPower(intakePower);

        pan.setPosition(panpos);
        wrist.setPosition(wristpos);


        flip1.setPosition(flip1pos);
        flip2.setPosition(1-flip1pos);

        claw1.setPosition(claw1pos);
        claw2.setPosition(claw2pos);


        if(gamepad1.cross){
            claw1pos = 0.9;
            claw2pos = 0.1;
        }

        if(gamepad1.circle){
            claw1pos = 0.4;
            claw2pos = 0.6;
        }

        if(gamepad1.triangle){
            generateMotionProfile(pivot1.getPosition(), servointakepos, maxvel, maxaccel);
        }

        if(gamepad1.square){
            generateMotionProfile(pivot1.getPosition(), servotransferpos, maxvel2, maxaccel2);
        }

        switchservo.setPosition(one);

        telemetry.addData("intake sensor 1", intSensor2.isPressed());
        telemetry.addData("intake sensor 2", intSensor1.isPressed());
        telemetry.addData("left", intakeLeftExt.getCurrentPosition());
        telemetry.addData("right", intakeRightExt.getCurrentPosition());
        telemetry.addData("position", extensionencoder.getCurrentPosition());
        telemetry.addData("position2", intakeMotor.getCurrentPosition());
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


