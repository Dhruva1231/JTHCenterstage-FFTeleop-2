package org.firstinspires.ftc.teamcode.drive.opmode.Teleop;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.pegging.servointake1pos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.pegging.servointake2pos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.pegging.servointake3pos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.pegging.servointake4pos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.pegging.servointake5pos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.betweenflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.initializeflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.intake1flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.intake2flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.intake3flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.intake5flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.intakeflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.transferflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.stackpos.bottom;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.stackpos.normal;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.stackpos.top;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.barrier;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.barrierinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.barrierpreinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.base;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.cancel;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.cancelinter1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.cancelinter2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.deposit;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.idle;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.initialization1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.initialization2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.initialize;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.intake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.intakeinter1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.outtake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.outtakeinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.outtakepre;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.pre;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.redo;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.retardintake1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.retardintake2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.retardintake3;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.retardintake4;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.shoot;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.transfer;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.transferinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.wristposotion.diagonalleft;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.wristposotion.diagonalright;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.wristposotion.horizontal;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.wristposotion.verticalleft;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.wristposotion.verticalright;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3red.servotransferpos;

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
@TeleOp(name="Blue Teleop")
public class NewTeleop3 extends OpMode {

    boolean wbool1 = false;
    boolean wbool2 = false;
    boolean wbool3 = false;
    boolean wbool4 = false;
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


    enum wristposotion{
        horizontal,
        verticalleft,
        diagonalleft,
        verticalright,
        diagonalright,
        inter
    }

    //horizonal = 0.53
    //vertical left = 0.17
    //diagonal left = 0.35
    //vertical right = 0.87
    //diagonal right = 0.7

    flip flip = initializeflip;
    wristposotion wristposition = horizontal;
    private MotionProfile profile;

    private double startTime;

    private IMU imu;
    private double slow = 1.0;
    private double turnslow = 1.0;
    private double progSlowmode = 1.0;


    public static double time1 = 1;
    public static double time2 = 0.2;
    public static double time3 = 0.2;
    public static double time4 = 0.6;

    private boolean left = false;
    private boolean right = false;
    boolean onOff = false;
    boolean onOff2 = false;
    boolean extendInt = false;
    boolean intaking = false;
    public DcMotorEx intakeLeftExt;
    public DcMotorEx outtakeMotor;
    public DcMotorEx intakeRightExt;
    private PIDController Lcontroller;
    private PIDController Ocontroller;

    public static double intakePower;

    //right claw grab = 0.5, retract = 0
    //left claw grab = 0.44, retract = 0.98
    public static double Lp = 0.006, Li = 0, Ld = 0.0001;

    public static double Lf;

    //Ltarget Max 750, Min -75
    public static int Ltarget;

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
        specialsensor,
        retardintake1,
        retardintake2,
        retardintake3,
        retardintake4,
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
    ElapsedTime wristtimer  = new ElapsedTime();
    ElapsedTime holdtimer  = new ElapsedTime();
    ElapsedTime holdtimer2  = new ElapsedTime();
    public static double claw1pos = 0.4;
    public static double claw2pos = 0.6;


    public static double flip1pos = 0.26;
    public static double servoPosition;
    public static double climbservoPosition = 0.55;
    public static double pposadd = -0.07;

    public Servo flip1;
    public Servo latch;
    public Servo flip2;
    public Servo pivot1;
    public Servo pivot2;
    public Servo climb;
    public Servo airplane;

    public Servo switchservo;
    public AnalogInput distance1;
    public static double Hp = 0.03;
    public static double one = 0.35;
    public static double two = 0.3;
    public static double plane = 0.5;

    public static double slowmode = 1;
    public static double strafeslowmode = 1;

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

        Lcontroller = new PIDController(Lp,Li,Ld);
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
        climb = hardwareMap.get(Servo.class, "2s");

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


        switchservo.setPosition(0.35);
        latch.setPosition(0.7);
        airplane.setPosition(0.5);


    }

    @Override
    public void start() {
        startTime = getRuntime();
        timer.reset();
        wristtimer.reset();
        // Get the current time
    }

    @Override
    public void loop(){

        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double angle = Math.toDegrees(currentHeading);

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

        latch.setPosition(two);
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        climb.setPosition(climbservoPosition);

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

        claw1.setPosition(claw2pos);
        claw2.setPosition(claw1pos);

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

            case cancel:
                if(Ltarget > 100){
                    intakePower = 0.2;
//                    intakeMotor.setPower(0.2);
                    timer.reset();
                    New = cancelinter1;
                }else{
//                    latch.setPosition(0.7);
                    Otarget = 10;
                    timer.reset();
                    New = base;
                }
                break;

            case cancelinter1:
                if(timer.seconds() > 0.25){
                    Ltarget = 50;
                    timer.reset();
                    New = cancelinter2;
                }
                break;

            case cancelinter2:
                if(timer.seconds() > 0.5){
                    Ltarget = -50;
                }
                if(timer.seconds() > 0.75){
                    Otarget = 30;
                    timer.reset();
                    New = base;
                }
                break;

            case pre:
                //move to intake
                flip1pos = 0.26;
                if(counter == 0){
                    flip = intakeflip;
                }else{
                    flip = intake1flip;
                }
                timer.reset();
                New = initialize;
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
                if(gamepad1.right_trigger > 0 && !onOff2 && holdtimer2.seconds() > 0.5){
//                    latch.setPosition(0.3);
                    holdtimer2.reset();
                    onOff2 = true;
                    extendInt = true;
                    Ltarget = 600;
                    intakePower = 0;
                }else if(gamepad1.right_trigger > 0 && onOff2 && holdtimer2.seconds() > 0.5){
//                    latch.setPosition(0.3);
                    holdtimer2.reset();
                    servoPosition = servointakepos;
//                    generateMotionProfile(pivot1.getPosition(), servointakepos, maxvel1, maxaccel1);
                    intakePower = 0;
                    onOff2 = false;
                    Ltarget = 0;
                }

                if(gamepad2.dpad_up){
                    servoPosition = servointake5pos;
                    intakePower = -1;
                }
                else if(gamepad2.dpad_right){
                    servoPosition = servointake4pos;
                    intakePower = -1;
                }
                else if(gamepad2.dpad_down){
                    servoPosition = servointake3pos;
                    intakePower = -1;
                }
                else if(gamepad2.dpad_left){
                    servoPosition = servointake2pos;
                    intakePower = -1;
                }

                int lstickpos1 = (int) (20 * gamepad2.right_stick_y);

                if(gamepad2.right_stick_y != 0){
                    Ltarget = Ltarget - lstickpos1;
                }

                if(((gamepad1.right_bumper||gamepad2.right_bumper) && !onOff && holdtimer.seconds() > 0.25 && (intakeLeftExt.getVelocity() < 50))||(extendInt && intakeLeftExt.getCurrentPosition() > 500)){
                    extendInt = false;
                    holdtimer.reset();
                    onOff = true;
                    //intake
                    servoPosition = servointakepos;
//                    generateMotionProfile(pivot1.getPosition(), servointakepos, maxvel1, maxaccel1);
                    intakePower = -1;
                }else if((gamepad1.right_bumper||gamepad2.right_bumper) && onOff && holdtimer.seconds() > 0.25 && !(intakeLeftExt.getVelocity() > 25)){
                    holdtimer.reset();
                    onOff = false;
                    intakePower = 0;
                    servoPosition = servointake1pos;
//                    generateMotionProfile(pivot1.getPosition(), servointake1pos, maxvel1, maxaccel1);
                }else if(gamepad1.left_bumper){
                    holdtimer.reset();
                    onOff = false;
                    intakePower = 1;
                }

                if((gamepad1.dpad_up)||(intSensor1.isPressed() && intSensor2.isPressed())){
                    intaking = false;
                    slow = 1;
                    turnslow = 1;
//                    intakeMotor.setPower(-0.5);
                    intakePower = -1;
                    timer.reset();
                    flip = intake3flip;

                    New = intakeinter1;
                }
                break;

            case retardintake1:
                if(timer.seconds() > 0.2){
                    intakePower = 0.4;
                    timer.reset();
                    New = retardintake2;
                }
                break;

            case retardintake2:
                if(timer.seconds() > 0.2){
                    intakePower = -1;
                    timer.reset();
                    New = retardintake3;
                }
                break;

            case retardintake3:
                if(timer.seconds() > 0.2){
                    intakePower = 0.4;
                    timer.reset();
                    New = retardintake4;
                }
                break;

            case retardintake4:
                if(timer.seconds() > 0.2){
                    intakePower = -1;
                    timer.reset();
                    New = intakeinter1;
                }
                break;

            case intakeinter1:
                if(timer.seconds() > 0.1){
                    Ltarget = -50;
                }
                if(timer.seconds() > time4){
//                    latch.setPosition(0.7);
                    timer.reset();
                    New = transfer;
                }
                break;

            case transfer:
                flip = transferflip;
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
                    flip = intake5flip;
                    timer.reset();
                    New = outtakeinter;
                }
                break;

            case outtakeinter:
                if(timer.seconds() > 0.49){
                    if(intSensor2.isPressed() && intSensor1.isPressed()){
                        timer.reset();
                        flip = intakeflip;
                        //fix claw pos
                        claw1pos = 0.4;
                        claw2pos = 0.6;
                        New = retardintake1;
                    }
                }
                if(timer.seconds()>0.5){
                    if(gamepad2.cross || gamepad1.cross){
                        if(timer.seconds() > 0.5){
                            Otarget = 200;
                            flip1pos = 0.81;
                            intakePower = 0;
                            timer.reset();
                            New = barrier;
                        }
                    }
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
                if(Otarget > 600 && gamepad2.cross){
                    plane = 0.12;
                }

                int lstickpos2 = (int) (15 * gamepad2.right_stick_y);

                if(gamepad2.right_stick_y != 0){
                    Otarget = Otarget - lstickpos2;
                }


                if(gamepad2.dpad_left){
                    strafeslowmode = 0.6;
                }else{
                    strafeslowmode = 1;
                }

                //horizonal = 0.53
                //vertical left = 0.17
                //diagonal left = 0.35
                //vertical right = 0.87
                //diagonal right = 0.7

                wbool1 = true;
//                if((gamepad1.right_trigger > 0 && gamepad1.left_trigger > 0)||(gamepad2.right_trigger > 0 && gamepad2.left_trigger > 0)){
//                    wristpos = 0.53;
//                }
//                else if((gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0)||(gamepad2.right_trigger > 0 && gamepad2.left_trigger == 0)){
//                    wristpos = 0.87;
//                }
//                else if((gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0)||(gamepad2.left_trigger > 0 && gamepad2.right_trigger == 0)){
//                    wristpos = 0.17;
//                }
//                panpos = Range.clip(-0.00603*(angle)+pposadd, 0.22, 0.72);
                if((gamepad1.left_bumper||gamepad2.left_bumper) && timer.seconds() > 0.5){
                    left = true;
                    claw1pos = 0.4;
                }
                if((gamepad1.right_bumper||gamepad2.right_bumper) && timer.seconds() > 0.5){
                    right = true;
                    claw2pos = 0.6;
                }
                if(left && right){
                    timer.reset();
                    New = deposit;
                }

                break;

            case deposit:
                wbool1 = false;
                panpos = 0.47;
                wristposition = horizontal;
                if(timer.seconds() > 0.5){
                    wbool1 = false;
                    left = false;
                    right = false;
                    c1pos = 0.1;
                    c2pos = 0.9;
                    wristpos = 0.87;
                    flip1pos = 0.35;
                }
                if(timer.seconds() > 0.75){
                    Otarget = 125;
                }
                if(timer.seconds() > 1){
                    Otarget = 50;
                }
                if(timer.seconds() > 1){
                    Otarget = -25;
                    timer.reset();
                    New = pre;
                }
                break;


            case idle:

                break;

            case specialsensor:

                break;
        }


        if(gamepad1.triangle){
            New = cancel;
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

        //horizonal = 0.53
        //vertical left = 0.17
        //diagonal left = 0.35
        //vertical right = 0.87
        //diagonal right = 0.7

        switch(wristposition){
            case horizontal:
                if(timer.seconds() > 0.2 && wbool1){
                    wristpos = 0.53;
                }
                if(gamepad2.left_trigger < 0.1 && gamepad2.right_trigger > 0 && wbool1 && wristtimer.seconds() > 0.15){
                    wristtimer.reset();
                    wristposition = diagonalright;
                }

                if(gamepad2.left_trigger > 0 && gamepad2.right_trigger < 0.1 && wbool1 && wristtimer.seconds() > 0.15){
                    wristtimer.reset();
                    wristposition = diagonalleft;
                }

                if(gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0 && wbool1 && wristtimer.seconds() > 0.15){
                    wristtimer.reset();
                    wristposition = horizontal;
                }
                break;

            case verticalleft:
                if(timer.seconds() > 0.2 && wbool1){
                    wristpos = 0.87;
                }
                if(gamepad2.left_trigger < 0.1 && gamepad2.right_trigger > 0 && wbool1 && wristtimer.seconds() > 0.15){
                    wristtimer.reset();
                    wristposition = diagonalleft;
                }

                if(gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0 && wbool1 && wristtimer.seconds() > 0.15){
                    wristtimer.reset();
                    wristposition = horizontal;
                }
                break;

            case verticalright:
                if(timer.seconds() > 0.2 && wbool1){
                    wristpos = 0.19;
                }
                if(gamepad2.left_trigger > 0 && gamepad2.right_trigger < 0.1 && wbool1 && wristtimer.seconds() > 0.15){
                    wristtimer.reset();
                    wristposition = diagonalright;
                }

                if(gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0 && wbool1 && wristtimer.seconds() > 0.15){
                    wristtimer.reset();
                    wristposition = horizontal;
                }
                break;

            case diagonalleft:
                if(timer.seconds() > 0.2 && wbool1){
                    wristpos = 0.75;
                }
                if(gamepad2.left_trigger > 0 && gamepad2.right_trigger < 0.1 && wbool1 && wristtimer.seconds() > 0.15){
                    wristtimer.reset();
                    wristposition = verticalleft;
                }

                if(gamepad2.left_trigger < 0.1 && gamepad2.right_trigger > 0 && wbool1 && wristtimer.seconds() > 0.15){
                    wristtimer.reset();
                    wristposition = horizontal;
                }

                if(gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0 && wbool1 && wristtimer.seconds() > 0.15){
                    wristtimer.reset();
                    wristposition = horizontal;
                }
                break;

            case diagonalright:
                if(timer.seconds() > 0.2 && wbool1){
                    wristpos = 0.31;
                }
                if(gamepad2.left_trigger < 0.1 && gamepad2.right_trigger > 0 && wbool1 && wristtimer.seconds() > 0.15){
                    wristtimer.reset();
                    wristposition = verticalright;
                }

                if(gamepad2.left_trigger > 0 && gamepad2.right_trigger < 0.1 && wbool1 && wristtimer.seconds() > 0.15){
                    wristtimer.reset();
                    wristposition = horizontal;
                }

                if(gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0 && wbool1 && wristtimer.seconds() > 0.15){
                    wristtimer.reset();
                    wristposition = horizontal;
                }
                break;

            case inter:

                break;
        }

        switch (flip) {
            case initializeflip:
                startTime = getRuntime();
                flip = betweenflip;
                break;

            case transferflip:
                servoPosition = servotransferpos;
//                generateMotionProfile(pivot1.getPosition(), servotransferpos, maxvel2, maxaccel2);
                startTime = getRuntime();
                flip = betweenflip;
                break;

            case intake1flip:
                servoPosition = servointake1pos;
//                generateMotionProfile(pivot1.getPosition(), servointake1pos, maxvel2, maxaccel2);
//                startTime = getRuntime();
                flip = betweenflip;
                break;

            case intake2flip:
                servoPosition = servointake2pos;
//                generateMotionProfile(pivot1.getPosition(), servointake2pos, maxvel2, maxaccel2);
//                startTime = getRuntime();
                flip = betweenflip;
                break;

            case intake3flip:
                servoPosition = servointake3pos;
//                generateMotionProfile(pivot1.getPosition(), servointake3pos, maxvel2, maxaccel2);
//                startTime = getRuntime();
                flip = betweenflip;
                break;

            case intake4flip:
                servoPosition = servointake4pos;
//                generateMotionProfile(pivot1.getPosition(), servointake4pos, maxvel2, maxaccel2);
//                startTime = getRuntime();
                flip = betweenflip;
                break;

            case intake5flip:
                servoPosition = servointake5pos;
//                generateMotionProfile(pivot1.getPosition(), servointake5pos, maxvel2, maxaccel2);
//                startTime = getRuntime();
                flip = betweenflip;
                break;

            case intakeflip:
                servoPosition = servointakepos;
//                generateMotionProfile(pivot1.getPosition(), servointakepos, maxvel2, maxaccel2);
//                startTime = getRuntime();
                flip = betweenflip;
                break;

            case betweenflip:

                break;
        }

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

