package org.firstinspires.ftc.teamcode.drive.opmode.Teleop;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.NewStatesAuto.servoPosition;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.NewStatesAuto.servointake1pos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.NewStatesAuto.servointake2pos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.NewStatesAuto.servointake3pos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.NewStatesAuto.servointake4pos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.NewStatesAuto.servointake5pos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.betweenflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.initializeflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.intake1flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.intake5flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.intakeflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.transferflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.stackpos.normal;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.barrier;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.barrierinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.barrierpreinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.base;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.cancel;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.cancelinter1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.cancelinter2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.deposit;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.initialization1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.initialization2;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.initialize;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.intake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.intakeinter1;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.outtake;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.outtakeinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.outtakepre;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.pre;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.transfer;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.transferinter;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3red.servotransferpos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name="outreach bot")
public class outreachbot extends OpMode {

    private DcMotorEx leftFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightBackDrive;

    public Servo pivot1;
    public Servo pivot2;


    @Override
    public void init(){


        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "0");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "1");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "2");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "3");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivot1 = hardwareMap.get(Servo.class, "00");
        pivot2 = hardwareMap.get(Servo.class, "11");


        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void start() {

        // Get the current time
    }

    @Override
    public void loop(){
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


        if(gamepad1.cross){
            pivot1.setPosition(1);
        }else{
            pivot1.setPosition(0.5);
        }

        if(gamepad1.triangle){
            pivot2.setPosition(0.5);
        }else{
            pivot2.setPosition(1);
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        }

}

