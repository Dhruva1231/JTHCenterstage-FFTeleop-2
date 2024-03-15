package org.firstinspires.ftc.teamcode.drive.opmode.Teleop;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.NewStatesAuto.servointake1pos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.NewStatesAuto.servointake2pos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.NewStatesAuto.servointake3pos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.NewStatesAuto.servointake4pos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Auto.NewStatesAuto.servointake5pos;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.betweenflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.initializeflip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.intake1flip;
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.flip.intake2flip;
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
import static org.firstinspires.ftc.teamcode.drive.opmode.Teleop.NewTeleop3.state.shoot;
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
@TeleOp(name="ExpansionHubTest")
public class TestProgram extends OpMode {

    public Servo port0;
    public Servo port1;
    public Servo port2;
    public Servo port3;
    public Servo port4;
    public Servo port5;


    private int counter = 0;

    private DcMotorEx motor0;
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx motor3;
    private FtcDashboard dashboard = FtcDashboard.getInstance();


    ElapsedTime timer  = new ElapsedTime();


    @Override
    public void init(){

        // Retrieve the IMU from the hardware map

        motor0 = hardwareMap.get(DcMotorEx.class, "0");
        motor1 = hardwareMap.get(DcMotorEx.class, "1");
        motor2 = hardwareMap.get(DcMotorEx.class, "2");
        motor3 = hardwareMap.get(DcMotorEx.class, "3");

        port0 = hardwareMap.get(Servo.class, "00");
        port1 = hardwareMap.get(Servo.class, "11");
        port2 = hardwareMap.get(Servo.class, "22");
        port3 = hardwareMap.get(Servo.class, "33");
        port4 = hardwareMap.get(Servo.class, "44");
        port5 = hardwareMap.get(Servo.class, "55");

    }

    @Override
    public void start() {
        timer.reset();
        // Get the current time
    }

    @Override
    public void loop(){

        port0.setPosition(0);
        port1.setPosition(1);
        port2.setPosition(0);
        port3.setPosition(1);
        port4.setPosition(0);
        port5.setPosition(1);

        motor0.setPower(0.5);
        motor1.setPower(0.5);
        motor2.setPower(0.5);
        motor3.setPower(0.5);

        telemetry.addData("port0", motor0.getCurrentPosition());
        telemetry.addData("port1", motor1.getCurrentPosition());
        telemetry.addData("port2", motor2.getCurrentPosition());
        telemetry.addData("port3", motor3.getCurrentPosition());

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        telemetry.update();
    }


}

