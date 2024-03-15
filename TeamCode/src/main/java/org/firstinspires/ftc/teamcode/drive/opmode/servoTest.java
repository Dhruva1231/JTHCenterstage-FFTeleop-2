package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="servoTest")
public class servoTest extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public Servo flip1;

    @Override
    public void runOpMode() {
        flip1 = hardwareMap.get(Servo.class, "4s");
        waitForStart();
        while (opModeIsActive()) {
            flip1.setPosition(0);
        }
    }
}

