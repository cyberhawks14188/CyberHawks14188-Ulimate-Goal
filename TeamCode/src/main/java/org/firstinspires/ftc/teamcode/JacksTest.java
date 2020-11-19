package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
@TeleOp



public class JacksTest extends LinearOpMode {
    TestHardware robot  = new TestHardware();

    @Override


    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);
        double mPort0Speed = 0;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            mPort0Speed = (gamepad1.left_stick_y);
            if(gamepad1.a){
               robot.sPort0.setPosition(0);
            }else{
                robot.sPort0.setPosition(1);
            }
            robot.mPort0.setPower(mPort0Speed);
            telemetry.update();
        }
    }
}
