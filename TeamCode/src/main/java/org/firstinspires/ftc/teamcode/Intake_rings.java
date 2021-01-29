package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class Intake_rings extends LinearOpMode {

    public  void runOpMode(){
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            robot.IN_M.setPower(-1);
            robot.STG_M.setPower(1);
            robot.STOP_S.setPosition(.3);
        }
        robot.IN_M.setPower(0);
        robot.STG_M.setPower(0);
    }
}
