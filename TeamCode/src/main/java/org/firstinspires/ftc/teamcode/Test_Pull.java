package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
public class Test_Pull extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    @Override
    public void runOpMode(){
    robot.init(hardwareMap);
    }
    public void tele(){
        telemetry.addData("E1", robot.LF_M.getCurrentPosition());
    }
}
