package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class Test extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){

        }
}
}
