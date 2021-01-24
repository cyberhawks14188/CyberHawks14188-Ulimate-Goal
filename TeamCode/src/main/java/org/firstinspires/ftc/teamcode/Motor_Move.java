package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class Motor_Move extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    @Override
    public void runOpMode(){
        waitForStart();
        robot.init(hardwareMap);
        while(opModeIsActive()){
            robot.LF_M.setPower(.12);
            robot.LB_M.setPower(.12);
            robot.RF_M.setPower(.12);
            robot.RB_M.setPower(.12);
        }
    }

}