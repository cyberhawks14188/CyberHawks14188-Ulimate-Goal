package org.firstinspires.ftc.teamcode;
//Imports RobotCore

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

//Creates program Class
@Autonomous
//Sets name
public class Movement extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    Method_and_Varibles a = new Method_and_Varibles();
    //Declares Varibles
    //Enters the program method
    public void runOpMode() {
        robot.init(hardwareMap);
        a.robot.init(hardwareMap);

        //Resets Encoders
        robot.LF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LB_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.LB_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Waits for start to be pressed
        waitForStart();
        //Set position robot will go to
            a.Distance_From = 1000;
            a.distanceWithin = 100;
            a.startPosition = 0;
            a.targetVelocity = .5;
            //Runs movement until 100 away
        while (opModeIsActive()) {
                a.Movement(0, 10000, 0, 1000, 1000);
            }
            a.stop_motors();

        while(opModeIsActive()){
            a.telemetry();
        }
    }

}