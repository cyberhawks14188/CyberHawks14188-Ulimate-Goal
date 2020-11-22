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

    //Declares Varibles
    double x;
    double y;
    double z;
    double z_error;
    double x_error;
    double z_porportional;
    double x_porportional;
    double y_porportional;
    double Z_Intergral;
    double  X_Intergral;
    double Y_Intergral;
    double Z_Sum_of_Errors;
    double X_Sum_of_Errors;
    double Y_Sum_of_Errors;
    double Z_Last_Error;
    double X_Last_Error;
    double Y_Last_Error;
    double Z_Diffrence_of_Errors;
    double X_Diffrence_of_Errors;
    double Y_Diffrence_of_Errors;
    double Z_Derivitive;
    double X_Derivitive;
    double Y_Derivitive;
    double z_encoder_diffrence;
    double Distance;
    double y_error;
    double y_slope;
    double x_slope;
    double slope;
    int last_y_setpoint;
    double Z_PM;
    double Z_IM;
    double Z_DM;
    double pass;
    double X_PM;
    double X_IM;
    double X_DM;
    double Y_PM;
    double Y_IM;
    double Y_DM;
    int last_x_setpoint;
    double Z_setpoint;
    double Y_Distiance_From_Setpoint;
    double X_setpoint;
    double Y_setpoint;
    double LF_Distance;
    double LB_Distance;
    int E1;
    int E2;
    int E3;
    double RF_Distance;
    double RB_Distance;
    double Speed_Setpoint;
    double Highest_Motor_Power;
    double Distance_From ;
    int Y_Average;
    double Y_A2;
    int Slow_Down_Distance;
    double Slow_Rate;

    double X_B2;
    int breakout;
    double runtime;
    double Detected;

    double Intial_Speed_Setpoint;
    //Enters the program method
    public void runOpMode() {
        robot.init(hardwareMap);
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
            Distance_From = 1000;
            Speed_Setpoint = .5;
            X_setpoint = 0;
            Y_setpoint = 3000;
            Z_setpoint = 0;
            Slow_Down_Distance = 1000;
            //Runs movement until 300 away
            while (Distance_From >= 300) {
                Movement();
            }
            stop_motors();

        while(opModeIsActive()){
            telemetry();
        }
    }
    //Stops all drive motors
    public void stop_motors(){
        robot.LF_M.setPower(0);
        robot.LB_M.setPower(0);
        robot.RF_M.setPower(0);
        robot.RB_M.setPower(0);
    }
    //Displays useful telementry onto DS phone
    public void telemetry(){
        telemetry.addData("E1", E1);
        telemetry.addData("E2", E2);
        telemetry.addData("E3", E3);
        telemetry.addData("Distance_From", Distance_From);
        telemetry.addData("Distance", Distance);
        telemetry.addData("Speed_Setpoint", Speed_Setpoint);
        telemetry.addData("Y", y);
        telemetry.addData("X", x);
        telemetry.addData("Y_Setpoint", Y_setpoint);
        telemetry.addData("X_Setpoint", X_setpoint);
        telemetry.addData("LF", Speed_Setpoint*(LF_Distance/Highest_Motor_Power));
        telemetry.addData("LB", Speed_Setpoint*(LB_Distance/Highest_Motor_Power));
        telemetry.addData("RF", Speed_Setpoint*(RF_Distance/Highest_Motor_Power));
        telemetry.addData("RB", Speed_Setpoint*(RB_Distance/Highest_Motor_Power));
        telemetry.addData("Dectected", Detected);
        telemetry.update();
    }
    //Uses a PID to move robot to XYZ setpoints
    public void Movement() {
        //Sets Multipliers
        X_PM = 1;
        X_IM = 0;
        X_DM = 0;
        Y_PM = 1;
        Y_IM = 0;
        Y_DM = 0;
        Z_PM = 1;
        Z_IM = 0;
        Z_DM = 0;

        //Gets encoder Positions
         E1 = -robot.LF_M.getCurrentPosition();
         E2 = robot.LB_M.getCurrentPosition();
         E3 = robot.RF_M.getCurrentPosition();
        //Sets encoders to 1 at begining to prevent the robot from not moving due to the values being 0
        if (E1 == 0){
            E1 = 1;
        }
        if(E2 == 0){
            E2 = 1;
        }
        if(E3 == 0){
            E3 = 1;
        }
        //finds the diffrence of E1 and E3
        z_encoder_diffrence = E1-E3;
        //Z Porportional
        z_error =  Z_setpoint - z_encoder_diffrence;
        z_porportional = Z_PM*z_error;
        // Z Intergral
        //Finds the Sum of the Error
        Z_Sum_of_Errors = Z_Sum_of_Errors + z_error;
        Z_Intergral = Z_Sum_of_Errors * Z_IM;
        //Z Derivitive
        Z_Diffrence_of_Errors = z_error - Z_Last_Error;
        Z_Derivitive = Z_Diffrence_of_Errors * Z_DM;
        Z_Last_Error = z_error;
        z = z_porportional + Z_Intergral + Z_Derivitive;
        //Finds the X error
        x_error = X_setpoint - E2;
        //X Porportional
        x_porportional = x_error * X_PM;
        //X Intergral
        X_Sum_of_Errors = X_Sum_of_Errors + x_error;
        X_Intergral = X_Sum_of_Errors * X_IM;
        //X Derivitve
        X_Diffrence_of_Errors = x_error - X_Last_Error;
        X_Derivitive = X_Diffrence_of_Errors * X_DM;
        X_Last_Error = x_error;
        x = x_porportional + X_Intergral + X_Derivitive;
        //Finds Average Of E1&E3
        Y_Average = (E1+E3)/2;
        //finds the average of E1 and E3
        y_error = Y_setpoint - Y_Average;
        //Y Porportional
        y_porportional = Y_PM*y_error;
        // Y Intergral
        //Finds the Sum of the Error
        Y_Sum_of_Errors = Y_Sum_of_Errors + y_error;
        Y_Intergral = Y_Sum_of_Errors * Y_IM;
        //Y Derivitive
        Y_Diffrence_of_Errors = y_error - Y_Last_Error;
        Y_Derivitive = Y_Diffrence_of_Errors * Y_DM;
        Y_Last_Error = y_error;
        y = y_porportional + Y_Intergral + Y_Derivitive;
        //slope equation
       /* if(X_Final_Setpoint > 0 & Y_Final_Setpoint > 0){
           slope = Y_Final_Setpoint/X_Final_Setpoint;
        Y_setpoint = slope * E2;
        X_setpoint = (((E1+E3)/2)/slope);
        if(last_y_setpoint > 0){
            Y_setpoint = Y_setpoint + last_y_setpoint;
        }
        if(last_x_setpoint > 0){
            X_setpoint = X_setpoint + last_x_setpoint;
        }
           bypass = 1;
        }
        if(Y_setpoint >= Y_Final_Setpoint) {
            Y_setpoint = Y_Final_Setpoint;
        }
        if(X_setpoint >= X_Final_Setpoint){
            X_setpoint = X_Final_Setpoint;
         }
*/
        //Uses pythagrium therom to find distance and distance from
        Distance = Math.sqrt(Math.pow(Y_setpoint, 2) + (Math.pow(X_setpoint, 2)));
        Y_A2 = Y_setpoint - Y_Average;
        X_B2 = X_setpoint - E2;
        Distance_From = Math.sqrt(Math.pow(Y_A2, 2) + (Math.pow(X_B2, 2)));
        //If we are below our slow down distance begin our slow down
        if (Distance_From <= Slow_Down_Distance) {
            Slow_Rate = (Distance_From/Slow_Down_Distance);
            Speed_Setpoint = Slow_Rate*Speed_Setpoint;
            //Prevents robot from going to slow during deacceleration
        }
        //robot is stoped before reaching setpoint increse speed
        if (Speed_Setpoint <=.265){
            Speed_Setpoint = .265;
        }
        MotorEquation();
        telemetry();
    }
    public void MotorEquation() {
        //Make Equation
        LF_Distance = y+(x+z);
        LB_Distance = y-(x-z);
        RF_Distance = y-(x+z);
        RB_Distance = y+(x-z);
        //Finds Highest power out of the drive motors
        Highest_Motor_Power = Math.max(Math.max(Math.abs(RF_Distance), Math.abs(RB_Distance)), Math.max(Math.abs(LF_Distance), Math.abs(LB_Distance)));
        //Sets motors
        robot.LF_M.setPower((Speed_Setpoint*(LF_Distance/Highest_Motor_Power)));
        robot.LB_M.setPower((Speed_Setpoint*(LB_Distance/Highest_Motor_Power)));
        robot.RF_M.setPower((Speed_Setpoint*(RF_Distance/Highest_Motor_Power)));
        robot.RB_M.setPower((Speed_Setpoint*(RB_Distance/Highest_Motor_Power)));
    }
    //Runs Encoders

}