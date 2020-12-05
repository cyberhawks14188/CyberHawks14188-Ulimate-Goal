package org.firstinspires.ftc.teamcode;
//Imports RobotCore

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
 class Method_and_Varibles extends LinearOpMode {
     RobotHardware robot = new RobotHardware();
     public double x;
     public double y;
     public double z;
     public double z_error;
     public double x_error;
     public double z_porportional;
     public double x_porportional;
     public double previousTime;
     public double y_porportional;
     public double Z_Intergral;
     public double  X_Intergral;
     public double Y_Intergral;
     public double Z_Sum_of_Errors;
     public double lastDistanceFrom;
     public double X_Sum_of_Errors;
     public double targetVelocity;
     public double Y_Sum_of_Errors;
     public double Z_Last_Error;
     public double X_Last_Error;
     public double Y_Last_Error;
     public double Z_Diffrence_of_Errors;
     public double X_Diffrence_of_Errors;
     public double velocityDiffrenceOfErrors;
     public double velocityLastError;
     public double Y_Diffrence_of_Errors;
     public double VIM;
     public double velocityIntergral;
     public double Z_Derivitive;
     public double X_Derivitive;
     public double Y_Derivitive;
     public int moving;
     public double z_encoder_diffrence;
     public double Distance;
     public double y_error;
     public double y_slope;
     public double x_slope;
     public double slope;
     public int last_y_setpoint;
     public double Z_PM;
     public double Z_IM;
     public double Z_DM;
     public double pass;
     public double X_PM;
     public double X_IM;
     public double X_DM;
     public double Y_PM;
    public double Y_IM;
    public double Y_DM;
    public int last_x_setpoint;
    public double Z_setpoint;
    public double Y_Distiance_From_Setpoint;
    public double X_setpoint;
    public double VPM;
    public double velocityPorportion;
    public double expectedSpeedSetpoint;
    public double Y_setpoint;
    public double LF_Distance;
    public double velocityError;
    public double LB_Distance;
    public double distanceWithin;
    public double actualVelocity;
    public double velocityCorrection;
    public double velocitySetpoint;
    public int E1;
    public int E2;
    public int E3;
    public double RF_Distance;
    public double RB_Distance;
    public double Speed_Setpoint;
    public double Highest_Motor_Power;
    public double Distance_From ;
    public int Y_Average;
    public double startPosition;
    public double Y_A2;
    public double velocitySumOfErrors;
    public int loopcount;
    public int Slow_Down_Distance;
    public double veloictyDerivitive;

    public double VDM;
    public double Slow_Rate;
    public double X_B2;
    public int breakout;
    public double runtime;
    public double Detected;

    public void runOpMode(){

    }

    public double Intial_Speed_Setpoint;
     public void telemetry(){
         telemetry.addData("Time", time);
         telemetry.addData("Velocity", actualVelocity);
         telemetry.addData("E1", E1);
         telemetry.addData("E2", E2);
         telemetry.addData("E3", E3);
         telemetry.addData("Distance_From", Distance_From);
         telemetry.addData("Distance", Distance);
         telemetry.addData("Speed_Setpoint", Speed_Setpoint);
         telemetry.addData("Y", y);
         telemetry.addData("X", x);
         telemetry.addData("Velocity", actualVelocity);
         telemetry.addData("Y_Setpoint", Y_setpoint);
         telemetry.addData("X_Setpoint", X_setpoint);
         telemetry.addData("LF", Speed_Setpoint*(LF_Distance/Highest_Motor_Power));
         telemetry.addData("LB", Speed_Setpoint*(LB_Distance/Highest_Motor_Power));
         telemetry.addData("RF", Speed_Setpoint*(RF_Distance/Highest_Motor_Power));
         telemetry.addData("RB", Speed_Setpoint*(RB_Distance/Highest_Motor_Power));
         telemetry.addData("Dectected", Detected);
         telemetry.update();
     }
     public void Movement(double X_setpoint, double Y_setpoint, double Z_setpoint, double Slow_Down_Distance, double accelerationDistance) {
         robot.init(hardwareMap);
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
         VPM = 0;
         VIM = 0;
         VDM = 0;

         //Gets encoder Positions
         E1 = -robot.LF_M.getCurrentPosition();
         E2 = robot.LB_M.getCurrentPosition();
         E3 = robot.RF_M.getCurrentPosition();
         //Sets encoders to 1 at begining to prevent null error
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
         loopcount = loopcount + 1;
         time = getRuntime();
         if(time >= previousTime){
             //Velocity is number of ticks per second
             actualVelocity = (Distance_From - lastDistanceFrom)/720;
             previousTime = time + .2;
             lastDistanceFrom = Distance_From;
         }
         velocityError = velocitySetpoint - actualVelocity;
         velocityPorportion = velocityError * VPM;
         velocitySumOfErrors = velocitySumOfErrors + actualVelocity;
         velocityIntergral = velocitySumOfErrors * VIM;
         velocityDiffrenceOfErrors = velocityError - velocityLastError;
         veloictyDerivitive = velocityDiffrenceOfErrors * VDM;
         velocityCorrection = (velocityPorportion + velocityIntergral + veloictyDerivitive);

         if (accelerationDistance >= Distance - Distance_From){
             velocitySetpoint = ((Distance - Distance_From)/accelerationDistance)* targetVelocity;
         }

         if (Distance_From <= Slow_Down_Distance) {

                 velocitySetpoint = (Distance_From / Slow_Down_Distance)*targetVelocity;
         }
         if (velocitySetpoint <=.265) {
             velocitySetpoint = .265;
         }
         if (velocitySetpoint >= 1){
             velocitySetpoint = 1;
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
         robot.LF_M.setPower((velocityCorrection*(LF_Distance/Highest_Motor_Power)));
         robot.LB_M.setPower((velocityCorrection*(LB_Distance/Highest_Motor_Power)));
         robot.RF_M.setPower((velocityCorrection*(RF_Distance/Highest_Motor_Power)));
         robot.RB_M.setPower((velocityCorrection*(RB_Distance/Highest_Motor_Power)));
     }
     public void stop_motors(){
         robot.LF_M.setPower(0);
         robot.LB_M.setPower(0);
         robot.RF_M.setPower(0);
         robot.RB_M.setPower(0);
     }
}
