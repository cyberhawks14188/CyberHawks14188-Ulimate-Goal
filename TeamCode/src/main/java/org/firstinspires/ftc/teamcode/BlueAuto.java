package org.firstinspires.ftc.teamcode;
//Imports RobotCore
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//Creates program Class
@Autonomous
//Sets name
//
public class BlueAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    //Sets up Vuforia
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    //Uses Vuforia Developer Code
    private static final String VUFORIA_KEY = "AZickLn/////AAABmRdNRU8Vt0+EsSkecZ/dEtdwfmReQRmGjONFJw9IrZwj83V0JqVOw7lVMu8esNz/c2srCeQNiZSotXn5mKGHThTl4m0nN9xTmOVBgIkUOrtkA1rGeUkBw0dPy5AD8pk5L4Mv2yikiYUEXDVsPvVYjsp9p2+SHZNPXSBRL8OUPsUa+DpTQnpdRgtca4ZmRFGwUsfqkj/2pTz3/aS8KpFzZ6mjMVKJbJwiZnMhND5Bhy600+NNUNiTka0g6E+9lDEBQI5H0XVkEGCjHIFA0F8Z7L4iIZhotBPNB8kx3ep3MSRQSGg/yrzNIM4av2BqM2JVohuQFh2uSWyDJdgEwxtZ6drh3YZIa12CsuSHNkgaas2k";
    //Declares Varibles
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
    double SOTCurrent;
    public double X_Sum_of_Errors;
    double minimumVelocity;
    public double targetVelocity;
    double setVelocity;
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
    double GRIP_POS;
    public double z_encoder_diffrence;
    public double Distance;
    public double y_error;
    public double y_slope;
    public double x_slope;
    public double slope;
    public int last_Y_EndSetpoint;
    public double Z_PM;
    public double Z_IM;
    public double Z_DM;
    public double pass;
    public double X_PM;
    double Last_Y_EndSetpoint;
    double Last_X_EndSetpoint;
    public double X_IM;
    public double X_DM;
    public double Y_PM;
    public double Y_IM;
    public double Y_DM;
    public int last_X_EndSetpoint;
    double minimumAccelerationVelocity;
    public double Z_setpoint;
    public double Y_Distiance_From_Setpoint;
    public double X_EndSetpoint;
    public double VPM;
    public double velocityPorportion;
    double Y_Setpoint;
    double Slope_Y_error;
    double Slope_X_Error;
    double Slope_Y_porportional;
    double Slope_X_Porportional;
    double Slope_Y_Derivitive;
    double Slope_X_Derivitive;
    double Slope_Y_Correction;
    double Timedloop;
    double Slope_X_Correction;
    double X_Setpoint;
    public double expectedSpeedSetpoint;
    public double Y_EndSetpoint;
    public double LF_Distance;
    public double velocityError;
    public double LB_Distance;
    public double distanceWithin;
    public double actualVelocity;
    public double velocityCorrection;
    public double velocitySetpoint;
    public double E1;
    public double E2;
    double time_passed;
    public double E3;
    public double RF_Distance;
    public double RB_Distance;
    double leaveLoop;
    public double Speed_Setpoint;
    public double Highest_Motor_Power;
    public double Distance_From ;
    double stopper;
    double stagerPower;
    double Slope_Y_PM;
    double shooterPM;
    double shooterActualVelocity;
    double shooterLastEncoder;
    double shooterError;
    double shooterSetpoint;
    double shooterPorportional;
    double shooterCorrection;
    double lastTime;
    double timepassed;
    double WB_Setpoint;
    double WB_error;
    double WB_PM;
    double Y_intercept;
    double Slope_Y_Last_errorl;
    double Slope_Y_Last_error;
    double Slope_Y_DM;
    double Slope_X_PM;
    double Slope_X_last_error;
    double Slope_X_DM;
    public double Y_Average;
    public double startPosition;
    public double Y_A2;
    public double velocitySumOfErrors;
    public int loopcount;
    double storingVarible1;
    double done;
    double storingVarible2;
    public int Slow_Down_Distance;
    double SOTSet;
    double maximumVelocity;
    double test;
    double SOTError;
    double shooterPower;
    double lastDistance;
    double SOTPower;
    double SOTP;
    public double veloictyDerivitive;

    public double VDM;
    public double Slow_Rate;
    public double X_B2;
    public int breakout;
    public double runtime;
    public double Detected;
    public double Intial_Speed_Setpoint;
    public void runOpMode() {
        robot.init(hardwareMap);
        //Resets Encoders
        robot.LF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LB_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.LB_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Detected = 0;

        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }
        //recongniton.getLabel() = Single
        //recongniton.getLabel() = Quad
        pass = getRuntime() + 10;
        while (isStarted() != true) {
            tfod.setZoom(3, 1.78);
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        if (recognition.getLabel() == "Single") {
                            Detected = 1;
                        }
                        if (recognition.getLabel() == "Quad")
                            Detected = 2;
                    }
                    telemetry();
                    telemetry.update();
                }
            }
        }


        waitForStart();

            //tfod.shutdown();
            //Waits for start to be pressed
            //Set position robot will go to
            shooterSetpoint = 0;
            SOTSet = 1.47;
            Distance_From = 1;
            WB_Setpoint = .32;
            GRIP_POS = .585;
            breakout = 1;
            targetVelocity = 37;
            while (Distance_From > .5 & opModeIsActive()) {
                Movement(-15, 48, 0, 6, 6);
                SubSystem();
            }
            stop_motors();


            if (Detected == 0) {
                Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
                Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
                Distance_From = 1;
                breakout = 1;
                targetVelocity = 30;
                //Runs movement until 100 away
                while (Distance_From > .5 & opModeIsActive()) {
                    Movement(-18, 64, 0, 6, 6);
                    SubSystem();
                }
                stop_motors();

            }

            if (Detected == 1) {
                Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
                Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
                Distance_From = 1;
                breakout = 1;
                targetVelocity = 30;
                //Runs movement until 100 away

                while (Distance_From > .5 & opModeIsActive()) {
                    Movement(9, 88, 0, 6, 6);
                    SubSystem();
                }
                stop_motors();

            }
            if (Detected == 2) {
                Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
                Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
                Distance_From = 1;
                breakout = 1;
                targetVelocity = 40;
                //Runs movement until 100 away
                while (Distance_From > .5 & opModeIsActive()) {
                    Movement(-17, 112, 0, 13, 6);
                    SubSystem();
                }
                stop_motors();

            }

            Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
            Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
            WB_Setpoint = 2.04;
            Distance_From = 1;
            breakout = 1;
            targetVelocity = 5;
            shooterSetpoint = 1900;
            //velocitySetpoint = minimumVelocity;
            //Runs movement until 100 away
            while (robot.WB_PT.getVoltage() <= 2.02 & opModeIsActive()) {
                Movement(Last_X_EndSetpoint, Last_Y_EndSetpoint, 0, 1, 1);
                SubSystem();
                if (robot.WB_PT.getVoltage() > 2) {
                    GRIP_POS = .1;
                }
            }
            stop_motors();
            Timedloop = getRuntime() + .5;
            while(getRuntime() <= Timedloop){
                SubSystem();
            }
            Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
            Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
            WB_Setpoint = 1.5;
            Distance_From = 1;
            breakout = 1;
            targetVelocity = 30;

            //Runs movement until 100 away
            while (Distance_From >= .5 & opModeIsActive()) {
                Movement(18.5, 60, 0, 6, 6);
                SubSystem();
            }
        Last_X_EndSetpoint = -robot.LB_M.getCurrentPosition()* 0.00436111;
        Last_Y_EndSetpoint = (robot.LF_M.getCurrentPosition() * 0.00436111 + robot.RF_M.getCurrentPosition() * 0.00436111)/2;
        Distance_From = 1;
        breakout = 1;
        targetVelocity = minimumVelocity;
        stop_motors();
        Timedloop = getRuntime() + 1;
        while(getRuntime() <= Timedloop){
            SubSystem();
            Movement(Last_X_EndSetpoint, Last_Y_EndSetpoint, 0, .01, .01);

        }
        WB_Setpoint = 2.04;
        stagerPower = -.7;
        stopper = .5;
        shooterSetpoint = 1900;
        //velocitySetpoint = minimumVelocity;
        //Runs movement until 100 away
        while (robot.Ring1_DS.getDistance(DistanceUnit.INCH) < 2 || robot.Ring2_DS.getDistance(DistanceUnit.INCH) < 2 || robot.Ring3_DS.getDistance(DistanceUnit.INCH) < 4 & opModeIsActive()) {
            //Movement(Last_X_EndSetpoint, Last_Y_EndSetpoint, 0, 1, 1);

            SubSystem();
        }
        stop_motors();
        Timedloop = getRuntime() + 1;
        while(getRuntime() <= Timedloop){
            SubSystem();
        }

            stop_motors();
            while (opModeIsActive()) {
                telemetry();
            }
            stop_motors();
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
        telemetry.addData("WB_PT", robot.WB_PT.getVoltage());
        telemetry.addData("E1", robot.LF_M.getCurrentPosition() * 0.00436111);
        telemetry.addData("E2", -robot.LB_M.getCurrentPosition() * 0.00436111);
        telemetry.addData("E3", robot.RF_M.getCurrentPosition() * 0.00436111);
        telemetry.addData("Y Slope Correction",Slope_Y_Correction);
        telemetry.addData("X Slope Corredton", Slope_X_Correction);
        telemetry.addData("Distance_From", Distance_From);
        telemetry.addData("Distance", Distance);
        telemetry.addData("Speed_Setpoint", Speed_Setpoint);
        telemetry.addData("Velocitiy Setpoint", velocitySetpoint);
        telemetry.addData("Velocity Correction", velocityCorrection);
        telemetry.addData("Actual Velocity", actualVelocity);
        telemetry.addData("Y", y);
        telemetry.addData("X", x);
        telemetry.addData("Y_EndSetpoint", Y_EndSetpoint);
        telemetry.addData("X_EndSetpoint", X_EndSetpoint);
        telemetry.addData("Last_Y_EndSetpoint", Last_Y_EndSetpoint);
        telemetry.addData("Last_X_EndSetpoint", Last_X_EndSetpoint);
        telemetry.addData("LF", robot.LF_M.getPower());
        telemetry.addData("LB", robot.LB_M.getPower());
        telemetry.addData("test", test);
        telemetry.addData("RF", robot.RF_M.getPower());
        telemetry.addData("slope", slope);
        telemetry.addData("RB", robot.RB_M.getPower());
        telemetry.addData("Dectected", Detected);

        telemetry.update();
    }

    //Uses a PID to move robot to XYZ setpoints
    public void Movement(double X_EndSetpoint, double Y_EndSetpoint, double Z_setpoint, double Slow_Down_Distance, double accelerationDistance) {
        //Sets Multipliers
        X_PM = .15;
        X_IM = .000000001;
        X_DM = .3;
        Y_PM = .15;
        Y_IM = .000000001;
        Y_DM = .15;
        Z_PM = .4;
        Z_IM = .000000001;
        Z_DM = .5;
        VPM = .5;
        VIM = .00000001;
        VDM = .7;
        Slope_X_DM = .25;
        Slope_Y_PM = .25;
        Slope_X_PM = .25;
        Slope_Y_DM = .25;
        minimumAccelerationVelocity = 12;
        minimumVelocity = 2;

        //Gets encoder Positions
        E1 = robot.LF_M.getCurrentPosition() * 0.00436111;
        E2 = -robot.LB_M.getCurrentPosition() * 0.00436111;
        E3 = robot.RF_M.getCurrentPosition() * 0.00436111;
        //Sets encoders to 1 at begining to prevent null error
        if (E1 == 0) {
            E1 = .001;
        }
        if (E2 == 0) {
            E2 = .001;
        }

        if (E3 == 0) {
            E3 = .001;
        }
        //finds the diffrence of E1 and E3
        z_encoder_diffrence = E1 - E3;
        //Z Porportional
        z_error = Z_setpoint - z_encoder_diffrence;
        z_porportional = Z_PM * z_error;
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
        x_error = X_EndSetpoint - E2;
        //X Porportional
        x_porportional = x_error * X_PM;
        //X Intergral

        X_Sum_of_Errors = X_Sum_of_Errors + x_error;
        X_Intergral = X_Sum_of_Errors * X_IM;
        //X Derivitve
        X_Diffrence_of_Errors = x_error - X_Last_Error;
        X_Derivitive = X_Diffrence_of_Errors * X_DM;
        X_Last_Error = x_error;
        x = x_porportional + X_Intergral + X_Derivitive + Slope_X_Correction;
        //Finds Average Of E1&E3
        Y_Average = (E1 + E3) / 2;
        //finds the average of E1 and E3
        y_error = Y_EndSetpoint - Y_Average;
        //Y Porportional
        y_porportional = Y_PM * y_error;
        // Y Intergral
        //Finds the Sum of the Error
        Y_Sum_of_Errors = Y_Sum_of_Errors + y_error;
        Y_Intergral = Y_Sum_of_Errors * Y_IM;
        //Y Derivitive
        Y_Diffrence_of_Errors = y_error - Y_Last_Error;
        Y_Derivitive = Y_Diffrence_of_Errors * Y_DM;
        Y_Last_Error = y_error;
        y = y_porportional + Y_Intergral + Y_Derivitive + Slope_Y_Correction;

        //Uses pythagrium therom to find distance and distance from
        Distance = Math.sqrt((Math.pow((Y_EndSetpoint-Last_Y_EndSetpoint), 2)) + (Math.pow((X_EndSetpoint-Last_X_EndSetpoint), 2)));
        telemetry.addData("Y_EndSetpoint", Y_EndSetpoint);
        telemetry.addData("X_EndSetpoint", X_EndSetpoint);
        Y_A2 = Y_EndSetpoint - Y_Average;
        X_B2 = X_EndSetpoint - E2;
        Distance_From = Math.sqrt(Math.pow(Y_A2, 2) + (Math.pow(X_B2, 2)));
        if (breakout == 1) {
            lastDistanceFrom = Distance_From;
            breakout = 0;
        }

        //See if jitery or weird movement then add a back the interval sample time
        time = getRuntime();
        time_passed = time - (previousTime);
        //Velocity is number of ticks per second
        actualVelocity = Math.abs((Distance_From - lastDistanceFrom)) / time_passed;
        previousTime = time;
        lastDistanceFrom = Distance_From;
        if (accelerationDistance >= Distance - Distance_From) {

            velocitySetpoint = ((Distance - Distance_From) * (targetVelocity / accelerationDistance));
            if(velocitySetpoint <= minimumAccelerationVelocity){
                velocitySetpoint = minimumAccelerationVelocity;
            }
        }
        //Run the deacceleration when our distance from is less then the Slow_Down_Distance
        if (Distance_From <= Slow_Down_Distance) {

            velocitySetpoint = Distance_From * (targetVelocity / Slow_Down_Distance) + minimumVelocity;
        }
        storingVarible1 = Y_EndSetpoint;
        storingVarible2 = X_EndSetpoint;
        if (velocitySetpoint <= minimumVelocity) {
            velocitySetpoint = minimumVelocity;
        }
        //Velcotiy Max speed per second in inches
        if (velocitySetpoint >= 50) {
            velocitySetpoint = 50;
        }
        if(actualVelocity == 0 & velocitySetpoint >.1){
            velocitySetpoint = velocitySetpoint + 3;
        }
        //Find the error between desitired velcotiy and the current robot velcoity
        velocityError = velocitySetpoint - actualVelocity;
        //Runs and PID on the velcoity of our robot

        velocityPorportion = velocityError * VPM;
        velocitySumOfErrors = velocitySumOfErrors + velocityError;
        velocityIntergral = velocitySumOfErrors * VIM;
        velocityDiffrenceOfErrors = velocityError - velocityLastError;
        velocityLastError = velocityError;
        veloictyDerivitive = velocityDiffrenceOfErrors * VDM;
        //Finds the sum of the PID
        velocityCorrection = (velocityPorportion + velocityIntergral + veloictyDerivitive);
        //Velocity Minimum

        slope = ((Y_EndSetpoint - Last_Y_EndSetpoint) / (X_EndSetpoint - Last_X_EndSetpoint));
        //If we are going diangle
        if(Math.abs(slope) >= 1) {
            maximumVelocity = 40 + (10-((1/Math.abs(slope))*10));
        }
        else if(Math.abs(slope) < 1){
            maximumVelocity = 40 - (10-(Math.abs(slope) * 10));
        }
        Y_intercept = Y_EndSetpoint-(X_EndSetpoint*slope);
        if (Y_EndSetpoint - Last_Y_EndSetpoint != 0 & X_EndSetpoint - Last_X_EndSetpoint != 0) {
            Y_Setpoint = slope * E2+Y_intercept;
            X_Setpoint = (Y_Average-Y_intercept) / slope;
            telemetry.addData("1", "1");
        }
        //Horizonatal line
        else if (Y_EndSetpoint - Last_Y_EndSetpoint == 0) {
            maximumVelocity = 30;
            Y_Setpoint = Y_EndSetpoint;
            X_Setpoint = E2;
            telemetry.addData("2", "2");
        }
        //Vertical Line
        else if (X_EndSetpoint - Last_X_EndSetpoint == 0) {
            maximumVelocity = 50;
            Y_Setpoint = Y_Average;
            X_Setpoint = X_EndSetpoint;
            telemetry.addData("3", "3");
        }
        if (Distance_From <= 4) {

            Y_Setpoint = Y_EndSetpoint;
            X_Setpoint = X_EndSetpoint;

        }
        //Find error between the desired point on line and our currenet E2 reading
        Slope_Y_error = Y_Setpoint - Y_Average;
        //Finds error between the desired point on line and our curernt E1 and E3 average
        Slope_X_Error = X_Setpoint - E2;
        //Runs the Y Slope PD
        Slope_Y_porportional = Slope_Y_error * Slope_Y_PM;
        Slope_Y_Derivitive = (Slope_Y_error - Slope_Y_Last_error) * Slope_Y_DM;
        Slope_Y_Last_error = Slope_Y_error;
        //Find the sum of the porportional and the derivitive
        Slope_Y_Correction = Slope_Y_porportional + Slope_Y_Derivitive;
        //Runs the X SLope PD
        Slope_X_Porportional = Slope_X_Error * Slope_X_PM;
        Slope_X_Derivitive = (Slope_X_Error - Slope_X_last_error) * Slope_X_DM;
        Slope_X_last_error = Slope_X_Error;
        //Find the sum of the  porprotional and the dervititive
        Slope_X_Correction = Slope_X_Porportional + Slope_X_Derivitive;
        MotorEquation();
        telemetry();
    }

    public void MotorEquation() {
        //Motor equation from the PID output
        LF_Distance = y+(x+z);
        LB_Distance = y-(x-z);
        RF_Distance = y-(x+z);
        RB_Distance = y+(x-z);
        //Finds Highest power out of the drive motors
        Highest_Motor_Power = Math.max(Math.max(Math.abs(RF_Distance), Math.abs(RB_Distance)), Math.max(Math.abs(LF_Distance), Math.abs(LB_Distance)));
        //Sets motors
        robot.LF_M.setPower(((velocityCorrection+velocitySetpoint)/maximumVelocity) * (LF_Distance/Highest_Motor_Power));
        robot.LB_M.setPower(((velocityCorrection+velocitySetpoint)/maximumVelocity) * (LB_Distance/Highest_Motor_Power));
        robot.RF_M.setPower(((velocityCorrection+velocitySetpoint)/maximumVelocity) * (RF_Distance/Highest_Motor_Power));
        robot.RB_M.setPower(((velocityCorrection+velocitySetpoint)/maximumVelocity) * (RB_Distance/Highest_Motor_Power));
    }

    //Runs Encoders
    public void SubSystem() {
        SOTCurrent = robot.SOT_PT.getVoltage();
        SOTP = -20;
        WB_PM = .3;
        SOTError = SOTSet - SOTCurrent;
        SOTPower = SOTError * SOTP;
        WB_error = WB_Setpoint - robot.WB_PT.getVoltage();
        if (shooterSetpoint > 0){
            shooterPM = 15;
            timepassed = getRuntime() - lastTime;
            shooterActualVelocity = Math.abs(robot.SOT_M.getCurrentPosition() - shooterLastEncoder) / timepassed;
            lastTime = getRuntime();
            shooterLastEncoder = robot.SOT_M.getCurrentPosition();
            shooterError = shooterSetpoint - shooterActualVelocity;
            shooterPorportional = shooterError * shooterPM;
            shooterCorrection = shooterPorportional;
        }
        robot.WB_M.setPower(WB_error*WB_PM);
        robot.SOT_M.setPower((shooterCorrection + shooterSetpoint)/2800);
        robot.GRIP_S.setPosition(GRIP_POS);
        robot.SOT_S.setPower(SOTPower);
        robot.STOP_S.setPosition(stopper);
        robot.STG_M.setPower(-stagerPower);

    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.825f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}