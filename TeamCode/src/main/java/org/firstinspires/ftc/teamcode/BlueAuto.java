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
    double minimum_speed;
    public double Y_DM;
    public int last_X_EndSetpoint;
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
    public double Speed_Setpoint;
    public double Highest_Motor_Power;
    public double Distance_From ;
    double stopper;
    double stagerPower;
    double Slope_Y_PM;
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
    public int Slow_Down_Distance;
    double SOTSet;
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
        //Runs setup for vuforia
        initVuforia();
        initTfod();
        if (tfod != null) {
             tfod.activate();
        }
        //Sets up the 2 diffrent ring placements

        pass = getRuntime() + 5;
        //Runs vuforia for a certain amount of time to deteect how many rings are in the stack
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
        tfod.shutdown();
        //Waits for start to be pressed


        //Set position robot will go to

        Distance_From = 60;
        breakout = 1;
        minimum_speed = 15;
        minimumVelocity = 3;
        distanceWithin = 1;
        startPosition = 0;
        targetVelocity = 45;
        //Runs movement until 100 away
        while (Distance_From >= .5) {
            Movement(-10, 50, 0, 12, 6);
        }
        lastDistance =  Distance;
        stop_motors();
        if(Detected == 0){
            Distance_From = 30;
            breakout = 1;
            minimum_speed = 15;
            minimumVelocity = 3;
            distanceWithin = 1;
            startPosition = 0;
            targetVelocity = 32;
            //Runs movement until 100 away
            while (Distance_From >= .5) {
                Movement(-8, 80, 0, 12, 6);
            }
        }
        if(Detected == 1){
            Distance_From = 50;
            breakout = 1;
            minimum_speed = 15;
            minimumVelocity = 3;
            distanceWithin = 1;
            startPosition = 0;
            targetVelocity = 35;
            //Runs movement until 100 away
            while (Distance_From >= .5) {
                Movement(10, 98, 0, 12, 6);
            }
        }
        if(Detected == 2){
            Distance_From = 120;
            breakout = 1;
            minimum_speed = 15;
            
            minimumVelocity = 3;
            distanceWithin = 1;
            startPosition = 0;
            targetVelocity = 45;
            //Runs movement until 100 away
            while (Distance_From >= .5) {
                Movement(-8, 134, 0, 12, 6);
            }
        }
        Distance_From = 50;
        breakout = 1;
        minimum_speed = 15;
        minimumVelocity = 3;
        distanceWithin = 1;
        startPosition = 0;
        targetVelocity = 45;
        //Runs movement until 100 away
        while (Distance_From >= .5) {
            Movement(54, 85, 0, 12, 6);
        }
        Distance_From = 120;
        breakout = 1;
        minimum_speed = 15;
        minimumVelocity = 3;
        distanceWithin = 1;
        startPosition = 0;
        targetVelocity = 45;
        //Runs movement until 100 away
        while (Distance_From >= .5) {
            Movement(25, 5, 0, 12, 6);
        }
        Distance_From = 120;
        breakout = 1;
        minimum_speed = 15;
        minimumVelocity = 3;
        distanceWithin = 1;
        startPosition = 0;
        targetVelocity = 45;
        //Runs movement until 100 away
        while (Distance_From >= .5) {
            Movement(-18, 50, 0, 12, 6);
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
        telemetry.addData("Velocitiy Setpoint", velocitySetpoint);
        telemetry.addData("Velocity Correction", velocityCorrection);
        telemetry.addData("Actual Velocity", actualVelocity);
        telemetry.addData("Y", y);
        telemetry.addData("X", x);
        telemetry.addData("Y_EndSetpoint", Y_EndSetpoint);
        telemetry.addData("X_EndSetpoint", X_EndSetpoint);
        telemetry.addData("LF", Speed_Setpoint*(LF_Distance/Highest_Motor_Power));
        telemetry.addData("LB", Speed_Setpoint*(LB_Distance/Highest_Motor_Power));
        telemetry.addData("RF", Speed_Setpoint*(RF_Distance/Highest_Motor_Power));
        telemetry.addData("RB", Speed_Setpoint*(RB_Distance/Highest_Motor_Power));
        telemetry.addData("Dectected", Detected);
        telemetry.update();
    }

    //Uses a PID to move robot to XYZ setpoints
    public void Movement(double X_EndSetpoint, double Y_EndSetpoint, double Z_setpoint, double Slow_Down_Distance, double accelerationDistance) {
        //Sets Multipliers
        X_PM = 1.5;
        X_IM = .000001;
        X_DM = 1;
        Y_PM = .5;
        Y_IM = .00001;
        Y_DM = .3;
        Z_PM = 1.5;
        Z_IM = .00001;
        Z_DM = .9;
        VPM = .96;
        VIM = .00099;
        VDM = .96;
        Slope_X_DM = .3;
        Slope_Y_PM = .3;
        Slope_X_PM = .3;
        Slope_Y_DM = .3;


        //Gets encoder Positions
        E1 = robot.LF_M.getCurrentPosition() * 0.00436111;
        E2 = -robot.LB_M.getCurrentPosition() * 0.00436111;
        E3 = robot.RF_M.getCurrentPosition() * 0.00436111;
        //Sets encoders to 1 at begining to prevent null error
        if (E1 == 0) {
            E1 = 1;
        }
        if (E2 == 0) {
            E2 = 1;
        }

        if (E3 == 0) {
            E3 = 1;
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
        //slope equation
       /* if(X_Final_Setpoint > 0 & Y_Final_Setpoint > 0){
           slope = Y_Final_Setpoint/X_Final_Setpoint;
        Y_EndSetpoint = slope * E2;
        X_EndSetpoint = (((E1+E3)/2)/slope);
        if(last_Y_EndSetpoint > 0){
            Y_EndSetpoint = Y_EndSetpoint + last_Y_EndSetpoint;
        }
        if(last_X_EndSetpoint > 0){
            X_EndSetpoint = X_EndSetpoint + last_X_EndSetpoint;
        }
           bypass = 1;
        }
        if(Y_EndSetpoint >= Y_Final_Setpoint) {
            Y_EndSetpoint = Y_Final_Setpoint;
        }
        if(X_EndSetpoint >= X_Final_Setpoint){
            X_EndSetpoint = X_Final_Setpoint;
         }
*/

        //Uses pythagrium therom to find distance and distance from
        Distance = Math.abs((Math.sqrt(Math.pow(Y_EndSetpoint, 2) + (Math.pow(X_EndSetpoint, 2)))) - lastDistance);
        Y_A2 = Y_EndSetpoint - Y_Average;
        X_B2 = X_EndSetpoint - E2;
        Distance_From = Math.sqrt(Math.pow(Y_A2, 2) + (Math.pow(X_B2, 2)));

        if (breakout == 1) {
            lastDistanceFrom = Distance_From;
            breakout = 0;
            slope = ((Y_EndSetpoint - Last_Y_EndSetpoint) / (X_EndSetpoint - Last_X_EndSetpoint));
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
            if (velocitySetpoint <= minimum_speed) {
                velocitySetpoint = minimum_speed;
            }
        }
        //Run the deacceleration when our distance from is less then the Slow_Down_Distance
        if (Distance_From <= Slow_Down_Distance) {

            velocitySetpoint = Distance_From * (targetVelocity / Slow_Down_Distance) + minimumVelocity;
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
        if (velocitySetpoint <= minimumVelocity) {
            velocitySetpoint = minimumVelocity;
        }
        //Velcotiy Max speed per second in inches
        if (velocitySetpoint >= 62.8) {
            velocitySetpoint = 62.8;
        }


        //If we are going diangle
        if (Y_EndSetpoint - Last_Y_EndSetpoint != 0 & X_EndSetpoint - Last_X_EndSetpoint != 0) {


            Y_Setpoint = slope * E2;
            X_Setpoint = Y_Average / slope;

        }
        //Horizonatal line
        else if (Y_EndSetpoint - Last_Y_EndSetpoint == 0) {
            Y_Setpoint = Y_EndSetpoint;
            X_Setpoint = E2;
        }
        //Vertical Line
        else if (X_EndSetpoint - Last_X_EndSetpoint != 0) {
            Y_Setpoint = Y_Average;
            X_Setpoint = X_EndSetpoint;
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
        Last_X_EndSetpoint = X_EndSetpoint;
        Last_Y_EndSetpoint = Y_EndSetpoint;
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
        robot.LF_M.setPower(((velocityCorrection+velocitySetpoint)/62.8) * (LF_Distance/Highest_Motor_Power));
        robot.LB_M.setPower(((velocityCorrection+velocitySetpoint)/62.8) * (LB_Distance/Highest_Motor_Power));
        robot.RF_M.setPower(((velocityCorrection+velocitySetpoint)/62.8) * (RF_Distance/Highest_Motor_Power));
        robot.RB_M.setPower(((velocityCorrection+velocitySetpoint)/62.8) * (RB_Distance/Highest_Motor_Power));
    }

    //Runs Encoders
    public void SubSystem() {
        SOTCurrent = robot.SOT_ANGL_PT.getVoltage();
        SOTP = 1.5;
        SOTError = SOTSet - SOTCurrent;
        SOTPower = SOTError * SOTP;
        robot.SOT_S.setPower(SOTPower);
        robot.STOP_S.setPosition(stopper);
        robot.STG_M.setPower(stagerPower);
        robot.SOT_M.setPower(shooterPower);

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
        tfodParameters.minResultConfidence = 0.75f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
