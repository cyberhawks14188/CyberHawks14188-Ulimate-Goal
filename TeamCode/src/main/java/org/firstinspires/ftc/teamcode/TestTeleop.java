package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class  TestTeleop extends LinearOpMode {

    //Declares Variables to prevent NAN
    double shooterSetpoint;
    double intakePower;
    double stagerPower;
    double ring1Sensor;
    double ring2Sensor;
    double ring3Sensor;
    double stopper = .3;
    double xSpeedSetPoint = 1;
    double highestMotorPower;
    double LFM;
    double speed;
    double LBM;
    double RFM;
    double RBM;
    boolean stagerLoop = false;
    boolean stagerControl = false;
    boolean shooterControl = false;
    boolean gPadBControl = false;
    double leftG1StickPoint;
    double shooterVelocity;
    double shooterActualVelocity;
    double shooterPorportional;
    double shooterPM;
    double shooterDM;
    double shooterLastError;
    double shooterDerivitive;
    double shooterCorrection;
    double shooterError;
    double timepassed;
    double wobbleSet;
    double wobbleCurrent;
    double wobbleError;
    double wobblePower;
    double wobbleP = 1.1;
    double GRIP_S = .6;
    double shooterLastEncoder;
    double SOTCurrent;
    double SOTSet = 1.47;
    double lastTime;
    double SOTError;
    double SOTPower;
    double SOTP = -20;
    double shooterFSM = 0;
    double WB_FSM = 5;
    boolean WBControl = false;
    @Override
    public void runOpMode() {
        //Calling upon the HardwareMap
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);
        double wobbleEndSet = robot.WB_PT.getVersion();
        //Waits for the play button to be pressed
        waitForStart();
        //main loop
        while (opModeIsActive()) {
            //all sensor readings
            //Takes a reading from all of the distance sensors
            ring1Sensor = robot.Ring1_DS.getDistance(DistanceUnit.INCH);
            ring2Sensor = robot.Ring2_DS.getDistance(DistanceUnit.INCH);
            ring3Sensor = robot.Ring3_DS.getDistance(DistanceUnit.INCH);
            //Takes potentiometer reading and writes it to a variable for use later in program
            SOTCurrent = robot.SOT_PT.getVoltage();
            wobbleCurrent = robot.WB_PT.getVoltage();
            //Using a finite State Machine to easily control the stager and intake.
            //Also using custom one button on/off system where we use a boolean to see if the state has changed and then do an action
            if(gamepad1.a && stagerControl == false){
                if(intakePower == 0){
                    shooterFSM = 1;
                }else{
                    shooterFSM = 2;
                }
                stagerControl = true;
            }else if(!gamepad1.a){
                stagerControl = false;
            }
            //stage 0 shuts all motors off and this stager is starting stage
            if(shooterFSM == 0){
                intakePower = 0;
                stagerPower = 0;
            }
            //stage 1 is intaking stage. Sets ring stopper to closed and intakes until sensors see that there is 3 rings in robot.
            if(shooterFSM == 1){
                stopper = .3;
                if(ring1Sensor < 2 && ring2Sensor < 2 && ring3Sensor < 4){
                    intakePower = 0;
                    stagerPower = 0;
                    shooterFSM = 2;
                }else {
                    intakePower = -1;
                    stagerPower = -1;
                }
            }
            //stage 2 is shooting stage. If button b is pressed rings shoot other wise motors are off
            if(shooterFSM == 2) {
                if (gamepad1.b) {
                    stopper = .5;
                    stagerPower = -.9;
                    intakePower = 0;
                } else {
                    stagerPower = 0;
                    intakePower = 0;
                }
            }
            //Lets us reverse the intake direction if intake gets jammed
            if(gamepad1.back){
                intakePower = 1;
            }//POWERSHOT
            if(gamepad1.dpad_left){
                SOTSet = 1.645;
            }else if(gamepad1.dpad_right){
                SOTSet = 1.47;//TOP GOAL
            }
            //Shooter Control
            //Manual adjusting the setpoint to adjust last second if needed
            if(gamepad1.y){
                SOTSet = SOTSet - .003;
            }else if(gamepad1.x){
                SOTSet = SOTSet + .003;
            }

            //Shooter Angle PID Loop follo the setpoint set above
            SOTError = SOTSet - SOTCurrent;
            SOTPower = SOTError * SOTP;
            //Flywheel speed setpoint control. We use our custom one button on/off system to use the left bumper to set the shooter speed.
            if (gamepad1.left_bumper && shooterControl == false) {
                if(shooterSetpoint == 0){
                    shooterSetpoint = 1900;//set point is 1900 encoder ticks per second

                }else{
                    shooterSetpoint = 0;
                    shooterCorrection = 0;//we set both of these variables to ensure that neither one has power
                }
                shooterControl = true;
            }else if(!gamepad1.left_bumper){
                shooterControl = false;
            }
            //This is thhe Flywheels PID. This makes sure the Shotoer speed is the same no matter the battery power
            if(shooterSetpoint !=0){
                shooterPM = 15;//set the proportional multiplier for the shooter speed
                timepassed = getRuntime() - lastTime;
                shooterActualVelocity = Math.abs(robot.SOT_M.getCurrentPosition()-shooterLastEncoder)/timepassed;
                lastTime = getRuntime();
                shooterLastEncoder = robot.SOT_M.getCurrentPosition();
                shooterError = shooterSetpoint - shooterActualVelocity;
                shooterPorportional = shooterError *  shooterPM;
                shooterCorrection = shooterPorportional;
            }
            //finite State machine for the wobble goal.
            //We use the custom one button cycle to switch between each state.
            if(gamepad1.left_trigger > .05 && WBControl == false){
                if(WB_FSM < 4){
                    WB_FSM = WB_FSM + 1;
                }else{
                    WB_FSM = 0;
                }
                WBControl = true;
            }else if(gamepad1.left_trigger < .05){
                WBControl = false;
            }
            //State 0 is claw open and in grabbing position
            if(WB_FSM == 0) {
                wobbleEndSet = 2.324;
                GRIP_S = .1;
            }else if(WB_FSM == 1){//state 1 is grab wobble goal while still in down position
                GRIP_S = .7;
            }else if(WB_FSM == 2){//Brings wobble goal arm in stored position to drive to wall
                wobbleEndSet = .6;
            }else if(WB_FSM == 3) {//Brings wobble goal above wall, gripper still closed
                wobbleEndSet = 1;
            }else if(WB_FSM == 4){// Gripper opens
                GRIP_S = .1;
            }else if(WB_FSM == 5){//Closes Claw
                GRIP_S = .7;
            }
            //Lets up manually change set point if needed. Set EndSet to current set point + or - t o allow the arm to go to its current position + or - the setpoint.
            if (gamepad1.dpad_up) {
                wobbleEndSet = wobbleSet - .1;
            } else if (gamepad1.dpad_down) {
                wobbleEndSet = wobbleSet + .1;
            }
            //This slowly brings the arm to the endsetpoint to ensure little wear from arm bannging on the wheels
            if(wobbleEndSet > .2 + wobbleSet){
                wobbleSet = wobbleSet + .05;
            }else if(wobbleEndSet < .2 - wobbleSet){
                wobbleSet = wobbleSet - .05;
            }else{
                wobbleSet = wobbleEndSet;
            }
            //PID to control the wobble arm to go to desired set point
            wobbleError = wobbleSet - robot.WB_PT.getVoltage();
            wobblePower = wobbleError * wobbleP;
            //Drivetrain Control
            //Sets slow speed by using the right bumper
            if(gamepad1.right_bumper){
                xSpeedSetPoint = .5;
            }else{
                xSpeedSetPoint = 1;
            }
            //sets calculation variables to let us calculate drivetrain direction
            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double z = -gamepad1.right_stick_x;
            //calculates motor speed and direction
            LFM = y - (x + z);
            LBM = y + (x - z);
            RFM = y + (x + z);
            RBM = y - (x - z);
            //We use highest motor power to make sure no wheel speed ever goes over 1 and lets su strafe more accuratly
            highestMotorPower = Math.max(Math.max(Math.abs(LFM), Math.abs(LBM)), Math.max(Math.abs(RFM), Math.abs(RBM)));
            leftG1StickPoint = Math.sqrt((gamepad1.left_stick_x*gamepad1.left_stick_x) + (gamepad1.left_stick_y*gamepad1.left_stick_y));
            //Since we use highest motor power to make sure no motor power goes over 1, there will always be a motor going full power
            //So we use the joysticks again to make apply a ratio and the set motor power
            if (Math.abs(gamepad1.right_stick_x) <.01){
                speed = leftG1StickPoint;
            }else if(Math.abs(gamepad1.left_stick_x + gamepad1.left_stick_y) < .02){
                speed = Math.abs(gamepad1.right_stick_x);
            } else{
                speed = (leftG1StickPoint + Math.abs(gamepad1.right_stick_x))/2;
            }
            //Displaying Telemetry
            telemetry.addData("WB_FSM", WB_FSM);
            telemetry.addData("WBSET", wobbleSet);
            telemetry.addData("WBerror", wobbleError);
            telemetry.addData("WB_PT", robot.WB_PT.getVoltage());
            telemetry.addData("E1", robot.LF_M.getCurrentPosition() * 0.00436111);
            telemetry.addData("E2", robot.LB_M.getCurrentPosition() * 0.00436111);
            telemetry.addData("E3", robot.RF_M.getCurrentPosition() * 0.00436111);
            telemetry.addData("ShooterEncoder", robot.SOT_M.getCurrentPosition());
            telemetry.addData("ShooterPower", (shooterSetpoint+shooterCorrection)/2800);
            telemetry.addData("shooterError", shooterError);
            telemetry.addData("Shooter Correction", shooterCorrection);
            telemetry.addData("shooterSetpoint", shooterSetpoint);
            telemetry.addData("LeftX", gamepad1.left_stick_x);
            telemetry.addData("LeftY", gamepad1.left_stick_y);
            telemetry.addData("speed", speed);
            telemetry.addData("LFM", robot.LF_M.getPower());
            telemetry.addData("shooterSetpoint", shooterSetpoint);
            telemetry.addData("stagerControl", stagerControl);
            telemetry.addData("DS1", ring1Sensor);
            telemetry.addData("DS2", ring2Sensor);
            telemetry.addData("DS3", ring3Sensor);
            telemetry.addData("Potentiometer", robot.SOT_PT.getVoltage());
            telemetry.addData("SOTPower", SOTPower);
            telemetry.addData("SOTCurrent", SOTCurrent);
            telemetry.addData("SOTError", SOTError);
            telemetry.addData("SOTSet", SOTSet);
            telemetry.addData("WobblePower", wobblePower);
            telemetry.addData("WBmotor", robot.WB_M.getPower());
            telemetry.update();

            //Setting Motor Power
            robot.LF_M.setPower(((LFM/highestMotorPower) * speed)* xSpeedSetPoint);
            robot.LB_M.setPower(((LBM/highestMotorPower) * speed)* xSpeedSetPoint);
            robot.RF_M.setPower(((RFM/highestMotorPower) * speed)* xSpeedSetPoint);
            robot.RB_M.setPower(((RBM/highestMotorPower) * speed)* xSpeedSetPoint);
            robot.WB_M.setPower(wobblePower);
            robot.SOT_M.setPower((shooterSetpoint + shooterCorrection)/2800);
            robot.IN_M.setPower(intakePower);
            robot.STG_M.setPower(-stagerPower);
            robot.GRIP_S.setPosition(GRIP_S);
            robot.SOT_S.setPower(SOTPower);
            robot.STOP_S.setPosition(stopper);
        }
    }
}