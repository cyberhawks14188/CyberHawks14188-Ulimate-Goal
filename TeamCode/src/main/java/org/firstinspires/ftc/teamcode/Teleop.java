package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class Teleop extends LinearOpMode {

    //Declares Variables
    double shooterPower;
    double intakePower;
    double stagerPower;
    double ring1Sensor;
    double ring2Sensor;
    double ring3Sensor;
    double stopper;
    double xSpeedSetPoint = 1;
    double yzSpeedSetPoint = 1;
    double highestMotorPower;
    double LFM;
    double LBM;
    double RFM;
    double RBM;
    boolean stagerLoop = false;
    boolean stagerControl = false;
    boolean shooterControl = false;
    boolean gPadBControl = false;
    double wobbleSet = 0;
    double wobbleCurrent;
    double wobbleError;
    double wobblePower;
    double wobbleP = .01;
    double GRIP_S;
    double SOTCurrent;
    double SOTSet = 2;
    double SOTError;
    double SOTPower;
    double SOTP = 1.5;


    @Override
    public void runOpMode() {
        //Calling upon the HardwareMap
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);
        //Waits for the play button to be pressed
        waitForStart();
        //main loop
        while (opModeIsActive()) {

            //Stager Control
            //Takes a reading from all of the distance sensors
            ring1Sensor = robot.Ring1_DS.getDistance(DistanceUnit.INCH);
            ring2Sensor = robot.Ring2_DS.getDistance(DistanceUnit.INCH);
            ring3Sensor = robot.Ring3_DS.getDistance(DistanceUnit.INCH);

            if(gamepad1.a && stagerControl == false){
                if(intakePower == 0){
                    intakePower = -1;
                    stagerPower = -1;
                    stopper = .1;
                    stagerLoop = true;
                }else{
                    intakePower = 0;
                    stagerPower = 0;
                    stagerLoop = false;
                }
                stagerControl = true;
            }else if(!gamepad1.a){
                stagerControl = false;
            }
            if(stagerLoop == true && ring1Sensor < 2 && ring2Sensor< 2 && ring3Sensor < 4){
                    intakePower = 0;
                    stagerPower = 0;
            }
            /*if(gamepad1.b && gPadBControl == false){
                if(stopper < .3){
                    stopper = .5;
                    stagerPower = -1;
                }else{
                    stagerPower = 0;
                }
                gPadBControl = true;
            }else if(!gamepad1.b){
                gPadBControl = false;
            }
                */
            if(gamepad1.b){
                stopper = .5;
                stagerPower = -1;
            }
            //Shooter Control
            //Shooter angle
            SOTCurrent = robot.SOT_ANGL_PT.getVoltage();
            if(gamepad1.right_trigger > .05){
                SOTSet = SOTSet + .01;
            }else if(gamepad1.left_trigger > .05){
                SOTSet = SOTSet - .01;
            }
            SOTError = SOTSet - SOTCurrent;
            SOTPower = SOTError * SOTP;

            /*if(SOTPower >= 1){
                SOTPower = 1;
            }else if(SOTPower < 0){
                SOTPower = 0;
            }
*/
            //Stopper Servo Control
            if(gamepad1.x){
                stopper = .1;
            }else if(gamepad1.y){
                stopper = .5;
            }
            //Flywheel Speed Control
            if (gamepad1.left_bumper && shooterControl == false) {
                if(shooterPower == 0){
                    shooterPower = .8;
                }else{
                    shooterPower = 0;
                }
                shooterControl = true;
            }else if(!gamepad1.left_bumper){
                shooterControl = false;
            }
            if (gamepad1.dpad_up) {
                shooterPower = shooterPower + .01;
            }
            if (gamepad1.dpad_down) {
                shooterPower = shooterPower - .01;
            }

            //Gamepad2 manual controls
            //manual stager power control
            if(gamepad2.left_trigger > .05){
                stagerPower = 0;
            }else if(gamepad2.left_bumper){
                stagerPower = -1;
            }
            //Intake Control
            if (gamepad2.right_trigger >=.05){
                intakePower = 0;
            }
            if(gamepad2.right_bumper){
                intakePower = -1;
            }

            //Drivetrain Control
            if(gamepad1.right_bumper){
                yzSpeedSetPoint = .4;
                xSpeedSetPoint = .5;
            }else{
                yzSpeedSetPoint = 1;
                xSpeedSetPoint = 1;
            }
            double x = xSpeedSetPoint * -gamepad1.left_stick_x;
            double y = yzSpeedSetPoint * -gamepad1.left_stick_y;
            double z = yzSpeedSetPoint * -gamepad1.right_stick_x;
            LFM = y - (x + z);
            LBM = y + (x - z);
            RFM = y + (x + z);
            RBM = y - (x - z);
            highestMotorPower = Math.max(Math.max(Math.abs(LFM), Math.abs(LBM)), Math.max(Math.abs(RFM), Math.abs(RBM)));

            //Wobble Goal Arm
            wobbleCurrent = robot.WB_M.getCurrentPosition();
            wobbleSet = wobbleSet + gamepad2.right_stick_y;
            wobbleError = wobbleSet - wobbleCurrent;
            wobblePower = wobbleError * wobbleP;
            if (gamepad2.a){
                GRIP_S = .1;
            }else if(gamepad2.b){
                GRIP_S = .25;
            }

            //Setting Motor Power
            robot.LF_M.setPower(LFM);
            robot.LB_M.setPower(LBM);
            robot.RF_M.setPower(RFM);
            robot.RB_M.setPower(RBM);
            robot.WB_M.setPower(wobblePower);
            robot.SOT_M.setPower(shooterPower);
            robot.IN_M.setPower(intakePower);
            robot.STG_M.setPower(stagerPower);
            robot.GRIP_S.setPosition(GRIP_S);
            robot.SOT_S.setPower(SOTPower);
            robot.STOP_S.setPosition(stopper);

            //Displaying Telemetry
            telemetry.addData("shooterPower", shooterPower);
            telemetry.addData("stagerControl", stagerControl);
            telemetry.addData("DS1", ring1Sensor);
            telemetry.addData("DS2", ring2Sensor);
            telemetry.addData("DS3", ring3Sensor);
            telemetry.addData("Potentiometer", robot.SOT_ANGL_PT.getVoltage());
            telemetry.addData("SOTPower", SOTPower);
            telemetry.addData("SOTCurrent", SOTCurrent);
            telemetry.addData("SOTError", SOTError);
            telemetry.addData("SOTSet", SOTSet);
            telemetry.addData("WobblePower", wobblePower);
            telemetry.update();
        }
    }
}