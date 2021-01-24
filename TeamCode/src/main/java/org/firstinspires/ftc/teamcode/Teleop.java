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
    double stopper = .3;
    double xSpeedSetPoint = 1;
    double yzSpeedSetPoint = 1;
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
    double wobbleSet = 0;
    double wobbleCurrent;
    double wobbleError;
    double wobblePower;
    double wobbleP = .01;
    double GRIP_S = .4;
    double SOTCurrent;
    double SOTSet = 1.45;
    double SOTError;
    double SOTPower;
    double SOTP = -30;


    @Override
    public void runOpMode() {
        //Calling upon the HardwareMap
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);
        //Waits for the play button to be pressed
        waitForStart();
        //main loop
        while (opModeIsActive()) {

            //all sensor readings
                 //Takes a reading from all of the distance sensors
                ring1Sensor = robot.Ring1_DS.getDistance(DistanceUnit.INCH);
                ring2Sensor = robot.Ring2_DS.getDistance(DistanceUnit.INCH);
                ring3Sensor = robot.Ring3_DS.getDistance(DistanceUnit.INCH);

                //Takes potentiometer reading and writes it to varible
                SOTCurrent = robot.SOT_PT.getVoltage();

                //Gets wobble goal motor encoder reading
                wobbleCurrent = robot.WB_M.getCurrentPosition();

            //Intake Control and stager control
                if(gamepad1.a && stagerControl == false){
                    if(intakePower == 0){
                        intakePower = -1;
                        stagerPower = -1;
                        stopper = .3;
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
                if(gamepad1.b){
                    stopper = .5;
                    stagerPower = -1;
                }

            //Shooter Control
                //Shooter angle
                if(gamepad1.y){
                    SOTSet = SOTSet - .001;
                }else if(gamepad1.x){
                    SOTSet = SOTSet + .001;
                }
                SOTError = SOTSet - SOTCurrent;
                SOTPower = SOTError * SOTP;

                //Stopper Servo Control
                if(gamepad2.x){
                    stopper = .3;
                }else if(gamepad2.y){
                    stopper = .5;
                }
                if(gamepad1.back){
                    SOTSet = .78;
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
                if (gamepad1.dpad_right) {
                    shooterPower = shooterPower + .01;
                }
                if (gamepad1.dpad_left) {
                    shooterPower = shooterPower - .01;
                }

            //Wobble Goal Arm
            if(gamepad1.dpad_up){
                wobbleSet = wobbleSet - 10;
            }else if(gamepad1.dpad_down) {
                wobbleSet = wobbleSet + 10;
            }
            wobbleError = wobbleSet - wobbleCurrent;
            wobblePower = wobbleError * wobbleP;
            if (gamepad1.right_trigger > .05){
                GRIP_S = .6;
            }else if(gamepad1.left_trigger > .05){
                GRIP_S = .1;
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
                leftG1StickPoint = Math.sqrt((gamepad1.left_stick_x * gamepad1.left_stick_x)+ (gamepad1.left_stick_y*gamepad1.left_stick_y));
                if (Math.abs(gamepad1.right_stick_x) <.005){
                    speed = leftG1StickPoint;
                }
                else{
                    speed = (leftG1StickPoint + Math.abs(gamepad1.right_stick_x))/2;
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

            //Setting Motor Power
            robot.LF_M.setPower((LFM/highestMotorPower) * speed);
            robot.LB_M.setPower((LBM/highestMotorPower) * speed);
            robot.RF_M.setPower((RFM/highestMotorPower) * speed);
            robot.RB_M.setPower((RBM/highestMotorPower) * speed);
            robot.WB_M.setPower(wobblePower);
            robot.SOT_M.setPower(shooterPower);
            robot.IN_M.setPower(intakePower);
            robot.STG_M.setPower(stagerPower);
            robot.GRIP_S.setPosition(GRIP_S);
            robot.SOT_S.setPower(SOTPower);
            robot.STOP_S.setPosition(stopper);

            //Displaying Telemetry
            telemetry.addData("LeftX", gamepad1.left_stick_x);
            telemetry.addData("LeftY", gamepad1.left_stick_y);
            telemetry.addData("speed", speed);
            telemetry.addData("LFM", robot.LF_M.getPower());
            telemetry.addData("shooterPower", shooterPower);
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
            telemetry.update();
        }
    }
}