package org.firstinspires.ftc.teamcode.TeleOpCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp
public class OdoTeleop extends LinearOpMode {
    TeleOpVariables TV = new TeleOpVariables();
    TeleOpAlgorithms Alg = new TeleOpAlgorithms();
    @Override
    public void runOpMode() {
        //Calling upon the HardwareMap and other classes

        
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);
//assigning stating types for sensors
        TV.wobbleEndSet = robot.WB_PT.getVoltage();
        robot.Ring1_CS.setGain(10);
        robot.Ring2_CS.setGain(10);
        robot.Ring3_CS.setGain(10);
        NormalizedRGBA Ring1Color = robot.Ring1_CS.getNormalizedColors();
        NormalizedRGBA Ring2Color = robot.Ring2_CS.getNormalizedColors();
        NormalizedRGBA Ring3Color = robot.Ring3_CS.getNormalizedColors();

        //Waits for the play button to be pressed
        waitForStart();
        //main loop
        while (opModeIsActive()) {
            //setting variables to use in other classes
            TV.x = gamepad1.left_stick_x;
            TV.y = gamepad1.left_stick_y;
            TV.z = gamepad1.right_stick_x;

            TV.e1current = robot.LF_M.getCurrentPosition();
            TV.e2current = robot.LB_M.getCurrentPosition();//add in multipllyer offset if needed
            TV.e3current = robot.RF_M.getCurrentPosition();

            TV.ring1Sensor = Ring1Color.red;
            TV.ring2Sensor = Ring2Color.red;
            TV.ring3Sensor = Ring3Color.red;

            //Takes potentiometer reading and writes it to a variable for use later in program
            TV.SOTCurrent = robot.SOT_PT.getVoltage();
            TV.wobbleCurrent = robot.WB_PT.getVoltage();

            //DriveBase Slowdown
        if (gamepad1.right_bumper) {
            TV.xSpeedSetPoint = .5;
        } else {
            TV.xSpeedSetPoint = 1;
        }

            TV.gamepadAState = gamepad1.a;
            TV.gamepadBState = gamepad1.b;
            TV.gamepadDpadLeftState = gamepad1.dpad_left;
            TV.gamepadDpadRightState = gamepad1.dpad_right;
            TV.gamepadXState = gamepad1.x;
            TV.gamepadYState = gamepad1.y;
            TV.currentRunTime = getRuntime();
            TV.SOTCurrentEncoder = robot.SOT_M.getCurrentPosition();
            TV.wobbleCurrent = robot.WB_PT.getVoltage();
            TV.gamepadLeftTrigger = gamepad1.left_trigger;
            TV.gamepadDpadUpState = gamepad1.dpad_up;
            TV.gamepadDpadDownState = gamepad1.dpad_down;

            Alg.OdometryCalc();
            Alg.RingSystem();
            Alg.shooterControl();
            Alg.WobbleControl();
            Alg.DriveBase();

            //Lets us reverse the intake direction if intake gets jammed
            if (gamepad1.back) {
                TV.intakePower = 1;
            }

            //Displaying Telemetry
            telemetry.addData("X Position", TV.xCoordinatePosition / TV.COUNTS_PER_INCH);
            telemetry.addData("Y Position", TV.yCoordinatePosition / TV.COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", Math.toDegrees(TV.thetaInRadians));
            telemetry.addData("theta in Radians", TV.thetaInRadians);
            telemetry.addData("WB_FSM", TV.WB_FSM);
            telemetry.addData("WBSET", TV.wobbleSet);
            telemetry.addData("WBerror", TV.wobbleError);
            telemetry.addData("WB_PT", robot.WB_PT.getVoltage());
            telemetry.addData("E1", TV.e1current);
            telemetry.addData("E2", TV.e2current);
            telemetry.addData("E3", TV.e3current);
            telemetry.addData("ShooterEncoder", robot.SOT_M.getCurrentPosition());
            telemetry.addData("ShooterPower", (TV.shooterSetpoint + TV.shooterCorrection) / 2800);
            telemetry.addData("shooterError", TV.shooterError);
            telemetry.addData("Shooter Correction", TV.shooterCorrection);
            telemetry.addData("shooterSetpoint", TV.shooterSetpoint);
            telemetry.addData("speed", TV.speed);
            telemetry.addData("LFM", robot.LF_M.getPower());
            telemetry.addData("shooterSetpoint", TV.shooterSetpoint);
            telemetry.addData("stagerControl", TV.stagerControl);
            telemetry.addData("Potentiometer", robot.SOT_PT.getVoltage());
            telemetry.addData("SOTPower", TV.SOTPower);
            telemetry.addData("SOTCurrent", TV.SOTCurrent);
            telemetry.addData("SOTError", TV.SOTError);
            telemetry.addData("SOTSet", TV.SOTSet);
            telemetry.addData("WobblePower", TV.wobblePower);
            telemetry.addData("WBmotor", robot.WB_M.getPower());
            telemetry.addData("Ring1RedValue", TV.ring1Sensor);
            telemetry.addData("Ring2RedValue", TV.ring2Sensor);
            telemetry.addData("Ring3RedValue", TV.ring3Sensor);
            telemetry.update();

            //Setting Motor Power
            robot.LF_M.setPower(((TV.LFM / TV.highestMotorPower) * TV.speed) * TV.xSpeedSetPoint);
            robot.LB_M.setPower(((TV.LBM / TV.highestMotorPower) * TV.speed) * TV.xSpeedSetPoint);
            robot.RF_M.setPower(((TV.RFM / TV.highestMotorPower) * TV.speed) * TV.xSpeedSetPoint);
            robot.RB_M.setPower(((TV.RBM / TV.highestMotorPower) * TV.speed) * TV.xSpeedSetPoint);
            robot.WB_M.setPower(TV.wobblePower);
            robot.SOT_M.setPower((TV.shooterSetpoint + TV.shooterCorrection) / 2800);
            robot.IN_M.setPower(TV.intakePower);
            robot.STG_M.setPower(-TV.stagerPower);
            robot.GRIP_S.setPosition(TV.GRIP_S);
            robot.SOT_S.setPower(TV.SOTPower);
            robot.STOP_S.setPosition(TV.stopper);
        } }}