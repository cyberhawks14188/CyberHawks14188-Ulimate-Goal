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

            //Setting Motor Power
            robot.LF_M.setPower(Alg.DriveBase(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_bumper));
            robot.LB_M.setPower(Alg.LBMset());
            robot.RF_M.setPower(Alg.RFMset());
            robot.RB_M.setPower(Alg.RBMset());
            robot.WB_M.setPower(Alg.WobbleControl(gamepad1.left_trigger, gamepad1.dpad_up, gamepad1.dpad_down, robot.WB_PT.getVoltage()));
            robot.SOT_M.setPower(Alg.shooterControl(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.y, gamepad1.x, gamepad1.left_bumper, robot.SOT_M.getCurrentPosition(), getRuntime(), robot.SOT_PT.getVoltage()));
            robot.IN_M.setPower(Alg.RingSystem(gamepad1.a, gamepad1.b, Ring1Color.red, Ring2Color.red, Ring3Color.red, gamepad1.back));
            robot.STG_M.setPower(-Alg.stagerP());
            robot.GRIP_S.setPosition(Alg.grip());
            robot.SOT_S.setPower(Alg.sotaset());
            robot.STOP_S.setPosition(Alg.stopperSet());

            //Displaying Telemetry
            telemetry.addData("X Position", Alg.OdometryCalc(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition()));
            telemetry.addData("Y Position", Alg.odoy());
            telemetry.addData("Orientation (Degrees)", Math.toDegrees(Alg.thetaINRadians()));
            telemetry.addData("theta in Radians", Alg.thetaINRadians());
            telemetry.addData("E1", TV.e1current);
            telemetry.addData("E2", TV.e2current);
            telemetry.addData("E3", TV.e3current);
            telemetry.addData("WB_FSM", TV.WB_FSM);
            telemetry.addData("WBSET", TV.wobbleSet);
            telemetry.addData("WBerror", TV.wobbleError);
            telemetry.addData("WB_PT", robot.WB_PT.getVoltage());
            telemetry.addData("WBMotorPower", robot.WB_M.getPower());
            telemetry.addData("ShooterEncoder", robot.SOT_M.getCurrentPosition());
            telemetry.addData("ShooterPower", (TV.shooterSetpoint + TV.shooterCorrection) / 2800);
            telemetry.addData("shooterError", TV.shooterError);
            telemetry.addData("Shooter Correction", TV.shooterCorrection);
            telemetry.addData("shooterSetpoint", TV.shooterSetpoint);
            telemetry.addData("LFM", robot.LF_M.getPower());
            telemetry.addData("Potentiometer", robot.SOT_PT.getVoltage());
            telemetry.addData("SOTPower", TV.SOTPower);
            telemetry.addData("SOTAngleCurrent", TV.SOTCurrent);
            telemetry.addData("SOTAngleSet", TV.SOTSet);
            telemetry.addData("Ring1RedValue", TV.ring1Sensor);
            telemetry.addData("Ring2RedValue", TV.ring2Sensor);
            telemetry.addData("Ring3RedValue", TV.ring3Sensor);
            telemetry.update();
        } }}