package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp


public class Teleop extends LinearOpMode {


    double shooterPower;
    double intakePower;
    double stagerPower;
    double shooterAngle = 0;
    @Override
    public void runOpMode() {


        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);
        waitForStart();

        shooterPower = 0;
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double z = gamepad1.right_stick_x;

            robot.LF_M.setPower(y - (x + z));
            robot.LB_M.setPower(y + (x - z));
            robot.RF_M.setPower(-(y + (x + z)));
            robot.RB_M.setPower(-(y - (x - z)));

            if (shooterAngle < 0){
                shooterAngle = 0;
            }else if(shooterAngle >.35){
                shooterAngle = .35;
            }else if (gamepad1.left_bumper & shooterAngle >= 0 & shooterAngle <=.35){
                shooterAngle = shooterAngle + (.0005);
            }else if(gamepad1.left_trigger >= 0.1 & shooterAngle >= 0 & shooterAngle <=.35){
                shooterAngle = shooterAngle - (.0005);
            }

            if (gamepad1.a) {
                shooterPower = 1;
            }
            if (gamepad1.b) {
                shooterPower = .7;
            }
            if (shooterPower>1){
                shooterPower = 1;
            }else if (shooterPower < 0){
                shooterPower = 0;
            }else if (gamepad1.dpad_down){
                shooterPower = shooterPower - .001;
            }else if (gamepad1.dpad_up){
                shooterPower = shooterPower + .001;
            }
            if (gamepad1.right_bumper) {
                 stagerPower = -1;
            }
            if (gamepad1.right_trigger >= .05) {
                stagerPower = 0;
            }
            //stagerPower = gamepad2.right_stick_y;
            robot.SOT_M.setPower(shooterPower);
            robot.IN_M.setPower(intakePower);
            robot.STG_M.setPower(stagerPower);
            robot.SOT_S.setPosition(shooterAngle);
            telemetry.addData("Shooter Angle", shooterAngle);
            telemetry.addData("Shooter Speed", shooterPower);
            telemetry.update();

        }

    }
}