package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Teleop", group="Linear Opmode")
@Disabled

public class Teleop extends LinearOpMode {


    double shooterPower;
    double intakePower;
    double stagerPower;
    @Override
    public void runOpMode() {


        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);


        shooterPower = 0;
        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double z = gamepad1.right_stick_x;

            robot.LF_M.setPower(y - (x - z));
            robot.LB_M.setPower(y + (x + z));
            robot.RF_M.setPower(y + (x - z));
            robot.RB_M.setPower(y - (x + z));

            if (gamepad1.a) {
                shooterPower = 1;
            }
            if (gamepad1.b) {
                shooterPower = 0;
            }
            if (gamepad1.left_bumper) {
                intakePower = 1;
            }
            if (gamepad1.left_trigger >= .05) {
                intakePower = 0;
            }
            if (gamepad1.right_bumper) {
                stagerPower = 1;
            }
            if (gamepad1.right_trigger >= .05) {
                stagerPower = 0;
            }
            robot.SOT_M.setPower(shooterPower);
            robot.IN_M.setPower(intakePower);
            robot.STG_M.setPower(stagerPower);


        }

    }
}