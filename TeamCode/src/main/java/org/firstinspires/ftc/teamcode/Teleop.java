package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Teleop", group="Iterative Opmode")
@Disabled
public class Teleop extends OpMode
{
    RobotHardware robot = new RobotHardware();
    double shooterPower;
    double intakePower;
    double stagerPower;
    // Declare OpMode members.
    //private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        shooterPower = 0;
    }
    @Override
    public void loop() {
        while(opModeIsActive()){
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double z = gamepad1.right_stick_x;

            robot.LF_M.setPower(y-(x-z));
            robot.LB_M.setPower(y+(x+z));
            robot.RF_M.setPower(y+(x-z));
            robot.RB_M.setPower(y-(x+z));

            if(gamepad1.a){
               shooterPower = 1;
            }
            if(gamepad1.b){
                shooterPower = 0;
            }
            if(gamepad1.left_bumper){
                intakePower = 1;
            }
            if(gamepad1.left_trigger >= .05){
                intakePower = 0;
            }
            if(gamepad1.right_bumper){
                stagerPower = 1;
            }
            robot.SOT_M.setPower(shooterPower);




        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
