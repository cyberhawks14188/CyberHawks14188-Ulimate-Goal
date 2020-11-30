package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp


public class Teleop extends LinearOpMode {


    double shooterPower;
    double intakePower;
    double stagerPower;
    double shooterAngle = 0;
    double Ring1Sensor;
    double Ring1Switch;
    double Ring2Switch;
    double Ring2Sensor;
    double RingCounter = 0;
    @Override
    public void runOpMode() {


        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);
        waitForStart();

        shooterPower = 0;
        while (opModeIsActive()) {
            Ring1Sensor = robot.Ring1_DS.getDistance(DistanceUnit.INCH);
            Ring2Sensor = robot.Ring2_DS.getDistance(DistanceUnit.INCH);
            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double z = -gamepad1.right_stick_x;

            robot.LF_M.setPower(y - (x + z));
            robot.LB_M.setPower(y + (x - z));
            robot.RF_M.setPower(y + (x + z));
            robot.RB_M.setPower(y - (x - z));

            if(Ring1Sensor < 7){
                Ring1Switch = 1;
            }else if(Ring1Sensor >= 7 && Ring1Switch == 1){
                RingCounter = RingCounter + 1;
                Ring1Switch = 0;
            }
            shooterAngle = shooterAngle + (.01 * (-gamepad2.left_stick_y));
            if (gamepad1.a) {
                shooterPower = 1;
            }
            if (gamepad1.b) {
                shooterPower = 0;
            }
            if (gamepad1.dpad_down) {
                stagerPower = -1;
            }
            if (gamepad1.dpad_right) {
                stagerPower = 0;
            }
            if(Ring2Sensor < 7){
                Ring2Switch = 1;
            }
            if (gamepad1.left_bumper || Ring2Switch == 1) {
                stagerPower = 0;
            }
            if (gamepad1.left_trigger >= .05) {
                stagerPower = -1;
                Ring2Switch = 0;
            }
            if (gamepad1.right_trigger >=.05){
                intakePower = -1;
            }
            if(gamepad1.right_bumper){
                intakePower = 0;
            }


            robot.SOT_M.setPower(shooterPower);
            robot.IN_M.setPower(intakePower);
            robot.STG_M.setPower(stagerPower);
            robot.SOT_S.setPosition(shooterAngle);
            telemetry.addData("Distance Sensor", Ring1Sensor);
            telemetry.addData("Shooter Angle", shooterAngle);
            telemetry.addData("Ring Counter", RingCounter);
            telemetry.update();

        }

    }
}