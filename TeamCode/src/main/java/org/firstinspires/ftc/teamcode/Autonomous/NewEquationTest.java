package org.firstinspires.ftc.teamcode.Autonomous;
import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.TurnControl;
import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.DirectionCalcClass;
import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.SpeedClass;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses.Odometry;
@Autonomous

public class NewEquationTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    SpeedClass SpeedClass = new SpeedClass();
    DirectionCalcClass DirectionClass = new DirectionCalcClass();
    TurnControl TurnControl = new TurnControl();
    Odometry OdoClass = new Odometry();
    double breakout;
    double startPointX;
    double startPointY;
    double loopcount;
@Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        startPointX = 0;
        startPointY = 0;
        breakout = 1;
        while((DirectionClass.distanceFromReturn() >= .4 && opModeIsActive()) || (breakout == 1 && opModeIsActive())){
            Movement(50, 0, 0, 10, 2, 8);
            Telemetry();
            PowerSetting();
            breakout = 0;
        }
        /*startPointX = OdoClass.odoXReturn();
        startPointY = OdoClass.odoYReturn();
        while(opModeIsActive()){
            Movement(0, 0, 0, 10, 3, 3);
            Telemetry();
            PowerSetting();
        }
        \
        */

    }
    public void Telemetry(){
        telemetry.addData("Odo X",OdoClass.odoXReturn());
        telemetry.addData("Odo Y",OdoClass.odoYReturn());
        telemetry.addData("Theta Angle", OdoClass.thetaInDegreesReturn());
        telemetry.addData("Distance", DirectionClass.distanceReturn());
        telemetry.addData("Distance From", DirectionClass.distanceFromReturn());
        telemetry.addData("Speed",  SpeedClass.SpeedReturn());
        telemetry.addData("Current Speed", SpeedClass.CurrentSpeed());
        telemetry.addData("time", getRuntime());
        telemetry.addData("Distance Delta", SpeedClass.DistanceDelta());
        loopcount = loopcount + 1;
        telemetry.addData("Cycle count", loopcount);

        telemetry.update();
    }
    public void Movement(double endpointx, double endpointy, double thetasetpoint, double targetspeed, double accelerationdistance, double deccelerationdistance){
        OdoClass.OdometryCalc(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition(), getRuntime());
        DirectionClass.DirectionCalc(startPointX, startPointY, endpointx, endpointy, OdoClass.odoXReturn(), OdoClass.odoYReturn(), TurnControl.theta);
        SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn());
        SpeedClass.SpeedCalc(OdoClass.odoXReturn(), OdoClass.odoYReturn(), getRuntime(), SpeedClass.speedSetpoint);
        TurnControl.turnControl(thetasetpoint, OdoClass.thetaInDegreesReturn(),1);
        telemetry.addData("Speed Setpoint", SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn()));
    }
    public void PowerSetting(){
        robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * (SpeedClass.speed + .1));
        robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * (SpeedClass.speed+ .1));
        robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * (SpeedClass.speed+ .1));
        robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * (SpeedClass.speed+ .1));
    }
}
