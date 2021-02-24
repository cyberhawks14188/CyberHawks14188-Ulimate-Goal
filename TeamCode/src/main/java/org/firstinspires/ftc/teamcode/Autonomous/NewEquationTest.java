package org.firstinspires.ftc.teamcode.Autonomous;
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
    double startPointX;
    double startPointY;
@Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        startPointX = 0;
        startPointY = 0;
        while(opModeIsActive()){
            Movement(48, 0, 0, 1, 12, 12);
            Telemetry();
            PowerSetting();
        }
    }
    public void Telemetry(){
        telemetry.addData("Odo X",OdoClass.odoXReturn());
        telemetry.addData("Odo Y",OdoClass.odoYReturn());
        telemetry.addData("Distance", DirectionClass.distanceReturn());
        telemetry.addData("Distance From", DirectionClass.distanceFromReturn());
        telemetry.addData("Speed", SpeedClass.speed);
        telemetry.update();
    }
    public void Movement(double endpointx, double endpointy, double thetasetpoint, double targetspeed, double accelerationdistance, double deccelerationdistance){
        OdoClass.OdometryCalc(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
        SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance);
        SpeedClass.SpeedCalc(OdoClass.odoXReturn(), OdoClass.odoYReturn(), getRuntime(), SpeedClass.speedSetpoint);
        TurnControl.turnControl(thetasetpoint, OdoClass.thetaInDegreesReturn(),1);
        DirectionClass.DirectionCalc(startPointX, startPointY, endpointx, endpointy, OdoClass.odoXReturn(), OdoClass.odoYReturn(), 48, 0);
    }
    public void PowerSetting(){
        robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * SpeedClass.speed);
        robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * SpeedClass.speed);
        robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * SpeedClass.speed);
        robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * SpeedClass.speed);
    }
}
