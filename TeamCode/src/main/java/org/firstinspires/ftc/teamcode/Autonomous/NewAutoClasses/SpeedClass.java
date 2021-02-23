package org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses;
import org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses.DirectionCalcClass;
public class SpeedClass {
    DirectionCalcClass DirectionClass = new DirectionCalcClass();
    double speedCurrent;
    double lastOdoX;
    double lastOdoY;
    double positionErrorX;
    double positionErrorY;
    double distanceDelta;
    double timePrevious;
    double speedLastError;
    double speedError;
    double speedPorportional;
    double speedPM;
    double speedDerivative;
    double speedDM;
    public double speed;
    public double speedSetpoint;
    double speedMinimum = .001;

    public double SpeedCalc(double odoX, double odoY, double time, double speedsetpoint) {
        //Finds the difference between loop cycles in position
        positionErrorX = Math.abs(odoX - lastOdoX);
        positionErrorY = Math.abs(odoY - lastOdoY);
        //Finds the hypotenuse/distance traveled
        distanceDelta = Math.hypot(positionErrorX, positionErrorY);
        //Find the robots in/s
        speedCurrent = (distanceDelta) / (time - timePrevious);
        //Finds all of the next loop cycles values
        timePrevious = time;
        lastOdoX = odoX;
        lastOdoY = odoY;
        //Speed PD
        speedError = speedCurrent - speedsetpoint;
        speedPorportional = speedError * speedPM;
        speedDerivative = (speedError = speedLastError) * speedDM;
        speedLastError = speedError;
        //Speed at which the motor %'s will be going
        speed = speed + (speedDerivative + speedPorportional);
        if (speed <= .001){
            speed = .001;
        }
        return speed;
    }

    public double MotionProfile(double startpointX, double startpointY, double speedtarget, double accelerationdistance, double deccelerationdistance) {
        if (accelerationdistance <= DirectionClass.distance - DirectionClass.distanceFrom) {
            speedSetpoint = (DirectionClass.distance - DirectionClass.distanceFrom) * (speedtarget / accelerationdistance);
        }
        if (deccelerationdistance <= DirectionClass.distanceFrom) {
            speedSetpoint = DirectionClass.distanceFrom * (speedtarget / deccelerationdistance) + speedMinimum;
        }
    return speedSetpoint;
    }

}
