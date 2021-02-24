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
    double speedPM = .0001;
    double speedDerivative;
    double speedDM = .0001;
    public double speed;
    public double speedSetpoint;
    double speedMinimum = 0;

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
        speedError = speedsetpoint - speedCurrent;
        speedPorportional = speedError * speedPM;
        speedDerivative = (speedError - speedLastError) * speedDM;
        speedLastError = speedError;
        //Speed at which the motor %'s will be going
        speed = Math.abs(speed + (speedDerivative + speedPorportional));
        if (speed <= .00001){
            speed = .00001;
        }

        if(speed >= 1){
            speed = 1;
        }
        return speed;
    }
    public double CurrentSpeed(){return speedCurrent;}

    public double MotionProfile(double speedtarget, double accelerationdistance, double deccelerationdistance) {
        if (accelerationdistance <= DirectionClass.distance - DirectionClass.distanceFrom) {
            speedSetpoint = (DirectionClass.distance - DirectionClass.distanceFrom) * (speedtarget / accelerationdistance);
        }
        if (deccelerationdistance <= DirectionClass.distanceFrom) {
            speedSetpoint = DirectionClass.distanceFrom * (speedtarget / deccelerationdistance) + speedMinimum;
        }
    return speedSetpoint;
    }

}
