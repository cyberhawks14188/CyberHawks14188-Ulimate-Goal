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
    double speedPM = .00035;
    double speedDerivative;
    double speedDM = 0.0006;
    public double speed;
    public double speedSetpoint;
    double speedMinimum = 0;

    public void SpeedCalc(double odoX, double odoY, double time, double speedsetpoint) {
        if (time >= timePrevious + .75) {
            //Finds the difference between loop cycles in position
            positionErrorX = Math.abs(odoX - lastOdoX);
            positionErrorY = Math.abs(odoY - lastOdoY);
            lastOdoX = odoX;
            lastOdoY = odoY;
            //Finds the hypotenuse/distance traveled
            distanceDelta = Math.hypot(positionErrorX, positionErrorY);
            //Find the robots in/s
            speedCurrent = distanceDelta / (time - timePrevious);
            timePrevious = time;
        }

        //Finds all of the next loop cycles values

        //Speed PD
        speedError = speedsetpoint - speedCurrent;
        speedPorportional = speedError * speedPM;
        speedDerivative = (speedError - speedLastError) * speedDM;
        speedLastError = speedError;
        //Speed at which the motor %'s will be going
        speed = Math.abs(speed + ((speedDerivative + speedPorportional)));
        if (speed <= .11){
            speed = .11;
        }

        if(speed >= 1){
            speed = 1;
        }

    }
    public double SpeedReturn(){return speed;}
    public double CurrentSpeed(){return speedCurrent;}
    public double DistanceDelta(){return distanceDelta;}

    public double MotionProfile(double speedtarget, double accelerationdistance, double deccelerationdistance, double distance,  double distancefrom) {
        if (accelerationdistance > distance - distancefrom) {
            speedSetpoint = (distance - distancefrom) * (speedtarget / accelerationdistance);
        }
        else if (deccelerationdistance > distancefrom) {
            speedSetpoint = distancefrom * (speedtarget / deccelerationdistance);
        }
        else{
            speedSetpoint = speedtarget;
        }
    return speedSetpoint;
    }

}
