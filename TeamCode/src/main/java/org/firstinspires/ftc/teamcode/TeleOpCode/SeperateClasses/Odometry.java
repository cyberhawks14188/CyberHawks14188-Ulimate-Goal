package org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses;

public class Odometry {
    double deltaE1, deltaE2, deltaE3;
    double e1Previous, e2Previous, e3Previous;
    final double encoderWheelDistance = 14.42206;
    final double COUNTS_PER_INCH = 236.5;
    final double encoderWheelDistanceInch = encoderWheelDistance * COUNTS_PER_INCH;
    double thetaChange, thetaInRadians;
    double e2WithOffSet;
    final double e2XOffSet = 1815.7032;
    double yAverage;
    double yCoordinatePosition, xCoordinatePosition;
    double e1Current, e2Current, e3Current;

    public void OdometryCalc(double e1current, double e2current, double e3current){
        //finds the change in the encoders from the last loop cycle
        e1Current = e1current;
        e2Current = e2current;
        e3Current = e3current;
        deltaE1 = e1Current - e1Previous;
        deltaE2 = e2Current - e2Previous;
        deltaE3 = e3Current - e3Previous;
        //calculates the angle in radians
        thetaChange = (deltaE1 - deltaE3)/ encoderWheelDistanceInch;
        thetaInRadians = thetaInRadians + thetaChange;
        //calculates the e2 reading without the angle values affecting in it
        e2WithOffSet = deltaE2 - (thetaChange * e2XOffSet);
        //finds the average between the forward facing encoders to find the center of the robot
        yAverage = (deltaE1 + deltaE3)/2;

        yCoordinatePosition = yCoordinatePosition + (yAverage*Math.sin(thetaInRadians)) + (e2WithOffSet*Math.cos(thetaInRadians));
        xCoordinatePosition = xCoordinatePosition + (yAverage*Math.cos(thetaInRadians)) - (e2WithOffSet*Math.sin(thetaInRadians));
        //sets varibles to the current encoder reading to set Delta variables in the next loop cycle
        e1Previous = e1Current;
        e2Previous = e2Current;
        e3Previous = e3Current;
    }
    //returns the values to use in other classes
    public double odoXReturn(){return xCoordinatePosition/COUNTS_PER_INCH;}
    public double odoYReturn(){return yCoordinatePosition/COUNTS_PER_INCH;}
    public double thetaINRadiansReturn(){return thetaInRadians;}
    public double thetaInDegreesReturn(){return Math.toDegrees(thetaInRadians % 360);}
}
//        LFM = (-y) - ((-x) + (-z));
//        LBM = (-y) + ((-x) - (-z));
//        RFM = (-y) + ((-x) + (-z));
//        RBM = (-y) - ((-x) - (-z));
//        //We use highest motor power to make sure no wheel speed ever goes over 1 and lets us strafe accuratly
//        highestMotorPower = Math.max(Math.max(Math.abs(LFM), Math.abs(LBM)), Math.max(Math.abs(RFM), Math.abs(RBM)));
// speed = ((setspeed - actualspeed)*Proportionalmultiplier) + speed;
//if speed>1{
//speed =1;
//}
//       LFMtoMotor = LFM * speed;

//if(Farther than 1 in away){
//        setpointX = ____;
//        setpointY = ____;
//        slope = setpointY/setpointX;
//        currentXSet = Math.sin(slope);
//        CurrentYSet = Math.cos(slope);
//}else{
//currentXSet = setpointX;
//currentXSet = setpointY;
//}

