package org.firstinspires.ftc.teamcode.TeleOpCode.SeperateClasses;

public class Odometry {
    double deltaE1, deltaE2, deltaE3;
    double e1Previous, e2Previous, e3Previous;
    final double encoderWheelDistance = 14.300650233564467;
    final double COUNTS_PER_INCH = 239;
    final double encoderWheelDistanceInch = encoderWheelDistance * COUNTS_PER_INCH;
    double thetaChange, thetaInRadians;
    double e2WithOffSet;
    final double e2XOffSet = 1983.9474445737487;
    double yAverage;
    double yCoordinatePosition, xCoordinatePosition;

    public void OdometryCalc(double e1current, double e2current, double e3current){
        //finds the change in the encoders from the last loop cycle
        deltaE1 = e1current - e1Previous;
        deltaE2 = e2current - e2Previous;
        deltaE3 = e3current - e3Previous;
        //calculates the angle in radians
        thetaChange = (deltaE1 - deltaE3)/ encoderWheelDistanceInch;
        thetaInRadians = thetaInRadians + thetaChange;
        //calculates the e2 reading without the angle values affecting in it
        e2WithOffSet = deltaE2 - (thetaChange * e2XOffSet);
        //finds the average between the forward facing encoders to find the center of the robot
        yAverage = (deltaE1 + deltaE3)/2;

        yCoordinatePosition = yCoordinatePosition + (yAverage*Math.sin(thetaInRadians)) + (e2WithOffSet*Math.cos(thetaInRadians));
        xCoordinatePosition = xCoordinatePosition + (yAverage*Math.cos(thetaInRadians)) - (e2WithOffSet*Math.sin(thetaInRadians));
        //sets varibles to the current encoder reading to set Delta varibles in the next loop cycle
        e1Previous = e1current;
        e2Previous = e2current;
        e3Previous = e3current;
    }
    //returns the values to use in other classes
    public double odoXReturn(){return xCoordinatePosition/COUNTS_PER_INCH;}
    public double odoYReturn(){return yCoordinatePosition/COUNTS_PER_INCH;}
    public double thetaINRadiansReturn(){return thetaInRadians;}
}
