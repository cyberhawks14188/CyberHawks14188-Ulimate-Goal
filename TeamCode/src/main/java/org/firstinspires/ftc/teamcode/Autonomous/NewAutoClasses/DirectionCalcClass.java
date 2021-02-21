package org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses;

public class DirectionCalcClass {
    double distance;
    double distanceFrom;
    double distanceYLeg;
    double distanceXLeg;
    double distanceFromEndY;
    double distanceFromEndX;
    double LF_M_Direction;
    double LB_M_Direction;
    double RF_M_Direction;
    double RB_M_Direction;
    public void DirectionCalc(double startpointx, double startpointy, double endpointx, double endpointy, double odoX, double odoY){
        distanceYLeg = (startpointx - endpointx);
        distanceXLeg = (startpointy - endpointy);
        distance = Math.hypot(distanceXLeg, distanceYLeg);
        distanceFromEndY = odoY - endpointy;
        distanceFromEndX = odoX - endpointx;
        distanceFrom = Math.hypot(distanceXLeg, distanceFromEndY);
        //Y PD


        //X PD


        LF_M_Direction = y + (x + z);
        LB_M_Direction = y - (x - z);
        RF_M_Direction = y - (x + z);
        RB_M_Direction = y + (x - z);
    }
    public double distanceReturn(){return distance;}
    public double distanceFromReturn(){return distanceFrom;}
}
