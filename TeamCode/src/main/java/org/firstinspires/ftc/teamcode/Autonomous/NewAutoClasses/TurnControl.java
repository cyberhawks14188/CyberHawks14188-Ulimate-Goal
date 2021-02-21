package org.firstinspires.ftc.teamcode.Autonomous.NewAutoClasses;

public class TurnControl {
    double thetaError;
    double thetaProportionalMultiplier = 1;
    double thetaProportional;
    double thetaToEquation;
    double thetaSetPoint = 0;
    double thetaLastError = 0;
    double thetaDerivativeMultiplier = 1;
    double thetaDerivative;
    public void turnControl(double thetaendsetpoint, double thetaindegrees, double turnincrements){
        if (thetaendsetpoint > thetaSetPoint + 1){
            thetaSetPoint = thetaSetPoint + turnincrements;
        }else if(thetaendsetpoint < thetaSetPoint - 1){
            thetaSetPoint = thetaSetPoint - turnincrements;
        }
        thetaError = thetaSetPoint - thetaindegrees;
        thetaDerivative = (thetaError - thetaLastError)* thetaDerivativeMultiplier;
        thetaLastError = thetaError;
        thetaProportional = thetaError * thetaProportionalMultiplier;
        thetaToEquation = thetaProportional + thetaDerivative;
    }
}
