package org.firstinspires.ftc.teamcode.TeleOpCode;

public class TeleOpAlgorithms {
    TeleOpVariables TV = new TeleOpVariables();
    public void DriveBase(){

        //sets calculation variables to let us calculate drivetrain direction
        //calculates motor speed and direction
        TV.LFM = (-TV.y) - ((-TV.x) + (-TV.z));
        TV.LBM = (-TV.y) + ((-TV.x) - (-TV.z));
        TV.RFM = (-TV.y) + ((-TV.x) + (-TV.z));
        TV.RBM = (-TV.y) - ((-TV.x) - (-TV.z));
        //We use highest motor power to make sure no wheel speed ever goes over 1 and lets su strafe more accuratly
        TV.highestMotorPower = Math.max(Math.max(Math.abs(TV.LFM), Math.abs(TV.LBM)), Math.max(Math.abs(TV.RFM), Math.abs(TV.RBM)));
        TV.leftG1StickPoint = Math.sqrt((TV.x * TV.x) + (TV.y * TV.y));
        //Since we use highest motor power to make sure no motor power goes over 1, there will always be a motor going full power
        //So we use the joysticks again to make apply a ratio and the set motor power
        if (Math.abs(TV.z) < .01) {
            TV.speed = TV.leftG1StickPoint;
        } else if (Math.abs(TV.x + TV.y) < .02) {
            TV.speed = Math.abs(TV.z);
        } else {
            TV.speed = (TV.leftG1StickPoint + Math.abs(TV.z)) / 2;
        }
    }
    public void OdometryCalc(){
        TV.deltaE1 = TV.e1current - TV.e1Previous;
        TV.deltaE2 = TV.e2current - TV.e2Previous;
        TV.deltaE3 = TV.e3current - TV.e3Previous;

        TV.thetaChange = (TV.deltaE1 - TV.deltaE3)/ TV.encoderWheelDistanceInch;
        TV.thetaInRadians = TV.thetaInRadians + TV.thetaChange;

        TV.e2WithOffSet = TV.deltaE2 - (TV.thetaChange * TV.e2XOffSet);

        TV.yAverage = (TV.deltaE1 + TV.deltaE3)/2;

        TV.yCoordinatePosition = TV.yCoordinatePosition + (TV.yAverage*Math.sin(TV.thetaInRadians)) + (TV.e2WithOffSet*Math.cos(TV.thetaInRadians));
        TV.xCoordinatePosition = TV.xCoordinatePosition + (TV.yAverage*Math.cos(TV.thetaInRadians)) - (TV.e2WithOffSet*Math.sin(TV.thetaInRadians));

        TV.e1Previous = TV.e1current;
        TV.e2Previous = TV.e2current;
        TV.e3Previous = TV.e3current;
    }
    public void RingSystem(){
        if (TV.gamepadAState && !TV.stagerControl) {
            if (TV.intakePower == 0) {
                TV.shooterFSM = 1;
            } else {
                TV.shooterFSM = 2;
            }
            TV.stagerControl = true;
        } else if (!TV.gamepadAState) {
            TV.stagerControl = false;
        }
        //stage 0 shuts all motors off and this stager is starting stage
        if (TV.shooterFSM == 0) {
            TV.intakePower = 0;
            TV.stagerPower = 0;
        }
        //stage 1 is intaking stage. Sets ring stopper to closed and intakes until sensors see that there is 3 rings in robot.
        if (TV.shooterFSM == 1) {
            TV.stopper = .3;
            if (TV.ring1Sensor > .55 && TV.ring2Sensor > .55 && TV.ring3Sensor > .55) {
                TV.intakePower = 0;
                TV.stagerPower = 0;
                TV.shooterFSM = 2;
            } else {
                TV.intakePower = -1;
                TV.stagerPower = -1;
            }
        }
        //stage 2 is shooting stage. If button b is pressed rings shoot other wise motors are off
        if (TV.shooterFSM == 2) {
            if (TV.gamepadBState) {
                TV.stopper = .5;
                TV.stagerPower = -.9;
            } else {
                TV.stagerPower = 0;
            }
            TV.intakePower = 0;
        }

    }
    public void shooterControl(){
        if (TV.gamepadDpadLeftState) {
            TV.SOTSet = 1.645;
        } else if (TV.gamepadDpadRightState) {
            TV.SOTSet = 1.47;//TOP GOAL
        }
        //Shooter Control
        //Manual adjusting the setpoint to adjust last second if needed
        if (TV.gamepadYState) {
            TV.SOTSet = TV.SOTSet - .003;
        } else if (TV.gamepadXState) {
            TV.SOTSet = TV.SOTSet + .003;
        }

        //Shooter Angle PID Loop follo the setpoint set above
        TV.SOTError = TV.SOTSet - TV.SOTCurrent;
        TV.SOTPower = TV.SOTError * TV.SOTP;
        //Flywheel speed setpoint control. We use our custom one button on/off system to use the left bumper to set the shooter speed.
        if (TV.gamepadDpadLeftState && !TV.shooterControl) {
            if (TV.shooterSetpoint == 0) {
                TV.shooterSetpoint = 1900;//set point is 1900 encoder ticks per second

            } else {
                TV.shooterSetpoint = 0;
                TV.shooterCorrection = 0;//we set both of these variables to ensure that neither one has power
            }
            TV.shooterControl = true;
        } else if (!TV.gamepadLeftBumperState) {
            TV.shooterControl = false;
        }
        //This is thhe Flywheels PID. This makes sure the Shotoer speed is the same no matter the battery power
        if (TV.shooterSetpoint != 0) {
            TV.shooterPM = 15;//set the proportional multiplier for the shooter speed
            TV.timepassed = TV.currentRunTime - TV.lastTime;
            TV.shooterActualVelocity = Math.abs(TV.SOTCurrentEncoder - TV.shooterLastEncoder) / TV.timepassed;
            TV.lastTime = TV.currentRunTime;
            TV.shooterLastEncoder = TV.SOTCurrentEncoder;
            TV.shooterError = TV.shooterSetpoint - TV.shooterActualVelocity;
            TV.shooterPorportional = TV.shooterError * TV.shooterPM;
            TV.shooterCorrection = TV.shooterPorportional;
        }
    }
    public void WobbleControl(){
        //finite State machine for the wobble goal.
        //We use the custom one button cycle to switch between each state.
        if (TV.gamepadLeftTrigger > .05 && !TV.WBControl) {
            if (TV.WB_FSM < 4) {
                TV.WB_FSM = TV.WB_FSM + 1;
            } else {
                TV.WB_FSM = 0;
            }
            TV.WBControl = true;
        } else if (TV.gamepadLeftTrigger < .05) {
            TV.WBControl = false;
        }
        //State 0 is claw open and in grabbing position
        if (TV.WB_FSM == 0) {
            TV.wobbleEndSet = 2.324;
            TV.GRIP_S = .1;
        } else if (TV.WB_FSM == 1) {//state 1 is grab wobble goal while still in down position
            TV.GRIP_S = .7;
        } else if (TV.WB_FSM == 2) {//Brings wobble goal arm in stored position to drive to wall
            TV.wobbleEndSet = .6;
        } else if (TV.WB_FSM == 3) {//Brings wobble goal above wall, gripper still closed
            TV.wobbleEndSet = 1;
        } else if (TV.WB_FSM == 4) {// Gripper opens
            TV.GRIP_S = .1;
        } else if (TV.WB_FSM == 5) {//Closes Claw
            TV.GRIP_S = .7;
        }
        //Lets up manually change set point if needed. Set EndSet to current set point + or - t o allow the arm to go to its current position + or - the setpoint.
        if (TV.gamepadDpadUpState) {
            TV.wobbleEndSet = TV.wobbleSet - .1;
        } else if (TV.gamepadDpadDownState) {
            TV.wobbleEndSet = TV.wobbleSet + .1;
        }
        //This slowly brings the arm to the endsetpoint to ensure little wear from arm bannging on the wheels
        if (TV.wobbleEndSet > .2 + TV.wobbleSet) {
            TV.wobbleSet = TV.wobbleSet + .05;
        } else if (TV.wobbleEndSet < .2 - TV.wobbleSet) {
            TV.wobbleSet = TV.wobbleSet - .05;
        } else {
            TV.wobbleSet = TV.wobbleEndSet;
        }
        //PID to control the wobble arm to go to desired set point
        TV.wobbleError = TV.wobbleSet - TV.wobbleCurrent;
        TV.wobblePower = TV.wobbleError * TV.wobbleP;
    }
}
