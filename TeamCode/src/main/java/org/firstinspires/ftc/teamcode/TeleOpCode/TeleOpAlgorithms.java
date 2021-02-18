package org.firstinspires.ftc.teamcode.TeleOpCode;

public class TeleOpAlgorithms {
    //Calls the variable method to use all variables
    TeleOpVariables TV = new TeleOpVariables();
    //Method to calculate the motor powers to go in what direction we want at the correct speed we want
    public double DriveBase(double x, double y, double z, boolean gpadRightBumper){//parameters to get gamepad buttons to figure out where we want to go
        //sets Varibales to parameters so we can output values to diffecent classes easily
        TV.x = x;
        TV.y = y;
        TV.z = z;
        TV.gamepadRightBumperState = gpadRightBumper;
        //Slows down drivebase if the right bumper is pressed for accurate movements if needed
        if (TV.gamepadRightBumperState) {
            TV.xSpeedSetPoint = .5;
        } else {
            TV.xSpeedSetPoint = 1;
        }
        //Calculates variables to let us drive in the correct direction
        TV.LFM = (-TV.y) - ((-TV.x) + (-TV.z));
        TV.LBM = (-TV.y) + ((-TV.x) - (-TV.z));
        TV.RFM = (-TV.y) + ((-TV.x) + (-TV.z));
        TV.RBM = (-TV.y) - ((-TV.x) - (-TV.z));
        //We use highest motor power to make sure no wheel speed ever goes over 1 and lets us strafe accuratly
        TV.highestMotorPower = Math.max(Math.max(Math.abs(TV.LFM), Math.abs(TV.LBM)), Math.max(Math.abs(TV.RFM), Math.abs(TV.RBM)));
        //Since we use highest motor power to make sure no motor power goes over 1, there will always be a motor going full power
        //We use the joysticks again to make apply a ratio and the set motor power
        TV.leftG1StickPoint = Math.sqrt((TV.x * TV.x) + (TV.y * TV.y));
        if (Math.abs(TV.z) < .01) {
            TV.speed = TV.leftG1StickPoint;
        } else if (Math.abs(TV.x + TV.y) < .02) {
            TV.speed = Math.abs(TV.z);
        } else {
            TV.speed = (TV.leftG1StickPoint + Math.abs(TV.z)) / 2;
        }
        //Uses the calculations above to calculate the desired motor power
        TV.LFM = ((TV.LFM / TV.highestMotorPower) * TV.speed) * TV.xSpeedSetPoint;
        TV.LBM = ((TV.LBM / TV.highestMotorPower) * TV.speed) * TV.xSpeedSetPoint;
        TV.RFM = ((TV.RFM / TV.highestMotorPower) * TV.speed) * TV.xSpeedSetPoint;
        TV.RBM = ((TV.RBM / TV.highestMotorPower) * TV.speed) * TV.xSpeedSetPoint;
        return TV.LFM;//returns motor power variables
        }
        public double LBMset(){return TV.LBM;}
        public double RFMset(){return TV.RFM;}
        public double RBMset(){return TV.RBM;}
//calculates odometry to let us know where we are in the feild at all times
    public double OdometryCalc(double e1, double e2, double e3){
        //sets paramters to variables for easy use
        TV.e1current = e1;
        TV.e2current = e2;
        TV.e3current = e3;
        //finds the change in the encoders from the last loop cycle
        TV.deltaE1 = TV.e1current - TV.e1Previous;
        TV.deltaE2 = TV.e2current - TV.e2Previous;
        TV.deltaE3 = TV.e3current - TV.e3Previous;
        //calculates the angle in radians
        TV.thetaChange = (TV.deltaE1 - TV.deltaE3)/ TV.encoderWheelDistanceInch;
        TV.thetaInRadians = TV.thetaInRadians + TV.thetaChange;
        //calculates the e2 reading without the angle values affecting in it
        TV.e2WithOffSet = TV.deltaE2 - (TV.thetaChange * TV.e2XOffSet);
        //finds the average between the forward facing encoders to find the center of the robot
        TV.yAverage = (TV.deltaE1 + TV.deltaE3)/2;

        TV.yCoordinatePosition = TV.yCoordinatePosition + (TV.yAverage*Math.sin(TV.thetaInRadians)) + (TV.e2WithOffSet*Math.cos(TV.thetaInRadians));
        TV.xCoordinatePosition = TV.xCoordinatePosition + (TV.yAverage*Math.cos(TV.thetaInRadians)) - (TV.e2WithOffSet*Math.sin(TV.thetaInRadians));
        //sets varibles to the current encoder reading to set Delta varibles in the next loop cycle
        TV.e1Previous = TV.e1current;
        TV.e2Previous = TV.e2current;
        TV.e3Previous = TV.e3current;
        //returns the values to use in other classes
        return TV.xCoordinatePosition/TV.COUNTS_PER_INCH;
    }
    public double odoy(){return TV.yCoordinatePosition/TV.COUNTS_PER_INCH;}
    public double thetaINRadians(){return TV.thetaInRadians;}
    //Method uses sensores and gamepad to decide wether the intake or stager motor should be on and what position to set the stopper servo to
    public double RingSystem(boolean Astate, boolean Bstate, double cs1, double cs2, double cs3, boolean gpadback){
        //sets variables to the parameters for easy use in the method
        TV.gamepadAState = Astate;
        TV.gamepadBState = Bstate;
        TV.ring1Sensor = cs1;
        TV.ring2Sensor = cs2;
        TV.ring3Sensor = cs3;
        TV.gamepadBackState = gpadback;
        //uses a Finite State Machine to turn the intake and stager motors on or off and set the position of the stopper servo
        //to set what state the FSM is in, we use our 1 button function, The function uses a boolean to tell us if the button was pressed last loop cycle
        //if the button was't and it is now: change the state we are in, Then reapeat until the program shuts off
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
        if (TV.gamepadBackState) {
            TV.intakePower = 1;
        }//returns the variables for the the intake and stager power and the stopper position
        return TV.intakePower;
    }
    public double stagerP(){return TV.stagerPower;}
    public double stopperSet(){return TV.stopper;}
    // Shooter control method sets the shooter flywheel speed, runs the PID to make sure the flywheel is at the correct speed and adjusts the angle of the shooter
    public double shooterControl(boolean dpadleft, boolean dpadright, boolean gpady, boolean gpadx, boolean leftbutton, double sotmcurrent, double timePassed, double sotacurrent){
        //sets variables to the parameters for easy use in the method
        TV.gamepadDpadLeftState = dpadleft;
        TV.gamepadDpadRightState = dpadright;
        TV.gamepadYState = gpady;
        TV.gamepadXState = gpadx;
        TV.gamepadLeftBumperState = leftbutton;
        TV.SOTCurrentEncoder = sotmcurrent;
        TV.timepassed = timePassed;
        TV.SOTCurrent = sotacurrent;
        //uses Dpad left and right to be able to set the
        if (TV.gamepadDpadLeftState) {
            TV.SOTSet = 1.645;
        } else if (TV.gamepadDpadRightState) {
            TV.SOTSet = 1.27;//TOP GOAL
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
        TV.SOTPower = (TV.SOTError * TV.SOTP);
        //Flywheel speed setpoint control. We use our custom one button on/off system to use the left bumper to set the shooter speed.
        if (TV.gamepadLeftBumperState && !TV.shooterControl) {
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
        return -((TV.shooterSetpoint+TV.shooterCorrection)/2800);
    }
    public double sotaset(){ return TV.SOTPower; }
    public double WobbleControl(double gpadlefttrigger, boolean dpadup, boolean dpaddown, double wbpt){
        TV.gamepadLeftTrigger = gpadlefttrigger;
        TV.gamepadDpadUpState = dpadup;
        TV.gamepadDpadDownState = dpaddown;
        TV.wobbleCurrent = wbpt;
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
        return TV.wobblePower;
    }
    public double grip(){return TV.GRIP_S;}
}
