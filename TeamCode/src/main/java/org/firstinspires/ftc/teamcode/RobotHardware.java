
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.TouchSensor

public class RobotHardware{

    // Motors and Servos
    public DcMotor D_LF_M;
    public DcMotor D_LB_M;
    public DcMotor D_RF_M;
    public DcMotor D_RB_M;


    //Create Hardware map
    HardwareMap hardwareMap;

    public void init(HardwareMap hardwareMap) {


        // Define motors and servos
        D_LF_M = hardwareMap.get(DcMotor.class, "D_LF_M");
        D_LB_M = hardwareMap.get(DcMotor.class, "D_LB_M");
        D_RF_M = hardwareMap.get(DcMotor.class, "D_RF_M");
        D_RB_M = hardwareMap.get(DcMotor.class, "D_RB_M");

        //servo = hardwareMap.get(Servo.class, "servo");

        // Set all motors to zero power

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        D_LF_M.setMode(DcMotor.RunMode.RESET_ENCODERS);
        D_LB_M.setMode(DcMotor.RunMode.RESET_ENCODERS);
        D_RF_M.setMode(DcMotor.RunMode.RESET_ENCODERS);
        D_LF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        D_LB_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        D_RF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Zero power Behavor
        D_LF_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //or FLOAT
        D_LB_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        D_RF_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        D_RB_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        D_LF_M.getCurrentPosition();
        D_LB_M.getCurrentPosition();
        D_RF_M.getCurrentPosition();
        // Define and initialize ALL installed servos.
        //servo.setPosition(0);
    }
}
