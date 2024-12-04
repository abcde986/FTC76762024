package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HighArm extends SubsystemBase {

    public DcMotor secondLevelArm;




    public HighArm(HardwareMap ahwMap){

        //REV UltraPlanetary Hex Motor, the lowest and biggest one on the arm
        secondLevelArm = ahwMap.get(DcMotor.class, "secondLevelArm");



    }

    public void setPower (double power){
        secondLevelArm.setPower(power);
    }
}
