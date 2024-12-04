package org.firstinspires.ftc.teamcode.Subsystems;

import android.icu.text.CaseMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.LockedInHardware;

public class Arm extends SubsystemBase {

    private HighArm highArm;
    private LowerArm lowerArm;



    public Arm(HardwareMap ahwMap) {
        highArm = new HighArm(ahwMap);
        lowerArm = new LowerArm(ahwMap);


    }

    public void bothArmUp(){
        highArm.setPower(1.0);
        lowerArm.setPower(1.0);
    }

    public void bothArmDown(){
        highArm.setPower(-1.0);
        lowerArm.setPower(-1.0);
    }


}
