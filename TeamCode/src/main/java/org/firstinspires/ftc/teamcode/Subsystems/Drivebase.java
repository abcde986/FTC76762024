package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class

Drivebase extends SubsystemBase {

    MotorEx leftFront, rightFront, leftBack, rightBack;

    MecanumDrive mecanum;

    //drivebase
    //right motors: [front 3] [back 2]
    //left motors: [front 1] [back 0]
    public  Drivebase (HardwareMap ahwMap){
        leftFront = new MotorEx(ahwMap, "motor3");
        rightFront = new MotorEx(ahwMap, "motor2");
        leftBack = new MotorEx(ahwMap, "motor0");
        rightBack = new MotorEx(ahwMap, "motor1"); //define motors

        leftFront.setInverted(true);
        rightFront.setInverted(true);
        leftBack.setInverted(true);
        rightBack.setInverted(true); //motor directions

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE); //motor configs

        //frontLeft.set;

        mecanum = new MecanumDrive(leftFront, rightFront, leftBack, rightBack); //using class to make command work or smt idk
    }

    public void driveRobotCentric(double forwardSpeed, double strafeSpeed, double turnSpeed){
        mecanum.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

    public void driveFieldCentric(double forwardSpeed, double strafeSpeed, double turnSpeed, double angle){
        mecanum.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, angle);
    }

    public void setMotorSpeed(double frontLeft, double frontRight, double backLeft, double backRight) {
        mecanum.driveWithMotorPowers(frontLeft, frontRight, backLeft, backRight);
    }
}
