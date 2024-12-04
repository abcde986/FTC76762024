/* package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.CRServoImpl;

// this class names the motors.
public class
LockedInHardware {

    //drivebase
    public DcMotor backLeft = null;
    public DcMotor frontLeft = null;
    public DcMotor backRight = null;
    public DcMotor frontRight = null;
    public DcMotor thrustMaster = null;
    public DcMotor jorkinator5000 = null;
    public DcMotor peanitzOverdrive = null;
    public Servo loadShooter5000 = null;




    public void init(HardwareMap ahwMap) {

        //drivebase
        //right motors: [front 3] [back 2]
        //left motors: [front 1] [back 0]

        backLeft = ahwMap.get(DcMotor.class, "motor0");
        frontLeft = ahwMap.get(DcMotor.class, "motor1");
        backRight = ahwMap.get(DcMotor.class, "motor2");
        frontRight = ahwMap.get(DcMotor.class, "motor3");

        //set direction of the drivebase motors
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);


        //handle encoders of drivebase motors

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //REV UltraPlanetary Hex Motor, the lowest and biggest one on the arm
        thrustMaster = ahwMap.get(DcMotor.class, "thrustMaster");
        //REV Core Hex Motor, second motor on arm
        jorkinator5000 = ahwMap.get(DcMotor.class, "jorkinator5000");
        //REV Core Hex Motor #2, claw motor
        peanitzOverdrive = ahwMap.get(DcMotor.class, "peanitzOverdrive");

        //Continuous Rotation Servo for sweeper intake/outtake
        loadShooter5000 = ahwMap.get(Servo.class, "loadShooter5000");

    }


    //Stolen from Branson!! drivebase control
     public void simpleDrive(double joystick1X, double joystick1Y, double joystick2X) {
        // Reverse the X axis for the joystick input
        double y = -joystick1Y; // Forward/backward
        double x = -joystick1X * 1; // Left/right (strafing)
        double rx = -joystick2X; // Rotation

        // Calculate power for each wheel
        double frontLeftPower = y + x + rx;   // Front Left Wheel
        double backLeftPower = y - x + rx;    // Back Left Wheel
        double frontRightPower = y - x - rx;  // Front Right Wheel
        double backRightPower = y + x - rx;   // Back Right Wheel

        // Normalize the wheel powers if necessary
        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(backLeftPower),
                        Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

        double scalingFactor = 0.7;

        if (maxPower > 1) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeftPower *= scalingFactor;
        backLeftPower *= scalingFactor;
        frontRightPower *= scalingFactor;
        backRightPower *= scalingFactor;

        // Set wheel powers to motors (pseudo code)
         frontLeft.setPower(frontLeftPower);
         backLeft.setPower(backLeftPower);
         frontRight.setPower(frontRightPower);
         backRight.setPower(backRightPower);
    }

    public void simpleDrive(double leftStickX, double leftStickY, double rightStickX){

        double x = leftStickX;
        double y = leftStickY;
        double turn = rightStickX;

        double theta = Math.atan2(y,x);
        double  power = Math.hypot(x,y);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double frontLeftPower = power * cos/max + turn;
        double frontRightPower = power * sin/max - turn;
        double backLeftPower = power* sin/max + turn;
        double backRightPower = power * cos/max - turn;

        if ((power + Math.abs(turn)) >1 ){
            frontLeftPower /= power + turn;
            frontRightPower /= power + turn;
            backLeftPower /= power + turn;
            backRightPower /= power + turn;

        }

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

    }


    public void liftArm (double power){
        thrustMaster.setPower(power);
    }

    public void lowerArm (double power){
        thrustMaster.setPower(power);
    }

    public void secondLevelArm (double power) {
        jorkinator5000.setPower(power);
    }


    public void clawOpen() {
        peanitzOverdrive.setPower(0.8); // Full power to open
    }

    public void clawClose() {
        peanitzOverdrive.setPower(-0.8); // Full power to close
    }

    public void stopClaw() {
        peanitzOverdrive.setPower(0);
    }


    public void shootLoad (){
        loadShooter5000.getController().pwmEnable();
        loadShooter5000.setPosition(1);
    }

    public void stopLoad() {
        loadShooter5000.getController().pwmDisable();
    }

    public void takeInLoad (){
        loadShooter5000.getController().pwmEnable();
        loadShooter5000.setPosition(-1);
    }




    /*public void stopShootLoad(){
        loadShooter5000.getController();
    }*\


    /* public void moveClaw(){
        peanitzOverdrive.setPower(0.1);
        peanitzOverdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    } */


  /*  public void clawOpen (boolean open) {

        if (open == true) {
            peanitzOverdrive.setTargetPosition(0);
        }  else if (open == false){
            peanitzOverdrive.setTargetPosition(1);

        }
        peanitzOverdrive.setPower(0.1);
        peanitzOverdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    } */




