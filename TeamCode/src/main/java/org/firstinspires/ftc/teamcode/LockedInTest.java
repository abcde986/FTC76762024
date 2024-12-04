/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/* this is just to test if the motors are all connected and are working.
Running it will just make the robot launch forward. I wouldn't run it in front of a wall if i were you.
 */

/*@Autonomous(name = "MotorTest")
public class LockedInTest extends OpMode {

    DcMotor motor0;
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;

    // right motors: [front 3] [back 2]
    //back motors: [front 1] [back 0]

    @Override
    public void init() {
        LockedInHardware LockedInHardware = new LockedInHardware();

        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");

        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor3.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);

    }

    public void loop(){


        motor0.setPower(1);
        motor1.setPower(1);
        motor2.setPower(1);
        motor3.setPower(1);

    }
} penis*/