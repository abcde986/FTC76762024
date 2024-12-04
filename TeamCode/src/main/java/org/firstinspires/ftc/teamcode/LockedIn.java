/*package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Locked IN (pushbot (not anymore) teleop)")

public class LockedIn extends OpMode {

    private LockedInHardware robot = new LockedInHardware();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);


    }

    @Override
    public void loop() {

      //  double clawPosition = robot.peanitzOverdrive.getCurrentPosition();


        double lowerArmPosition = robot.thrustMaster.getCurrentPosition();

      //  telemetry.addData("arm encoder", "encoder: " + )


        telemetry.addData("LockedInStatus", "Run Time: " + runtime.toString());

        //claw position
     //   telemetry.addData("Claw Position", "Encoder:" + clawPosition);

        telemetry.addData("Thrust Master" , "Encoder:" + lowerArmPosition);

        //Driver 1
        robot.simpleDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        //Driver 2
        //move the arm down
        if (gamepad2.b) {
            robot.lowerArm(-1.5);
        } else if (!gamepad2.b) {
            robot.lowerArm(0);
        }

        //to make the arm hold do 0.5
        //move arm up
        if (gamepad2.a) {
            robot.liftArm(1.5);

        } else if (!gamepad2.a) {
            robot.lowerArm(0);
        }

        //move second level up
        if (gamepad2.right_trigger > 0.2) {
            // Move second level up
            robot.secondLevelArm(1);
            telemetry.addData("Second Arm", "Movement: going up");
        } else if (gamepad2.left_trigger > 0.2) {
            // Move second level down
            robot.secondLevelArm(-1); // Negative for down movement
            telemetry.addData("Second Arm", "Movement: going down");
        } else {
            // Stop the arm if no trigger is pressed
            robot.secondLevelArm(0);
            telemetry.addData("Second Arm", "Movement: stagnant");
        }

        //right bumper on gamepad2 opens claw while left bumper closes it then a safety clause to make sure the claw doesn't continuously move
     /*   if (gamepad2.right_bumper) {
            robot.clawOpen();
        } else if (gamepad2.left_bumper) {
            robot.clawClose();
        } else {
            robot.stopClaw();
        } **/

       /* if (gamepad2.y) {
            robot.clawOpen();
        } else if (!gamepad2.y) {
            robot.clawClose();
          //  robot.stopClaw();
        }*/

        /* if (gamepad2.y) {
            robot.clawOpen(false);
        }else if (!gamepad2.y){
            robot.clawOpen(true);
        } */

        /*if (gamepad2.dpad_up) {
            robot.shootLoad();
        } else if (gamepad2.dpad_down) {
            robot.takeInLoad();
        } else {
            robot.stopLoad();

        }

        /*if (gamepad2.dpad_down) {
            robot.takeInLoad();
        } else if (!gamepad2.dpad_down) {
            robot.stopLoad();
        }*/

