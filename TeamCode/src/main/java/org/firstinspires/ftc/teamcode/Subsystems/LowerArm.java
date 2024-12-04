package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LowerArm extends SubsystemBase {
    private final DcMotor thrustMaster;
    private final PIDController pidController;

    // Constants for PID control (tune these values)
    private static final double kP = 1.0;  // Proportional constant (tune this value)
    private static final double kI = 0.0;   // Integral constant
    private static final double kD = 0.1; // Derivative constant

    // Maximum and minimum positions (in encoder ticks)
    private static final int MAX_POSITION = 1000;
    private static final int MIN_POSITION = 0;

    // Target position for the arm
    private double targetPosition;
    private boolean holdPosition = false;

    public LowerArm(HardwareMap ahwMap) {
        thrustMaster = ahwMap.get(DcMotor.class, "thrustMaster");
        thrustMaster.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thrustMaster.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize the PID Controller with the constants
        pidController = new PIDController(kP, kI, kD);

        // Set initial target position to current position
        targetPosition = getCurrentPosition();
    }

    public int getCurrentPosition() {
        return thrustMaster.getCurrentPosition();
    }

    public void moveToPosition(double newTargetPosition) {
        // Ensure target is within limits
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, newTargetPosition));
    }

    public void stopMotor() {
        thrustMaster.setPower(0);
    }

    public void setPower(double power) {
        // If manual control is used, update the target position accordingly
        if (power != 0) {
            holdPosition = false;
            targetPosition = getCurrentPosition(); // Update target to prevent sudden jerks
            thrustMaster.setPower(power);
        } else {
            targetPosition = getCurrentPosition(); // Update target to prevent sudden jerks
            holdPosition = true;


            holdPosition(); // Engage PID control to hold position when no input
        }
    }
    //

    private void holdPosition() {
        // Calculate the PID output to maintain the arm position
        double currentPosition = getCurrentPosition();
        //number= constant number and angle can be found from encoder, encoderOffSet = how much needed to add to the encoder to make it 0
        double pidOutput = pidController.calculate(currentPosition, targetPosition) + number * Math.cos(angle + encoderOffset);


        // Clamp the output to [-1.0, 1.0] and set motor power
        double power = Math.max(Math.min(pidOutput, 1.0), -1.0);
        thrustMaster.setPower(power);
    }


    @Override
    public void periodic() {
        // Continuously hold the arm's position in the background
        if(holdPosition) {
            holdPosition();
        }
    }
}
