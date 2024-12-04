package org.firstinspires.ftc.teamcode.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.LowerArm;

public class LowArmToPositionCommand extends CommandBase {
    private final LowerArm lowerArm;
    private final double targetPosition;

    public LowArmToPositionCommand(LowerArm lowerArm, double targetPosition) {
        this.lowerArm = lowerArm;
        this.targetPosition = targetPosition;
        addRequirements(lowerArm);  // Declare subsystem dependencies
    }

    @Override
    public void initialize() {
        lowerArm.moveToPosition(targetPosition);
    }

    @Override
    public void execute() {
        lowerArm.moveToPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        // Command finishes when the arm is within a small tolerance of the target
        return Math.abs(targetPosition - lowerArm.getCurrentPosition()) < 5;
    }

    @Override
    public void end(boolean interrupted) {
        lowerArm.stopMotor();  // Stop the motor when the command ends
    }
}
