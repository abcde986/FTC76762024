package org.firstinspires.ftc.teamcode.Subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController.PwmStatus;
public class InOuttake extends SubsystemBase {
    private final Servo loadShooter5000;

    public InOuttake(HardwareMap ahwMap) {
        loadShooter5000 = ahwMap.get(Servo.class, "loadShooter5000");
    }

    public enum GripperState {
        Gripping,
        Idle,
        Opening
    }

    GripperState gripperState = GripperState.Idle;

    // Method to set the servo position
    public void setPosition(double position) {
        loadShooter5000.setPosition(position);
    }

    // Method to get the servo's current position
    public double getPosition() {
        return loadShooter5000.getPosition();
    }

    public void increaseGrip() {
        switch (gripperState) {
            case Gripping:
                break;
            case Idle:
                gripperState = GripperState.Gripping;
                setPosition(1);
                break;
            case Opening:
                gripperState = GripperState.Idle;
                setPosition(0.5);
                break;
        }
    }


    public void decreaseGrip() {
        switch (gripperState) {
            case Opening:
                break;
            case Idle:
                gripperState = GripperState.Opening;
                setPosition(0);
                break;
            case Gripping:
                gripperState = GripperState.Idle;
                setPosition(0.5);
                break;
        }
    }
}
