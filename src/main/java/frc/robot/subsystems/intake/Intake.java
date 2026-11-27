package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    private SparkMax dropdownMotor = new SparkMax(IntakeConstants.dropdownMotor, MotorType.kBrushless);
    private SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeMotor, MotorType.kBrushless);

    public enum WantedState {
        IDLE,
        STOW,
        INTAKE,
        REVERSE
    }

    public enum CurrentState {
        IDLE,
        STOW,
        INTAKE,
        REVERSE
    }

    public WantedState wantedState = WantedState.IDLE;
    public CurrentState currentState = CurrentState.IDLE;
    private CurrentState lastState = CurrentState.IDLE;

    public CurrentState getIntakeCurrentState() {
        return currentState;
}

    @Override
    public void periodic() {
        handleStates();
        applyStates();
        lastState = currentState;
}


}