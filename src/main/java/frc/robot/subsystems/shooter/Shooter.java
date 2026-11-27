package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    private SparkMax feeder = new SparkMax(ShooterConstants.feederMotor, MotorType.kBrushless);
    private SparkMax shooter = new SparkMax(ShooterConstants.shooterMotor, MotorType.kBrushless);

    public enum WantedState {
        OFF,
        PRE_SHOOT,
        SHOOT,
        REVERSE,
        DASHBOARD
    }

    public enum CurrentState {
        OFF,
        PRE_SHOOT,
        SHOOT,
        REVERSE,
        DASHBOARD
    } 

    public WantedState wantedState = WantedState.OFF;
    public CurrentState currentState = CurrentState.OFF;

    public CurrentState getShooterCurrentState() {
        return currentState;
}
    @Override
    public void periodic() {
        
        handleStates();
        applyStates();
}
}
