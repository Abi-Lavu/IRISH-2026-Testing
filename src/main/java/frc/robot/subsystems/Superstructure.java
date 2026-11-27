package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.transfer.Transfer;


public class Superstructure extends SubsystemBase {

    public enum SuperstructureWantedState {
        IDLE,
        DRIVING,
        INTAKING,
        REVERSE_INTAKE,
        REVUP,
        PRE_SHOOT,
        SHOOTING,
        DASHBOARD
    }

    public enum SuperstructureCurrentState {
        IDLE,
        DRIVING,
        INTAKING,
        REVERSE_INTAKE,
        REVUP,
        PRE_SHOOT,
        SHOOTING,
        DASHBOARD
    }

    private final Vision vision;
    private final Intake intake;
    private final Shooter shooter;
    private final Drive drive;
    private final Transfer transfer;

    public SuperstructureWantedState wantedState = SuperstructureWantedState.DRIVING;
    public SuperstructureCurrentState currentState = SuperstructureCurrentState.DRIVING;
    private SuperstructureCurrentState previousCurrentState = SuperstructureCurrentState.DRIVING;

    @AutoLog
    public static class SuperstructureInputs {
        public String currentState = "IDLE";
        public String wantedState = "IDLE";
        public double targetDistanceMeters = 0.0;
        public boolean hubVisible = false;
    }

    private final SuperstructureInputsAutoLogged superstructureInputs = new SuperstructureInputsAutoLogged();

    public Superstructure(Vision vision, Intake intake, Shooter shooter, Drive drive, Transfer transfer) {
        this.vision = vision;
        this.intake = intake;
        this.shooter = shooter;
        this.drive = drive;
        this.transfer = transfer;
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Superstructure", superstructureInputs);
        handleStates();
        applyStates();
    }

    private void updateInputs() {
        superstructureInputs.currentState = currentState.toString();
        superstructureInputs.wantedState = wantedState.toString();
        superstructureInputs.targetDistanceMeters = vision.getHubDistanceMeters().orElse(0.0);
        superstructureInputs.hubVisible = vision.hasHubTarget();
    }

    private void handleStates() {
        switch (wantedState) {
            case IDLE:
                currentState = SuperstructureCurrentState.IDLE;
                break;
            case DRIVING:
                currentState = SuperstructureCurrentState.DRIVING;
                break;
            case INTAKING:
                currentState = SuperstructureCurrentState.INTAKING;
                break;
            case REVERSE_INTAKE:
                currentState = SuperstructureCurrentState.REVERSE_INTAKE;
                break;
            case REVUP:
                currentState = SuperstructureCurrentState.REVUP;
                break;
            case PRE_SHOOT:
                currentState = SuperstructureCurrentState.PRE_SHOOT;
                break;
            case SHOOTING:
                currentState = SuperstructureCurrentState.SHOOTING;
                break;
            case DASHBOARD:
                currentState = SuperstructureCurrentState.DASHBOARD;
                break;
        }
        if (currentState != previousCurrentState) {
            Logger.recordOutput("Superstructure/LastStateChange", previousCurrentState + " -> " + currentState);
        }
    }

    private void applyStates() {
        switch (currentState) {
            case IDLE:
                break;
            case DRIVING:
                break;
            case INTAKING:
                break;
            case REVERSE_INTAKE:
                break;
            case REVUP:
                break;
            case PRE_SHOOT:
                break;
            case SHOOTING:  
                break;
            case DASHBOARD:
                break;
        }
    }

    public SuperstructureCurrentState getCurrentState() {
        return currentState;
    }

    public void requestIdle() {
        wantedState = SuperstructureWantedState.IDLE;
    }

    public void requestDriving() {
        wantedState = SuperstructureWantedState.DRIVING;
    }

    public void requestIntake() {
        wantedState = SuperstructureWantedState.INTAKING;
    }

    public void requestDashboard() {
        wantedState = SuperstructureWantedState.DASHBOARD;
    }

    public void requestShooting() {
        wantedState = SuperstructureWantedState.SHOOTING;
    }

    public void requestRevUp() {
        wantedState = SuperstructureWantedState.REVUP;
    }
    public void requestPreShoot() {
        wantedState = SuperstructureWantedState.PRE_SHOOT;
    }
    public void requestReverseIntake(){
        wantedState = SuperstructureWantedState.REVERSE_INTAKE;
    }

    public Command setIdle() {
        return Commands.runOnce(() -> {
            requestIdle();
        });
    }

    public Command setDriving() {
        return Commands.runOnce(() -> {
            requestDriving();
        });
    }

    public Command setIntake() {
        return Commands.runOnce(() -> {
            requestIntake();
        });
    }

    public Command setDashboard() {
        return Commands.runOnce(() -> {
            requestDashboard();
        });
    }

    public Command setShooting() {
        return Commands.runOnce(() -> {
            requestShooting();
        });
    }

    public Command setRevUp() {
        return Commands.runOnce(() -> {
            requestRevUp();
        });
    }
    public Command setPreShoot() {
        return Commands.runOnce(() -> {
            requestPreShoot();
        });
    }
    public Command setReverseIntake() {
        return Commands.runOnce(() -> {
            requestReverseIntake();
        });
    }
}
