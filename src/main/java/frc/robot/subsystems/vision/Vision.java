package frc.robot.subsystems.vision;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;


public class Vision extends SubsystemBase {
    public enum State {
        DISCONNECTED, 
        NO_TARGET, 
        HUB_TARGET 
    }

    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private State state = State.DISCONNECTED;

   
    private final BooleanPublisher dashboardHasTarget =
        NetworkTableInstance.getDefault().getBooleanTopic("/Vision/HasTarget").publish();

    public Vision(VisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        if (!inputs.connected) {
            state = State.DISCONNECTED;
        } else if (inputs.hasTarget && isHubTag(inputs.tagId)) {
            state = State.HUB_TARGET;
        } else {
            state = State.NO_TARGET;
        }

        dashboardHasTarget.set(state == State.HUB_TARGET);
        Logger.recordOutput("Vision/State", state);
        Logger.recordOutput("Vision/HubDistanceMeters", hasHubTarget() ? inputs.distanceMeters : 0.0);
    }

    public State getState() {
        return state;
    }

    public boolean hasHubTarget() {
        return state == State.HUB_TARGET;
    }

    
    public OptionalDouble getHubDistanceMeters() {
        return hasHubTarget() ? OptionalDouble.of(inputs.distanceMeters) : OptionalDouble.empty();
    }

   
    public OptionalDouble getHubTxDegrees() {
        return hasHubTarget() ? OptionalDouble.of(inputs.txDegrees) : OptionalDouble.empty();
    }

    private static boolean isHubTag(int id) {
        for (int hubId : VisionConstants.HubTagIds) {
            if (id == hubId) {
                return true;
            }
        }
        return false;
    }
}
