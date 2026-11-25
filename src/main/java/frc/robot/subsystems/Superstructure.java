package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase{
    public enum SuperstructureWantedState{
        IDLE, 
        DRIVING,
        INTAKING,
        DASHBOARD,
        SHOOTING,
        REVUP,
        
    }
}
