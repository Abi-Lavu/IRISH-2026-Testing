// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOTalonSRX;

public class RobotContainer {
    private final CommandXboxController xbox =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    // Xbox sticks are negative when pushed forward/left, so flip them.
    // WPILib wants forward, left and counterclockwise positive.
    private final Drive drive = new Drive(
        new DriveIOTalonSRX(),
        () -> -deadband(xbox.getLeftY()), // forward
        () -> -deadband(xbox.getLeftX()), // strafe left
        () -> -deadband(xbox.getRightX())); // turn counterclockwise

    public RobotContainer() {
        configureBindings();
        logCommands();
    }

    private void configureBindings() {
        // Left bumper (hold): slow mode.
        xbox.leftBumper().whileTrue(drive.slowMode());
    }

    /** No autos yet: the robot sits still in autonomous. */
    public Command getAutonomousCommand() {
        return Commands.none();
    }

    /** Logs which commands are running, so you can see button presses in AdvantageScope. */
    private static void logCommands() {
        CommandScheduler s = CommandScheduler.getInstance();
        s.onCommandInitialize(c -> Logger.recordOutput("Commands/" + c.getName(), true));
        s.onCommandFinish(c -> Logger.recordOutput("Commands/" + c.getName(), false));
        s.onCommandInterrupt(c -> Logger.recordOutput("Commands/" + c.getName(), false));
    }

    private static double deadband(double value) {
        return MathUtil.applyDeadband(value, DriveConstants.Deadband);
    }
}
