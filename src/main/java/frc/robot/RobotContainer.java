// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;


public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final CommandXboxController m_controller;

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public final HashMap<String, Command> eventMap = new HashMap<>();

  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    
    m_controller = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    configureButtonBindings();
  }

  private void configureButtonBindings(){

    Trigger aButton = m_controller.a().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    Trigger bButton = m_controller.b();
    Trigger yButton = m_controller.y();
    Trigger xButton = m_controller.x();
    Trigger lBumper = m_controller.leftBumper();
    Trigger rBumper = m_controller.rightBumper();
    Trigger lDPad = m_controller.povLeft();
    Trigger rDPad = m_controller.povRight();
    Trigger uDPad = m_controller.povUp();
    Trigger dDPad = m_controller.povDown();

    new JoystickButton(driverJoytick, 2 ).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));


  }


  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
