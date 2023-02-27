// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MoveClaw;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.WheelDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final Claw m_Claw = new Claw(Constants.clawPort);
  // private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem(Constants.elevatorPort);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));    
    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // JoystickButton bumber_buttonright = new JoystickButton(Constants.auxController, Button.kRightBumper.value);
    // bumber_buttonright.whileTrue(new MoveClaw(m_Claw, 1));
    // bumber_buttonright.onFalse(new MoveClaw(m_Claw, 0));

    // JoystickButton bumber_buttonleft= new JoystickButton(Constants.auxController, Button.kLeftBumper.value);
    // bumber_buttonleft.whileTrue(new MoveClaw(m_Claw, 2));
    // bumber_buttonright.onFalse(new MoveClaw(m_Claw, 0));

    // JoystickButton buttonElevatorUp = new JoystickButton(Constants.auxController, Button.kX.value);
    // buttonElevatorUp.whileTrue(new ElevatorCommand(m_Elevator, 1));
    // buttonElevatorUp.onFalse(new ElevatorCommand(m_Elevator, 0));

    // JoystickButton buttonElevatorDown = new JoystickButton(Constants.auxController, Button.kY.value);
    // buttonElevatorDown.whileTrue(new ElevatorCommand(m_Elevator, 2));
    // buttonElevatorDown.onFalse(new ElevatorCommand(m_Elevator, 0));



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
