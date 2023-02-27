// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.AccelerometerSubsystem;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;

// /** An example command that uses an example subsystem. */
// public class CollisionCheckCommand extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final AccelerometerSubsystem m_subsystem;
//   public boolean stopRobot = false;

//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public CollisionCheckCommand(AccelerometerSubsystem subsystem) {
//     m_subsystem = subsystem;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(subsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//         double xJerk = m_subsystem.returnCurrentAcceleration("X") - m_subsystem.returnPreviousAcceleration("X");
//         double zJerk = m_subsystem.returnCurrentAcceleration("Z") - m_subsystem.returnPreviousAcceleration("Z");
//         if (xJerk > Constants.COLLISION_THRESHOLD || zJerk > Constants.COLLISION_THRESHOLD){
//             stopRobot = true;
//         }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//       //code in here stops the robot
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//       if (!stopRobot){
//           return false;
//       }

//       else{
//           return true;
//       }

//   }
// }