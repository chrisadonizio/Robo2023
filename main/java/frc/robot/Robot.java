// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Drive;
import frc.robot.subsystems.AccelerometerSubsystem;
// import frc.robot.subsystems.Accel;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.WheelDrive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  //input order: steering, drive
  private static WheelDrive backRight = new WheelDrive (8,4);
  private static WheelDrive backLeft = new WheelDrive (7, 3);
  private static WheelDrive frontRight = new WheelDrive (6, 2);
  private static WheelDrive frontLeft = new WheelDrive (5, 1);
  private static XboxController controllerOne = new XboxController(0);
  private static int counter = 0;
  public static SwerveDrive m_swerveDrive = new SwerveDrive (backRight, backLeft, frontRight, frontLeft);
  // public static AccelerometerSubsystem accel = new AccelerometerSubsystem();
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_swerveDrive.zeroizeEncoders();
    counter++;
    if(counter%100==0){
    SmartDashboard.putNumber("Talon 1 PID",frontLeft.PIDVal);
    SmartDashboard.putNumber("Talon 2 PID",frontRight.PIDVal);
    SmartDashboard.putNumber("Talon 3 PID",backLeft.PIDVal);
    SmartDashboard.putNumber("Talon 4 PID",backRight.PIDVal);
    SmartDashboard.putNumber("Talon 1 Position", frontLeft.getSensorValue() );
    SmartDashboard.putNumber("Talon 2 Position", frontRight.getSensorValue() );
    SmartDashboard.putNumber("Talon 3 Position", backLeft.getSensorValue());
    SmartDashboard.putNumber("Talon 4 Position", backRight.getSensorValue());
  }

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //help 
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // new Drive().schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // double rightX = controllerOne.getRightX();
    // double leftX = controllerOne.getLeftX();
    // double leftY = controllerOne.getLeftY();

    // m_swerveDrive.drive(leftX, leftY, rightX);

    counter++;
    if(counter%100==0){
    SmartDashboard.putNumber("Talon 1 Position", frontLeft.getSensorValue() );
    SmartDashboard.putNumber("Talon 2 Position", frontRight.getSensorValue() );
    SmartDashboard.putNumber("Talon 3 Position", backLeft.getSensorValue());
    SmartDashboard.putNumber("Talon 4 Position", backRight.getSensorValue());

    SmartDashboard.putNumber("Swerve 1 Angle", m_swerveDrive.frontLeftAngle );
    SmartDashboard.putNumber("Swerve 2 Angle", m_swerveDrive.frontRightAngle );
    SmartDashboard.putNumber("Swerve 3 Angle", m_swerveDrive.backLeftAngle);
    SmartDashboard.putNumber("Swerve 4 Angle", m_swerveDrive.backRightAngle);
  }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
