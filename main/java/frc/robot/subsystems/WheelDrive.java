package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class WheelDrive extends SubsystemBase{
private TalonSRX angleMotor;
private CANSparkMax driveMotor;
public PIDController anglePIDController;
private int counter = 0; 
public double PIDVal = 0;
public WheelDrive (int driveMotor, int angleMotor) {
    this.angleMotor = new TalonSRX (angleMotor);
    this.driveMotor = new CANSparkMax(driveMotor, MotorType.kBrushless);
    // this.angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    anglePIDController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    anglePIDController.enableContinuousInput(-180, 180);
    anglePIDController.setTolerance(10, 10);
}
public void drive (double speed, double angle) {
    driveMotor.set(speed*0.3);
    PIDVal = anglePIDController.calculate(angleMotor.getSelectedSensorPosition(), angle);
    angleMotor.set(ControlMode.PercentOutput,(PIDVal/180) * 0.3);
}
public void moveAngleMotor(double currentPosition, double desiredPosition, boolean isForward){
    PIDVal = anglePIDController.calculate(currentPosition,desiredPosition);
    if(anglePIDController.atSetpoint())
    {
    if(isForward){
        angleMotor.set(ControlMode.PercentOutput,(PIDVal/180) * 0.3);
    }
    else{
        angleMotor.set(ControlMode.PercentOutput,(PIDVal/180*-1) * 0.3);
    }
    }
}

public double getSensorValue(){
    double tempPos = angleMotor.getSelectedSensorPosition(0);
    double degrees = (tempPos / 1000) * 360;
    degrees = (degrees + 180) % 360 - 180;

    return degrees;
}


}