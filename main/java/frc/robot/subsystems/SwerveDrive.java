package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
  public final double L = 10;
  public final double W = 10;
  private WheelDrive backRight;
  private WheelDrive backLeft;
  private WheelDrive frontRight;
  private WheelDrive frontLeft;
  public double backRightAngle = 0;
  public double backLeftAngle = 0;
  public double frontRightAngle = 0;
  public double frontLeftAngle = 0;

public SwerveDrive (WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {
    this.backRight = backRight;
    this.backLeft = backLeft;
    this.frontRight = frontRight;
    this.frontLeft = frontLeft;
}
  public void drive (double x1, double y1, double x2) {
    // double r = Math.sqrt ((L * L) + (W * W));
    double dx1 = deadzone(x1,0.1);
    double dx2 = deadzone(x2,0.1);
    double dy1 = deadzone(y1,0.1);

    y1 *= -1;
    double a = dx1 - dx2 * (L / Constants.OperatorConstants.drive_r);
    double b = dx1 + dx2 * (L / Constants.OperatorConstants.drive_r);
    double c = dy1 - dx2 * (W / Constants.OperatorConstants.drive_r);
    double d = dy1 + dx2 * (W / Constants.OperatorConstants.drive_r);
    
    //double magnitude = deadzone(Math.hypot(x1, y1)*0.9, 0.05);
    //double angle = Math.atan2(x1,y1);
    double backRightSpeed = Math.sqrt((a * a) + (d * d));
    double backLeftSpeed = Math.sqrt((a * a) + (c * c));
    double frontRightSpeed = Math.sqrt((b * b) + (d * d));
    double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

    backRightAngle = (Math.atan2 (a, d) / Math.PI) * 180;
    backLeftAngle = (Math.atan2 (a, c) / Math.PI) * 180;
    frontRightAngle = (Math.atan2 (b, d) / Math.PI) * 180;
    frontLeftAngle = (Math.atan2 (b, c) / Math.PI) * 180;
    
    backRight.drive (backRightSpeed, backRightAngle);
    backLeft.drive (backLeftSpeed, backLeftAngle);
    frontRight.drive (frontRightSpeed, frontRightAngle);
    frontLeft.drive (frontLeftSpeed, frontLeftAngle);
    
}
  /** Creates a new ExampleSubsystem. */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  public void stopAll(){
    backRight.drive (0, 0);
    backLeft.drive (0, 0);
    frontRight.drive (0, 0);
    frontLeft.drive (0, 0);
  }
  private double deadzone(double value, double deadzone)
  {
      if (Math.abs(value) < deadzone)
      {
          return 0;
      }
      return value;
  }
  public void zeroizeEncoders(){
      backRight.moveAngleMotor(backRight.getSensorValue(),0.0,true);
      backLeft.moveAngleMotor(backLeft.getSensorValue(),0.0,true);
      frontLeft.moveAngleMotor(frontLeft.getSensorValue(),0.0,false);
      frontRight.moveAngleMotor(frontRight.getSensorValue(),0.0,true);

  }
  }

