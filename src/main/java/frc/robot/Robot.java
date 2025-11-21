package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
  
  // Master motors
  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(1);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(3);
  
  // Followers
  private final WPI_TalonSRX leftFollower = new WPI_TalonSRX(2);
  private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(4);
  
  private final XboxController controller = new XboxController(0);
  
  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

  @Override
  public void robotInit() {
    // Set followers
    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    // Invert right side motors so they move forward together
    rightMaster.setInverted(true);
    rightFollower.setInverted(true);
    
    differentialDrive.setDeadband(0.05);
  }

  @Override
  public void teleopPeriodic() {
    // Left stick Y = forward/back
    double throttle = -controller.getLeftY();   // flip so up is positive
    // Right stick X = turning
    double rotation = controller.getRightX();

    differentialDrive.arcadeDrive(throttle, rotation, false);
  }
}
