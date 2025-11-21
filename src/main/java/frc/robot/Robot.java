package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

  // Master motors
  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(1);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(3);

  // Followers
  private final WPI_TalonSRX leftFollower = new WPI_TalonSRX(2);
  private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(4);

  private final XboxController controller = new XboxController(0);

  @Override
  public void robotInit() {
    // Set followers
    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    // Invert right side motors so they move forward together
    rightMaster.setInverted(true);
    rightFollower.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    // Left stick Y = forward/back
    double throttle = -controller.getLeftY();   // flip so up is positive
    // Right stick X = turning
    double rotation = controller.getRightX();

    // Arcade math
    double leftPower = throttle + rotation;
    double rightPower = throttle - rotation;

    // Clamp to [-1, 1]
    leftPower = Math.max(-1, Math.min(1, leftPower));
    rightPower = Math.max(-1, Math.min(1, rightPower));

    // Send it
    leftMaster.set(ControlMode.PercentOutput, leftPower);
    rightMaster.set(ControlMode.PercentOutput, rightPower);
  }
}
