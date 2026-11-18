package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {

  private final WPI_TalonSRX leftFront = new WPI_TalonSRX(3);
  private final WPI_TalonSRX leftRear = new WPI_TalonSRX(4);
  private final WPI_TalonSRX rightFront = new WPI_TalonSRX(1);
  private final WPI_TalonSRX rightRear = new WPI_TalonSRX(2);
  private final WPI_TalonSRX shooter = new WPI_TalonSRX(5);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftFront, rightFront);

  private final XboxController controller = new XboxController(0);

  public Robot() {
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    rightFront.setInverted(true);
    rightRear.setInverted(true);

    differentialDrive.setDeadband(0.05);
    differentialDrive.setMaxOutput(0.20);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    shooter.set(0.20);
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    // Left stick Y = forward/back
    double throttle = controller.getLeftY();
    // Left stick X = turning
    double rotation = controller.getLeftX();

    differentialDrive.arcadeDrive(throttle, rotation);

    shooter.set(controller.getRightTriggerAxis() / 5.0);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
