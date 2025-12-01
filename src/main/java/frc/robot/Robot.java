package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {
  
  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(1);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(3);

  private final WPI_TalonSRX leftFollower = new WPI_TalonSRX(2);
  private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(4);

  /*
   * Gear Motor is PG-71 gearbox; (226233 / 3179) : 1 ~71:1 ratio.
   * Gear Motor encoder is 7 pulses/rev of the motor.
   */
  private final Encoder gearMotorEncoder = new Encoder(8, 9, true); // TODO: plug these in
  private final Spark gearMotor = new Spark(8); // TODO: plug this in
  private final Spark wiperMotor = new Spark(9);
  // CameraServer does not require instantiation

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

  private final XboxController controller = new XboxController(0);

  @Override
  public void robotInit() {
    // Start camera
    CameraServer.startAutomaticCapture();
    // Set followers
    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    // Invert right side motors so they move forward together
    rightMaster.setInverted(true);
    rightFollower.setInverted(true);

    differentialDrive.setDeadband(0.05);

    // outputs 1.0 rotations per 1.0 *output shaft* rotations
    gearMotorEncoder.setDistancePerPulse(1 / (7 * (226233 / 3179)));
  }

  @Override
  public void teleopPeriodic() {
    // Left stick Y = forward/back
    double throttle = -controller.getLeftY(); // flip so up is positive
    // Left stick X = turning
    double rotation = controller.getLeftX();

    differentialDrive.arcadeDrive(throttle, rotation, false);

    if (controller.getBButton()) {
      wiperMotor.set(1.0);
    } else if (controller.getAButton()) {
      wiperMotor.set(-1.0);
    } else {
      wiperMotor.stopMotor();
    }

    gearMotor.set(controller.getRightY());

    SmartDashboard.putNumber("Encoder Distance (Rotations)", gearMotorEncoder.getDistance());
    SmartDashboard.putNumber("Encoder Rate (RPS)", gearMotorEncoder.getRate());
  }
}
