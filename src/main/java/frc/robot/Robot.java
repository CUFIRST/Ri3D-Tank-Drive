package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  /*
   * Encoders are CCW positive.
   */
  
  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(1);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(3);

  private final WPI_TalonSRX leftFollower = new WPI_TalonSRX(2);
  private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(4);

  private final Servo servo = new Servo(7);

  /*
   * Gear Motor is PG-71 gearbox; (226233 / 3179) : 1 ~71:1 ratio.
   * Gear Motor encoder is 7 pulses/rev of the motor.
   */
  private final Encoder gearMotorEncoder = new Encoder(8, 9, true);
  private final Spark gearMotor = new Spark(8);
  private final Spark wiperMotor = new Spark(9);
  // CameraServer does not require instantiation

  private final Encoder revQuadratureEncoder = new Encoder(5, 6, false);

  /* 
   * Absolute encoder is not connected to anything.
   * Change "fullRange" to change output units (e.g. 360 for degrees).
   * "expectedZero" must be in the same units as "fullRange".
   */
  private final DutyCycleEncoder revAbsoluteEncoder = new DutyCycleEncoder(7, 1, 0);

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
    revQuadratureEncoder.setDistancePerPulse(1.0 / 2048);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Gear Motor Encoder Distance (Rotations)", gearMotorEncoder.getDistance());
    SmartDashboard.putNumber("Gear Motor Encoder Rate (RPS)", gearMotorEncoder.getRate());
    SmartDashboard.putNumber("REV Encoder Distance (Rotations)", revQuadratureEncoder.getDistance());
    SmartDashboard.putNumber("REV Encoder Rate (RPS)", revQuadratureEncoder.getRate());
    SmartDashboard.putBoolean("Absolute Encoder Connected", revAbsoluteEncoder.isConnected());
    SmartDashboard.putNumber("Absolute Encoder Position (Rotations)", revAbsoluteEncoder.get());
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

    servo.setAngle(SmartDashboard.getNumber("Servo Commanded Angle (0°-180°)", servo.getAngle()));
    SmartDashboard.putNumber("Servo Commanded Angle (0°-180°)", servo.getAngle());
  }
}
