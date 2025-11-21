// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(1);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(3);
  private final WPI_TalonSRX leftFollower = new WPI_TalonSRX(2);
  private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(4);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
  
  // TODO: the gear ratio is almost certainly not correct
  private final DifferentialDrivetrainSim drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDualCIMPerSide,  KitbotGearing.k8p45, KitbotWheelSize.kSixInch, null);
  private final Field2d field = new Field2d();
  private final Timer timer = new Timer();

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.kZero, 0, 0);

  /** Creates a new Drive. */
  public Drive() {
    // Set followers
    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    // Invert right side motors so they move forward together
    rightMaster.setInverted(true);
    rightFollower.setInverted(true);

    differentialDrive.setDeadband(0.05);

    SmartDashboard.putData(field);

    timer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // This simulation code is intentionally not in simulationPeriodic()
    drivetrainSim.setInputs(leftMaster.getMotorOutputVoltage(), rightMaster.getMotorOutputVoltage());
    drivetrainSim.update(timer.get());
    timer.reset();
    
    odometry.update(drivetrainSim.getHeading(), drivetrainSim.getLeftPositionMeters(), drivetrainSim.getRightPositionMeters());
    field.setRobotPose(odometry.getPoseMeters());
  }

  public Command getTeleopCommand(XboxController controller) {
    return Commands.runEnd(() -> {
      // Left stick Y = forward/back
      double throttle = -controller.getLeftY();   // flip so up is positive
      // Right stick X = turning
      double rotation = controller.getRightX();

      differentialDrive.arcadeDrive(throttle, rotation, false);
    }, () -> {
      differentialDrive.stopMotor();
    }, this);
  }
}
