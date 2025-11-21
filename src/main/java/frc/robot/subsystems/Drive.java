// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(1);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(3);
  private final WPI_TalonSRX leftFollower = new WPI_TalonSRX(2);
  private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(4);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

  /** Creates a new Drive. */
  public Drive() {
    // Set followers
    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    // Invert right side motors so they move forward together
    rightMaster.setInverted(true);
    rightFollower.setInverted(true);

    differentialDrive.setDeadband(0.05);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
