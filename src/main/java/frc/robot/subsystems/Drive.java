// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
  private static final double MAX_SPEED = 5.118; // m/s

  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(1);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(3);
  private final WPI_TalonSRX leftFollower = new WPI_TalonSRX(2);
  private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(4);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
  
  private final DifferentialDrivetrainSim drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);
  private final Field2d field = new Field2d();
  private final Timer timer = new Timer();

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.kZero, 0, 0);
  // value copied from DifferentialDrivetrainSim::createKitbotSim()
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(26));

  /** Creates a new Drive. */
  public Drive() {
    // Set followers
    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    // Invert right side motors so they move forward together
    rightMaster.setInverted(true);
    rightFollower.setInverted(true);

    // TODO: current limits

    differentialDrive.setDeadband(0.05);

    SmartDashboard.putData(field);

    timer.start();

    configurePathPlanner();
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

    SmartDashboard.putNumber("Drive/Speed", getRobotRelativeSpeeds().vxMetersPerSecond);
  }

  public Command getTeleopCommand(XboxController controller) {
    // TODO: Limit the speed according to an adjustable NetworkTables value.
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
  
  private Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  
  private void resetPose(Pose2d pose) {
    odometry.resetPose(pose);
  }
  
  private ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(drivetrainSim.getLeftVelocityMetersPerSecond(), drivetrainSim.getRightVelocityMetersPerSecond()));
  }
  
  private void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    differentialDrive.tankDrive(wheelSpeeds.leftMetersPerSecond / MAX_SPEED, wheelSpeeds.rightMetersPerSecond / MAX_SPEED);
  }

  private void configurePathPlanner() {
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      System.err.println("Failed to load PathPlanner RobotConfig:");
      e.printStackTrace();
    }
    
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                              // ChassisSpeeds. Also optionally outputs individual
                                                              // module feedforwards
        new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential
                                   // drive trains
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
    
    // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
    //     field.setRobotPose(pose);
    // });
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        field.getObject("Target Pose").setPose(pose);
    });
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
        field.getObject("Path").setPoses(poses);
    });
  }
}
