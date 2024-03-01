// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.classes.BaseDrive;
import frc.robot.commands.*;
import frc.robot.constants.Constants;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.NeoModule;
import frc.robot.subsystems.chassis.PoseEstimator;
import frc.robot.subsystems.chassis.SwerveModule;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

class TestChassisContainer {
        public final Chassis m_chassis;
        private final BaseDrive m_baseDrive;
        public final PoseEstimator m_poseEstimator;
        private PhotonCamera m_ATCamera;
        private final CommandXboxController m_driveController;
        private final CommandXboxController m_manipController;
        private final SendableChooser<Command> autoChooser;

        TestChassisContainer() {

                NeoModule frontLeft = new NeoModule(41, 42, RobotConstants.MODULE_CONFIG, RobotConstants.MODULE_FF[0]); // TODO:
                                                                                                                        // lower
                                                                                                                        // ids
                                                                                                                        // to
                                                                                                                        // 35,
                                                                                                                        // 36
                NeoModule frontRight = new NeoModule(11, 12, RobotConstants.MODULE_CONFIG, RobotConstants.MODULE_FF[1]);
                NeoModule backLeft = new NeoModule(31, 32, RobotConstants.MODULE_CONFIG, RobotConstants.MODULE_FF[2]);
                NeoModule backRight = new NeoModule(21, 22, RobotConstants.MODULE_CONFIG, RobotConstants.MODULE_FF[3]);

                SwerveModule[] modules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };

                SwerveDriveKinematics swerveDriveKinematics = getSwerveDriveKinematics();

                m_ATCamera = new PhotonCamera(RobotConstants.AT_CAMERA_NAME);

                m_chassis = new Chassis(modules, swerveDriveKinematics);

                m_manipController = new CommandXboxController(0);

                m_poseEstimator = new PoseEstimator(
                                m_chassis,
                                m_ATCamera,
                                RobotConstants.CAM1_OFFSET,
                                RobotConstants.PIGEON_ID,
                                RobotConstants.STATE_STD_DEVS,
                                RobotConstants.VISION_STD_DEVS,
                                RobotConstants.SINGLE_TAG_STD_DEVS,
                                RobotConstants.MULTI_TAG_STD_DEVS);

                m_driveController = new CommandXboxController(0);

                m_baseDrive = new BaseDrive(
                                m_driveController.getHID(), RobotConstants.MOTION_LIMITS, RobotConstants.RATE_LIMITS);

                PortForwarder.add(5800, "photonvision.local", 5800);

                m_chassis.setDefaultCommand(
                                new FieldOrientedDrive(m_chassis, m_poseEstimator,
                                                m_baseDrive::calculateChassisSpeeds));

                AutoBuilder.configureHolonomic(
                                m_poseEstimator::getFusedPose, // Robot pose supplier
                                m_poseEstimator::resetPose, // Method to reset odometry, TODO check if it still works
                                                            // with the command
                                m_chassis::getRealChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                                m_chassis::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
                                // ChassisSpeeds
                                RobotConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
                                () -> {
                                        var alliance = DriverStation.getAlliance();
                                        if (alliance.isPresent()) {
                                                return alliance.get() == DriverStation.Alliance.Red;
                                        }
                                        return false;
                                },
                                m_chassis // Reference to this subsystem to set requirements
                );

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("AutoMode", autoChooser);

                configureBindings();
        }

        private static SwerveDriveKinematics getSwerveDriveKinematics() {
                Translation2d frontLeftLocation = new Translation2d(RobotConstants.WHEEL_WIDTH / 2.0,
                                RobotConstants.WHEEL_WIDTH / 2.0);
                Translation2d frontRightLocation = new Translation2d(RobotConstants.WHEEL_WIDTH / 2.0,
                                -RobotConstants.WHEEL_WIDTH / 2.0);
                Translation2d backLeftLocation = new Translation2d(-RobotConstants.WHEEL_WIDTH / 2.0,
                                RobotConstants.WHEEL_WIDTH / 2.0);
                Translation2d backRightLocation = new Translation2d(-RobotConstants.WHEEL_WIDTH / 2.0,
                                -RobotConstants.WHEEL_WIDTH / 2.0);

                return new SwerveDriveKinematics(
                                frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        }

        private void configureBindings() {
                m_driveController
                                .start()
                                .onTrue(new InstantCommand(() -> m_poseEstimator.resetPose(new Pose2d())));
                m_driveController
                                .leftBumper()
                                .whileTrue(
                                                new OrbitalTarget(
                                                                m_chassis,
                                                                m_baseDrive::calculateChassisSpeeds,
                                                                RobotConstants.TRANSLATION_PID,
                                                                RobotConstants.ROTATION_PID,
                                                                RobotConstants.MOTION_LIMITS,
                                                                m_poseEstimator,
                                                                () -> Constants.ORBIT_RADIUS,
                                                                RobotConstants.ORBITAL_FF_CONSTANT));

                m_driveController
                                .rightBumper()
                                .whileTrue(
                                                new RunCommand(
                                                                () -> m_chassis.driveRobotRelative(
                                                                                m_baseDrive.calculateChassisSpeeds()),
                                                                m_chassis));

                m_driveController
                                .pov(0)
                                .whileTrue(
                                                new AlignToPose(
                                                                m_chassis,
                                                                Constants.AMP_TRANSFORM,
                                                                m_poseEstimator,
                                                                RobotConstants.TRANSLATION_PID,
                                                                RobotConstants.ROTATION_PID,
                                                                RobotConstants.MOTION_LIMITS));
                m_driveController
                                .a()
                                .or(m_driveController.b())
                                .or(m_driveController.x())
                                .or(m_driveController.y())
                                .whileTrue(
                                                new FieldOrientedWithCardinal(
                                                                m_chassis,
                                                                m_poseEstimator,
                                                                () -> {
                                                                        double xCardinal = (m_driveController.y()
                                                                                        .getAsBoolean() ? 1 : 0)
                                                                                        - (m_driveController.a()
                                                                                                        .getAsBoolean() ? 1
                                                                                                                        : 0);
                                                                        double yCardinal = (m_driveController.b()
                                                                                        .getAsBoolean() ? 1 : 0)
                                                                                        - (m_driveController.x()
                                                                                                        .getAsBoolean() ? 1
                                                                                                                        : 0);
                                                                        double dir = -Math.atan2(yCardinal, xCardinal);
                                                                        dir = dir < 0 ? dir + 2 * Math.PI : dir; // TODO
                                                                                                                 // check
                                                                                                                 // if
                                                                                                                 // needed

                                                                        Logger.recordOutput("goalCardinal", dir);
                                                                        return dir;
                                                                },
                                                                m_baseDrive::calculateChassisSpeeds,
                                                                RobotConstants.ROTATION_PID,
                                                                RobotConstants.ROTATION_CONSTRAINTS,
                                                                RobotConstants.ROTATION_FF));

                m_manipController.leftTrigger().whileTrue(new DriveToNote(m_chassis));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
