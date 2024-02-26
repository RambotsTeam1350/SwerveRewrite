package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.Swerve.ModulePosition;

import java.util.HashMap;
import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private final SwerveModule m_FL;
    private final SwerveModule m_FR;
    private final SwerveModule m_RL;
    private final SwerveModule m_RR;

    private final AHRS m_gyro;

    private final HashMap<ModulePosition, SwerveModule> m_swerveModulesToPositions;
    private SwerveDriveOdometry m_odometry;

    private final SlewRateLimiter m_strafeSlewRateLimiter;
    private final SlewRateLimiter m_forwardSlewRateLimiter;
    private final SlewRateLimiter m_rotateSlewRateLimiter;

    private boolean isFieldCentric;

    public Drivetrain() {
        this.m_FL = new SwerveModule(0, Constants.Swerve.FL.kDriveMotorID, Constants.Swerve.FL.kAngleMotorID,
                Constants.Swerve.FL.kAngleAbsoluteEncoderID, Constants.Swerve.FL.kAngleAbsoluteEncoderOffset);
        this.m_FR = new SwerveModule(1, Constants.Swerve.FR.kDriveMotorID, Constants.Swerve.FR.kAngleMotorID,
                Constants.Swerve.FR.kAngleAbsoluteEncoderID, Constants.Swerve.FR.kAngleAbsoluteEncoderOffset);
        this.m_RL = new SwerveModule(2, Constants.Swerve.RL.kDriveMotorID, Constants.Swerve.RL.kAngleMotorID,
                Constants.Swerve.RL.kAngleAbsoluteEncoderID, Constants.Swerve.RL.kAngleAbsoluteEncoderOffset);
        this.m_RR = new SwerveModule(3, Constants.Swerve.RR.kDriveMotorID, Constants.Swerve.RR.kAngleMotorID,
                Constants.Swerve.RR.kAngleAbsoluteEncoderID, Constants.Swerve.RR.kAngleAbsoluteEncoderOffset);

        this.m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
        this.m_gyro.reset();

        this.m_swerveModulesToPositions = new HashMap<>(
                Map.of(
                        ModulePosition.FL,
                        this.m_FL,

                        ModulePosition.FR,
                        this.m_FR,

                        ModulePosition.RL,
                        this.m_RL,

                        ModulePosition.RR,
                        this.m_RR));
        this.m_odometry = new SwerveDriveOdometry(Constants.Swerve.kSwerveDriveKinematics, getHeadingRotation2d(),
                getModulePositions(), new Pose2d());

        this.m_strafeSlewRateLimiter = new SlewRateLimiter(Constants.Swerve.SlewRateLimits.kStrafeSlewRateLimit);
        this.m_forwardSlewRateLimiter = new SlewRateLimiter(Constants.Swerve.SlewRateLimits.kForwardsSlewRateLimit);
        this.m_rotateSlewRateLimiter = new SlewRateLimiter(
                Constants.Swerve.SlewRateLimits.kRotateSlewRateLimit);
    }

    /**
     * Drives the robot. This is best used for driving with joysticks.
     * For programmatic drivetrain control, consider using sendDrive instead.
     * 
     * @param inputX   The left/right translation instruction
     * @param inputY   The forward/back translation instruction
     * @param inputRot The rotational instruction
     */
    public void drive(double inputX, double inputY, double inputRot) {
        // If a translation input is between -.05 and .05, set it to 0
        double deadbandedX = MathUtil.applyDeadband(Math.abs(inputX),
                Constants.Controllers.Driver.kTranslationDeadband) * Math.signum(inputX);
        double deadbandedY = MathUtil.applyDeadband(Math.abs(inputY),
                Constants.Controllers.Driver.kTranslationDeadband) * Math.signum(inputY);

        // If the rotation input is between -.1 and .1, set it to 0
        double deadbandedRot = MathUtil.applyDeadband(Math.abs(inputRot),
                Constants.Controllers.Driver.kRotationDeadband) * Math.signum(inputRot);

        // Square values after deadband while keeping original sign
        deadbandedX = -Math.signum(deadbandedX) * Math.pow(deadbandedX, 2);
        deadbandedY = -Math.signum(deadbandedY) * Math.pow(deadbandedY, 2);
        deadbandedRot = -Math.signum(deadbandedRot) * Math.pow(deadbandedRot, 2);

        // Apply a slew rate to the inputs, limiting the rate at which the robot changes
        // speed
        double slewedX = this.m_strafeSlewRateLimiter.calculate(deadbandedX);
        double slewedY = this.m_forwardSlewRateLimiter.calculate(deadbandedY);
        double slewedRot = this.m_rotateSlewRateLimiter.calculate(deadbandedRot);

        // Send the processed output to the drivetrain
        sendDrive(slewedX, slewedY, slewedRot, true);
    }

    /**
     * Sends driving instructions to the motors that drive the robot.
     * 
     * @param translationX The left/right translation instruction
     * @param translationY The forward/back translation instruction
     * @param rotation     The rotational instruction
     * @param isOpenLoop   True to control the driving motor via %power.
     *                     False to control the driving motor via velocity-based
     *                     PID.
     */
    public void sendDrive(double translationX, double translationY, double rotation, boolean isOpenLoop) {
        // Convert inputs from % to m/sec
        translationY *= Constants.Swerve.MaxSpeeds.kTranslation;
        translationX *= Constants.Swerve.MaxSpeeds.kTranslation;
        rotation *= Constants.Swerve.MaxSpeeds.kRotation;

        ChassisSpeeds chassisSpeeds = isFieldCentric
                // Calculate field relative instructions if isFieldCentric is true
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationY, translationX, rotation, getHeadingRotation2d())
                // Calculate robot centric instructions if isFieldCentric is false
                : new ChassisSpeeds(translationY, translationX, rotation);

        // Convert ChassisSpeed instructions to useable SwerveModuleStates
        SwerveModuleState[] moduleStates = Constants.Swerve.kSwerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Normalize output if any of the modules would be instructed to go faster than
        // possible
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.MaxSpeeds.kTranslation);

        // Send instructions to each module
        for (SwerveModule module : this.m_swerveModulesToPositions.values())
            module.setDesiredState(moduleStates[module.getModuleNumber()], isOpenLoop);
    }

    /** @return The current direction the robot is facing in degrees */
    public double getHeadingDegrees() {
        return -Math.IEEEremainder(this.m_gyro.getAngle(), 360);
    }

    /**
     * @return The current direction the robot is facing as a {@link Rotation2d}
     *         object
     */
    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDegrees());
    }

    /**
     * Reset the heading of the robot, effectively changing the orientation of the
     * field
     */
    public void resetHeading() {
        this.m_gyro.reset();
    }

    /**
     * @return The position in meters and direction of the robot in degrees as a
     *         {@link Pose2d} object
     */
    public Pose2d getPoseMeters() {
        return this.m_odometry.getPoseMeters();
    }

    /**
     * @param moduleNumber The index of the module
     * @return The {@link SwerveModule swerve module} at that index
     */
    public SwerveModule getSwerveModule(int moduleNumber) {
        return this.m_swerveModulesToPositions.get(ModulePosition.values()[moduleNumber]);
    }

    /**
     * @param position The {@link ModulePosition position} of the module
     * @return The {@link SwerveModule swerve module} at that position
     */
    public SwerveModule getSwerveModule(ModulePosition position) {
        return this.m_swerveModulesToPositions.get(position);
    }

    // Methods related to field orientation
    /** @return Whether or not the robot is in field oriented mode */
    public boolean isFieldCentric() {
        return isFieldCentric;
    }

    /** @return The opposite of isFieldCentric() */
    public boolean isRobotCentric() {
        return !isFieldCentric;
    }

    /**
     * @param isFieldCentric Whether the robot should be set to field centric or not
     */
    public void setFieldCentric(boolean isFieldCentric) {
        this.isFieldCentric = isFieldCentric;
    }

    /**
     * @param isFieldCentric Whether the robot should be set to robot centric or not
     */
    public void setRobotCentric(boolean isRobotCentric) {
        this.isFieldCentric = !isRobotCentric;
    }

    /** Sets the robot to field centric if currently robot centric and vice versa */
    public void toggleFieldCentric() {
        this.isFieldCentric = !this.isFieldCentric;
    }

    /** Resets the wheels of the robot to point forward */
    public void zeroWheels() {
        for (SwerveModule module : this.m_swerveModulesToPositions.values())
            module.setDesiredState(
                    new SwerveModuleState(0, new Rotation2d(0)),
                    true);
    }

    /**
     * @return An array containing the current {@link SwerveModuleState state} of
     *         each module
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                this.m_swerveModulesToPositions.get(ModulePosition.FL).getState(),
                this.m_swerveModulesToPositions.get(ModulePosition.FR).getState(),
                this.m_swerveModulesToPositions.get(ModulePosition.RL).getState(),
                this.m_swerveModulesToPositions.get(ModulePosition.RR).getState()
        };
    }

    /**
     * Set the state of each module at once
     * 
     * @param states An array containing the desired {@link SwerveModuleState state}
     *               of each module
     */
    public void setModuleStates(SwerveModuleState[] states) {
        // Normalize output if any of the modules would be instructed to go faster than
        // possible
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.MaxSpeeds.kTranslation);

        // Send instructions to each module
        for (SwerveModule module : this.m_swerveModulesToPositions.values())
            module.setDesiredState(states[module.getModuleNumber()], true);
    }

    /** Set the state of each to 0,0 */
    public void resetModuleStates() {
        // Send instructions to each module
        for (SwerveModule module : this.m_swerveModulesToPositions.values())
            module.setDesiredState(new SwerveModuleState(), true);
    }

    /**
     * @return An array containing the current {@link SwerveModulePosition position}
     *         of each module
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                this.m_swerveModulesToPositions.get(ModulePosition.FL).getPosition(),
                this.m_swerveModulesToPositions.get(ModulePosition.FR).getPosition(),
                this.m_swerveModulesToPositions.get(ModulePosition.RL).getPosition(),
                this.m_swerveModulesToPositions.get(ModulePosition.RR).getPosition()
        };
    }

    /**
     * Updates the odometry of the robot using the {@link SwerveModulePosition
     * position}
     * of each module and the current heading of the robot
     */
    public void updateOdometry() {
        this.m_odometry.update(getHeadingRotation2d(), getModulePositions());

        for (SwerveModule module : this.m_swerveModulesToPositions.values()) {
            var modulePositionFromChassis = Constants.Swerve.kModuleTranslations[module.getModuleNumber()]
                    .rotateBy(getHeadingRotation2d())
                    .plus(getPoseMeters().getTranslation());
            module.setModulePose(
                    new Pose2d(
                            modulePositionFromChassis,
                            module.getHeadingRotation2d().plus(getHeadingRotation2d())));
        }
    }

    /**
     * Sets the odometry of the robot using a given pose
     * 
     * @param pose The pose of the robot
     */
    public void setOdometry(Pose2d pose) {
        this.m_odometry.resetPosition(
                getHeadingRotation2d(),
                getModulePositions(),
                pose);
    }

    /** Resets the odometry of the robot */
    public void resetOdometry() {
        this.m_odometry.resetPosition(
                getHeadingRotation2d(),
                getModulePositions(),
                new Pose2d());
    }

    @Override // Called every 20ms
    public void periodic() {
        updateOdometry();
        // Prints the current heading in degrees
        SmartDashboard.putNumber("Heading in Degrees", getHeadingDegrees());
        // Prints if the robot is in field or robot
        SmartDashboard.putBoolean("Field Centric", isFieldCentric());
    }

}
