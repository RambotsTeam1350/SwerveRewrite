package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
    private int m_moduleNumber;

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_angleMotor;

    private final boolean m_driveMotorInvert;
    private final boolean m_angleMotorInvert;

    private final RelativeEncoder m_driveRelativeEncoder;
    private final RelativeEncoder m_angleRelativeEncoder;

    private final CANcoder m_angleAbsoluteEncoder;
    private final double m_angleAbsoluteEncoderOffset;

    private SparkPIDController m_driveMotorSparkPIDController;
    private PIDController m_angleMotorPIDController;

    private SwerveModuleState m_state;
    private Pose2d m_pose;

    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, boolean driveMotorInvert,
            boolean angleMotorInvert, int angleAbsoluteEncoderID,
            double angleAbsoluteEncoderOffset) {

        this.m_moduleNumber = moduleNumber;
        this.m_driveMotorInvert = driveMotorInvert;
        this.m_angleMotorInvert = angleMotorInvert;
        this.m_driveMotor = new CANSparkMax(driveMotorID, CANSparkLowLevel.MotorType.kBrushless);
        this.m_driveMotor.restoreFactoryDefaults();
        this.m_driveMotor.setSmartCurrentLimit(45);
        this.m_driveMotor.getPIDController().setFF(0.0);
        this.m_driveMotor.getPIDController().setP(0.2);
        this.m_driveMotor.getPIDController().setI(0.0);
        this.m_driveMotor.setInverted(this.m_driveMotorInvert);
        this.m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);
        this.m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
        this.m_driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
        this.m_driveMotor.enableVoltageCompensation(12.6);
        this.m_driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.m_angleMotor = new CANSparkMax(angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        this.m_angleMotor.restoreFactoryDefaults();
        this.m_angleMotor.setSmartCurrentLimit(20);
        this.m_angleMotor.getPIDController().setFF(0.0);
        this.m_angleMotor.getPIDController().setP(0.2);
        this.m_angleMotor.getPIDController().setI(0.0);
        this.m_angleMotor.setInverted(this.m_angleMotorInvert);
        this.m_angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100);
        this.m_angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
        this.m_angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
        this.m_angleMotor.enableVoltageCompensation(12.6);
        this.m_angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.m_driveRelativeEncoder = this.m_driveMotor.getEncoder();
        this.m_driveRelativeEncoder.setPositionConversionFactor(Constants.Swerve.Drive.kRevolutionsToMeters);
        this.m_driveRelativeEncoder.setVelocityConversionFactor(Constants.Swerve.Drive.kRPMtoMPS);

        this.m_angleRelativeEncoder = this.m_angleMotor.getEncoder();
        this.m_angleRelativeEncoder.setPositionConversionFactor(Constants.Swerve.Angle.kRevolutionsToDegrees);
        this.m_angleRelativeEncoder.setVelocityConversionFactor(Constants.Swerve.Angle.kRevolutionsToDegrees / 60.0);

        this.m_angleAbsoluteEncoder = new CANcoder(angleAbsoluteEncoderID);
        this.m_angleAbsoluteEncoderOffset = angleAbsoluteEncoderOffset;

        this.m_driveMotorSparkPIDController = this.m_driveMotor.getPIDController();
        this.m_angleMotorPIDController = new PIDController(0.007, 0.00175, 0.0000625);
    }

    public void resetAngleRelativeEncoderToAbsolute() {
        double angle = (this.m_angleAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360.0)
                - this.m_angleAbsoluteEncoderOffset;
        this.m_angleRelativeEncoder.setPosition(angle);
    }

    public void resetDriveRelativeEncoder() {
        this.m_driveRelativeEncoder.setPosition(0);
    }

    /**
     * Useful for iterating over modules like an array
     * 
     * @return the number of this module
     */
    public int getModuleNumber() {
        return this.m_moduleNumber;
    }

    /** @return the direction this module is facing in degrees */
    public double getHeadingDegrees() {
        return this.m_angleRelativeEncoder.getPosition();
    }

    /**
     * @return the direction this module is facing as a {@link Rotation2d} object
     */
    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDegrees());
    }

    /** @return How far this module has driven total in meters */
    public double getDriveMeters() {
        return this.m_driveRelativeEncoder.getPosition();
    }

    /** @return The current speed of this module in m/sec */
    public double getDriveMetersPerSecond() {
        return this.m_driveRelativeEncoder.getVelocity();
    }

    /** @return The current {@link SwerveModuleState state} of this module */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveMetersPerSecond(), getHeadingRotation2d());
    }

    /**
     * @return The {@link SwerveModulePosition position} of this module
     *         expressed as the total distance driven and current heading
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveMeters(), getHeadingRotation2d());
    }

    /** @return The current {@link Pose2d pose} of this module */
    public Pose2d getModulePose() {
        return this.m_pose;
    }

    /** Sets this module's {@link Pose2d pose} */
    public void setModulePose(Pose2d pose) {
        this.m_pose = pose;
    }

    /**
     * Set the entire module to a desired {@link SwerveModuleState state},
     * controlling
     * both the direction and speed at the same time
     * 
     * @param desiredState The {@link SwerveModuleState state} to set the module to
     * @param isOpenLoop   True to control the driving motor via %power.
     *                     False to control the driving motor via velocity-based
     *                     PID.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        this.m_state = optimize(desiredState, getHeadingRotation2d());

        if (isOpenLoop) {
            // Calculate the %power for the driving motor
            double percentOutput = this.m_state.speedMetersPerSecond / Constants.Swerve.MaxSpeeds.kTranslation;
            // Send instruction to the motor
            this.m_driveMotor.set(percentOutput);
        } else {
            // Set the driving motor's PID controller to the desired speed
            this.m_driveMotorSparkPIDController.setReference(
                    this.m_state.speedMetersPerSecond,
                    CANSparkMax.ControlType.kVelocity,
                    1);
        }

        // Point turning motor at the target angle
        turnToDegrees(this.m_state.angle.getDegrees());
    }

    /**
     * Turn the module to point in some direction
     * 
     * @param angle the target angle in degrees
     */
    public void turnToDegrees(double angle) {
        double turnAngleError = Math.abs(angle - this.m_angleRelativeEncoder.getPosition());

        double pidOut = this.m_angleMotorPIDController.calculate(this.m_angleRelativeEncoder.getPosition(), angle);
        // if robot is not moving, stop the turn motor oscillating
        if (turnAngleError < .5
                && Math.abs(this.m_state.speedMetersPerSecond) <= Constants.Swerve.MaxSpeeds.kTranslation * .01)
            pidOut = 0;

        this.m_angleMotor.setVoltage(pidOut * RobotController.getBatteryVoltage());
    }

    public static SwerveModuleState optimize(
            SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();

        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle       Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;

        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }

        while (newAngle < lowerBound)
            newAngle += 360;
        while (newAngle > upperBound)
            newAngle -= 360;

        if (newAngle - scopeReference > 180)
            newAngle -= 360;
        else if (newAngle - scopeReference < -180)
            newAngle += 360;

        return newAngle;
    }

    @Override // Called every 20ms
    public void periodic() {
        // Prints the position of the swerve module heading in degrees
        SmartDashboard.putNumber("Module " + this.m_moduleNumber + " Position", getHeadingDegrees());
        // Prints the speed of the swerve module
        SmartDashboard.putNumber("Module " + this.m_moduleNumber + " Speed", getDriveMetersPerSecond());
    }
}