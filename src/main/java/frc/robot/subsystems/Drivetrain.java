package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private final SwerveModule m_FL;
    private final SwerveModule m_FR;
    private final SwerveModule m_RL;
    private final SwerveModule m_RR;

    public Drivetrain() {
        this.m_FL = new SwerveModule(Constants.Swerve.FL.kDriveMotorID, Constants.Swerve.FL.kAngleMotorID,
                Constants.Swerve.FL.kAngleAbsoluteEncoderID, Constants.Swerve.FL.kAngleAbsoluteEncoderOffset);
        this.m_FR = new SwerveModule(Constants.Swerve.FR.kDriveMotorID, Constants.Swerve.FR.kAngleMotorID,
                Constants.Swerve.FR.kAngleAbsoluteEncoderID, Constants.Swerve.FR.kAngleAbsoluteEncoderOffset);
        this.m_RL = new SwerveModule(Constants.Swerve.RL.kDriveMotorID, Constants.Swerve.RL.kAngleMotorID,
                Constants.Swerve.RL.kAngleAbsoluteEncoderID, Constants.Swerve.RL.kAngleAbsoluteEncoderOffset);
        this.m_RR = new SwerveModule(Constants.Swerve.RR.kDriveMotorID, Constants.Swerve.RR.kAngleMotorID,
                Constants.Swerve.RR.kAngleAbsoluteEncoderID, Constants.Swerve.RR.kAngleAbsoluteEncoderOffset);
    }

    public SwerveModule getFL() {
        return this.m_FL;
    }

    public SwerveModule getFR() {
        return this.m_FR;
    }

    public SwerveModule getRL() {
        return this.m_RL;
    }

    public SwerveModule getRR() {
        return this.m_RR;
    }
}
