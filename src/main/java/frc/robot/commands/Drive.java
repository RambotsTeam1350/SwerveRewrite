package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Drive extends Command {
    private final CommandXboxController m_controller;
    private final Drivetrain m_drivetrain;

    private final SlewRateLimiter m_strafeSlewRateLimiter;
    private final SlewRateLimiter m_forwardSlewRateLimiter;
    private final SlewRateLimiter m_rotateClockwiseSlewRateLimiter;

    public Drive(CommandXboxController controller, Drivetrain drivetrain) {
        this.m_controller = controller;

        this.m_drivetrain = drivetrain;

        this.m_strafeSlewRateLimiter = new SlewRateLimiter(Constants.Swerve.SlewRateLimits.kStrafeSlewRateLimit);
        this.m_forwardSlewRateLimiter = new SlewRateLimiter(Constants.Swerve.SlewRateLimits.kForwardsSlewRateLimit);
        this.m_rotateClockwiseSlewRateLimiter = new SlewRateLimiter(
                Constants.Swerve.SlewRateLimits.kRotateClockwiseSlewRateLimit);

        addRequirements(this.m_drivetrain);
    }

    @Override
    public void execute() {
        double forwardInput = this.m_controller.getLeftY();
        if (forwardInput < Constants.Controllers.Driver.kDeadband
                && forwardInput > -Constants.Controllers.Driver.kDeadband) {
            forwardInput = 0;
        }

        double strafeInput = this.m_controller.getLeftX();
        if (strafeInput < Constants.Controllers.Driver.kDeadband
                && strafeInput > -Constants.Controllers.Driver.kDeadband) {
            strafeInput = 0;
        }

        double rotateClockwiseInput = this.m_controller.getRightX();
        if (rotateClockwiseInput < Constants.Controllers.Driver.kDeadband
                && rotateClockwiseInput > -Constants.Controllers.Driver.kDeadband) {
            rotateClockwiseInput = 0;
        }

        forwardInput = this.m_forwardSlewRateLimiter.calculate(forwardInput);
        strafeInput = this.m_strafeSlewRateLimiter.calculate(strafeInput);
        rotateClockwiseInput = this.m_rotateClockwiseSlewRateLimiter.calculate(rotateClockwiseInput);

        final double length = Units.metersToInches(Constants.Swerve.Drive.kTrackWidth);
        final double width = Units.metersToInches(Constants.Swerve.Drive.kWheelBase);
        final double angleAdjustment = Math.sqrt((Math.pow(length, 2)) + (Math.pow(width, 2)));

        double A = ((strafeInput - rotateClockwiseInput) * (length / angleAdjustment));
        double B = ((strafeInput + rotateClockwiseInput) * (length / angleAdjustment));
        double C = ((forwardInput - rotateClockwiseInput) * (width / angleAdjustment));
        double D = ((forwardInput + rotateClockwiseInput) * (width / angleAdjustment));

        double FLWheelSpeed = Math.sqrt((Math.pow(B, 2)) + (Math.pow(D, 2)));
        double FRWheelSpeed = Math.sqrt((Math.pow(B, 2)) + (Math.pow(C, 2)));
        double RLWheelSpeed = Math.sqrt((Math.pow(A, 2)) + (Math.pow(D, 2)));
        double RRWheelSpeed = Math.sqrt((Math.pow(A, 2)) + (Math.pow(C, 2)));

        double FLWheelAngle = (Math.atan2(B, D)) * (180 / Math.PI);
        double FRWheelAngle = (Math.atan2(B, C)) * (180 / Math.PI);
        double RLWheelAngle = (Math.atan2(A, D)) * (180 / Math.PI);
        double RRWheelAngle = (Math.atan2(A, C)) * (180 / Math.PI);

        double maxWheelSpeed = FLWheelSpeed;
        if (FRWheelSpeed > maxWheelSpeed) maxWheelSpeed = FRWheelSpeed;
        if (RLWheelSpeed > maxWheelSpeed) maxWheelSpeed = RLWheelSpeed;
        if (RRWheelSpeed > maxWheelSpeed) maxWheelSpeed = RRWheelSpeed;
        if (maxWheelSpeed > 1) {
            FLWheelSpeed /= maxWheelSpeed;
            FRWheelAngle /= maxWheelSpeed;
            RLWheelSpeed /= maxWheelSpeed;
            RRWheelSpeed /= maxWheelSpeed;
        }

        this.m_drivetrain.getFL().setDriveMotor(FLWheelSpeed);
        this.m_drivetrain.getFR().setDriveMotor(FRWheelSpeed);
        this.m_drivetrain.getRL().setDriveMotor(RLWheelSpeed);
        this.m_drivetrain.getRR().setDriveMotor(RRWheelSpeed);

        this.m_drivetrain.getFL().setAngleMotor((this.m_drivetrain.getFL().getAngleAbsoluteEncoder() - FLWheelAngle) * Constants.Swerve.Angle.kP);
        this.m_drivetrain.getFR().setAngleMotor((this.m_drivetrain.getFR().getAngleAbsoluteEncoder() - FRWheelAngle) * Constants.Swerve.Angle.kP);
        this.m_drivetrain.getRL().setAngleMotor((this.m_drivetrain.getRL().getAngleAbsoluteEncoder() - RLWheelAngle) * Constants.Swerve.Angle.kP);
        this.m_drivetrain.getRR().setAngleMotor((this.m_drivetrain.getRR().getAngleAbsoluteEncoder() - RRWheelAngle) * Constants.Swerve.Angle.kP);
    }
}
