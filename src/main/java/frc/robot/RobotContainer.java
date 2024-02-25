package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    private final CommandXboxController m_driver;

    private final Drivetrain m_Drivetrain;

    private final Drive m_Drive;

    public RobotContainer() {
        this.m_driver = new CommandXboxController(Constants.Controllers.Driver.kPort);

        this.m_Drivetrain = new Drivetrain();

        this.m_Drive = new Drive(this.m_driver, this.m_Drivetrain);

        this.m_Drivetrain.setDefaultCommand(m_Drive);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
    }
}
