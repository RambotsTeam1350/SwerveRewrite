package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class SwerveModule {
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_angleMotor;
    
    private final RelativeEncoder m_driveRelativeEncoder;
    private final RelativeEncoder m_angleRelativeEncoder;

    private final CANcoder m_angleAbsoluteEncoder;

    public SwerveModule(int driveMotorID, int angleMotorID, int absoluteEncoderID, double absoluteEncoderOffset) {
        this.m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        this.m_angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        
        this.m_driveRelativeEncoder = this.m_driveMotor.getEncoder();
        this.m_angleRelativeEncoder = this.m_angleMotor.getEncoder();

        this.m_angleAbsoluteEncoder = new CANcoder(absoluteEncoderID);
    }

    public void setDriveMotor(double speed) {
        this.m_driveMotor.set(speed);
    }

    public void setAngleMotor(double speed) {
        this.m_angleMotor.set(speed);
    }

    public double getAngleAbsoluteEncoder() {
        return this.m_angleAbsoluteEncoder.getPosition().getValueAsDouble() * 180.0;
    }
}