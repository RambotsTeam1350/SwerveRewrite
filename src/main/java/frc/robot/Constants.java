package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class Controllers {
        public static final class Driver {
            public static final int kPort = 0;
            public static final double kTranslationDeadband = 0.05;
            public static final double kRotationDeadband = 0.1;
        }
    }

    public static final class Swerve {
        public static final class MaxSpeeds {
            public static final double kTranslation = 4.5; // m/s
            public static final double kRotation = Math.PI; // rad/s
        }

        public static final class SlewRateLimits {
            public static final double kForwardsSlewRateLimit = 2;
            public static final double kStrafeSlewRateLimit = 2;
            public static final double kRotateSlewRateLimit = 3;
        }

        public static final class CurrentLimits {
            public static final int kDriveMotor = 60;
            public static final int kAngleMotor = 20;
        }

        public static final class Drive {
            public static final double kGearRatio = (36.0 / 14.0) * (18.0 / 24.0) * (45.0 / 15.0);
            public static final double kWheelDiameter = 0.096706557;
            public static final double kRevolutionsToMeters = ((kWheelDiameter * Math.PI) / kGearRatio);
            public static final double kRPMtoMPS = kRevolutionsToMeters / 60.0;
            public static final double kTrackWidth = Units.inchesToMeters(25.5);
            public static final double kWheelBase = Units.inchesToMeters(25.5);
        }

        public static final class Angle {
            public static final double kP = 0.005;
            public static final double kGearRatio = (50.0 / 14.0) * (72.0 / 14.0);
            public static final double kRevolutionsToDegrees = 360.0 / kGearRatio;
        }

        public static final Translation2d[] kModuleTranslations = {
                new Translation2d(Drive.kWheelBase / 2.0, Drive.kTrackWidth / 2.0),
                new Translation2d(Drive.kWheelBase / 2.0, -Drive.kTrackWidth / 2.0),
                new Translation2d(-Drive.kWheelBase / 2.0, Drive.kTrackWidth / 2.0),
                new Translation2d(-Drive.kWheelBase / 2.0, -Drive.kTrackWidth / 2.0) };
        public static final SwerveDriveKinematics kSwerveDriveKinematics = new SwerveDriveKinematics(
                kModuleTranslations);

        public static final class FL {
            public static final int kDriveMotorID = 41;
            public static final int kAngleMotorID = 42;
            public static final boolean kDriveInvert = false;
            public static final boolean kAngleInvert = false;
            public static final int kAngleAbsoluteEncoderID = 4;
            public static final double kAngleAbsoluteEncoderOffset = -0.248779 * 360.0;
        }

        public static final class FR {
            public static final int kDriveMotorID = 11;
            public static final int kAngleMotorID = 12;
            public static final boolean kDriveInvert = false;
            public static final boolean kAngleInvert = false;
            public static final int kAngleAbsoluteEncoderID = 1;
            public static final double kAngleAbsoluteEncoderOffset = -0.372559 * 360.0;
        }

        public static final class RL {
            public static final int kDriveMotorID = 31;
            public static final int kAngleMotorID = 32;
            public static final boolean kDriveInvert = false;
            public static final boolean kAngleInvert = false;
            public static final int kAngleAbsoluteEncoderID = 3;
            public static final double kAngleAbsoluteEncoderOffset = 0.325195 * 360.0;
        }

        public static final class RR {
            public static final int kDriveMotorID = 21;
            public static final int kAngleMotorID = 22;
            public static final boolean kDriveInvert = false;
            public static final boolean kAngleInvert = false;
            public static final int kAngleAbsoluteEncoderID = 2;
            public static final double kAngleAbsoluteEncoderOffset = 0.109131 * 360.0;
        }

        public enum ModulePosition {
            FL,
            FR,
            RL,
            RR
        }
    }
}
