package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class Controllers {
        public static final class Driver {
            public static final int kPort = 0;
            public static final double kDeadband = 0.05;
        }
    }

    public static final class Swerve {
        public static final class SlewRateLimits {
            public static final double kForwardsSlewRateLimit = 0.05;
            public static final double kStrafeSlewRateLimit = 0.05;
            public static final double kRotateClockwiseSlewRateLimit = 0.05;
        }

        public static final class CurrentLimits {
            public static final int kDriveMotor = 60;
            public static final int kAngleMotor = 20;
        }

        public static final class Drive {
            public static final double kGearRatio = (36.0 / 14.0) * (18.0 / 24.0) * (45.0 / 15.0);
            public static final double kWheelDiameter = 0.096706557;
            public static final double kTrackWidth = Units.inchesToMeters(25.5);
            public static final double kWheelBase = Units.inchesToMeters(25.5);
        }

        public static final class Angle {
            public static final double kP = 0.005;
            public static final double kGearRatio = (50.0 / 14.0) * (72.0 / 14.0);
        }

        public static final SwerveDriveKinematics kSwerveDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(Drive.kWheelBase / 2.0, Drive.kTrackWidth / 2.0),
                new Translation2d(Drive.kWheelBase / 2.0, -Drive.kTrackWidth / 2.0),
                new Translation2d(-Drive.kWheelBase / 2.0, Drive.kTrackWidth / 2.0),
                new Translation2d(-Drive.kWheelBase / 2.0, -Drive.kTrackWidth / 2.0));

        public static final class FL {
            public static final int kDriveMotorID = 41;
            public static final int kAngleMotorID = 42;
            public static final int kAngleAbsoluteEncoderID = 4;
            public static final double kAngleAbsoluteEncoderOffset = 0.0;
        }

        public static final class FR {
            public static final int kDriveMotorID = 31;
            public static final int kAngleMotorID = 32;
            public static final int kAngleAbsoluteEncoderID = 3;
            public static final double kAngleAbsoluteEncoderOffset = 0.0;
        }

        public static final class RL {
            public static final int kDriveMotorID = 11;
            public static final int kAngleMotorID = 12;
            public static final int kAngleAbsoluteEncoderID = 1;
            public static final double kAngleAbsoluteEncoderOffset = 0.0;
        }

        public static final class RR {
            public static final int kDriveMotorID = 21;
            public static final int kAngleMotorID = 22;
            public static final int kAngleAbsoluteEncoderID = 2;
            public static final double kAngleAbsoluteEncoderOffset = 0.0;
        }
    }
}
