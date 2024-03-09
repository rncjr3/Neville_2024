package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static final class ModuleConstants{
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1/6.75;
        public static final double kTurningMotorGearRatio = 7.00 / 150.00;//150.00 / 7.00;
        public static final double kDriveEncoderRot2Meter  = kDriveMotorGearRatio * kWheelDiameterMeters * Math.PI;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2.0 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
        public static final double kPTurning = 0/5; 
    }

    public static final class DriveConstants{
        public static final double kTrackWidth = Units.inchesToMeters(24);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(24);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Left Front
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Right Front
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Left rear
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Right rear

            public static final int kFrontLeftDriveMotorPort = 6;
            public static final int kBackLeftDriveMotorPort = 8;
            public static final int kFrontRightDriveMotorPort = 1;
            public static final int kBackRightDriveMotorPort = 3;
            public static final int kFrontLeftTurningMotorPort = 5;
            public static final int kBackLeftTurningMotorPort = 7;
            public static final int kFrontRightTurningMotorPort = 2;
            public static final int kBackRightTurningMotorPort = 4;

            public static final boolean kFrontLeftTurningEncoderReversed = true;
            public static final boolean kBackLeftTurningEncoderReversed = true;
            public static final boolean kFrontRightTurningEncoderReversed = true;
            public static final boolean kBackRightTurningEncoderReversed = true;

            public static final boolean kFrontLeftDriveEncoderReversed = true;
            public static final boolean kBackLeftDriveEncoderReversed = true;
            public static final boolean kFrontRightDriveEncoderReversed = false;
            public static final boolean kBackRightDriveEncoderReversed = false;

            public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;
            public static final int kBackLeftDriveAbsoluteEncoderPort = 13;
            public static final int kFrontRightDriveAbsoluteEncoderPort = 11;
            public static final int kBackRightDriveAbsoluteEncoderPort = 10;

            public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
            public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
            public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
            public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

            public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0 * Math.PI/180.0;
            public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0 * Math.PI/180.0;
            public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0 * Math.PI/180.0;
            public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0 * Math.PI/180.0;

            public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
            public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

            public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
            public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
            public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
            public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

            //Shooters and hang constants go here.
            public static final double kSLeft = 0.097576;
            public static final double kVLeft = 2.6933;
            public static final double kALeft = 0.26236;

            public static final double kSRight = 0.099437;
            public static final double kVRight = 2.6173;
            public static final double kARight = 0.11195;

            //Drive/Rotation gain
            public static final double kRotGain = 2;
            public static final double kDriveGain = 3;

    }

    public static final class AutoConstants{
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0.1;
        public static final double kPYController = 0.1;
        public static final double kPThetaController = 10;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants{
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 4;
        public static final int kDriverRotAxis = 0;

        public static final int kDriverFieldOrientedButtonIdx = 5;

        public static final double kDeadband = 0.1;
    }

    public static class OperatorConstants{
        public static final int kOperatorControllerPort = 1;
    }

    public static final class BalanceConstants{
        public static final double balanceP = 0.0275;
        public static final double balanceI = 0;
        public static final double balanceD = .01;
    }

}
