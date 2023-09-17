package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

@Config
public class DrivetrainSubsystem extends SubsystemBase {
    public static final double MAX_VELOCITY = 0.65;
    public static final double MAX_ROTATION_VELOCITY = Math.PI / 2;
    private static final double WHEEL_DIAMETER = 0.098; // 98mm
    private static final double DISTANCE_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER ;
    public static double kP = 1.1;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kS = 1.0;
    public static double kV = 2.8;
    private final MotorEx leftFrontMotor;
    private final MotorEx leftBackMotor;
    private final MotorEx rightFrontMotor;
    private final MotorEx rightBackMotor;
    private final BHI260IMU imu;
    private final MecanumDriveOdometry odometry;
    private final ElapsedTime timer = new ElapsedTime();
    private final MultipleTelemetry telemetry;
    private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
            new Translation2d(0.1275,0.205),
            new Translation2d(0.1275, -0.205),
            new Translation2d(-0.1275, 0.205),
            new Translation2d(-0.1275, -0.205));

    public DrivetrainSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Create the IMU / gyro to read the robot's angle
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(imuParameters);

        // Create the drive motors
        leftFrontMotor = configureMotor(hardwareMap, "left_front_drive", true);
        leftBackMotor = configureMotor(hardwareMap,"left_back_drive", true);
        rightFrontMotor = configureMotor(hardwareMap, "right_front_drive", false);
        rightBackMotor = configureMotor(hardwareMap, "right_back_drive", false);

        // Create the odometry object that will track the robot's pose on the field
        odometry = new MecanumDriveOdometry(kinematics, getGyroAngle(), new Pose2d());
    }

    private static MotorEx configureMotor(HardwareMap hardwareMap, String name, boolean inverted) {
        MotorEx motor = new MotorEx(hardwareMap, name);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setVeloCoefficients(kP, kI, kD);

        double distancePerPulse = (DISTANCE_PER_REVOLUTION / motor.getCPR());
        motor.setDistancePerPulse(distancePerPulse);
        motor.setInverted(inverted);
        motor.resetEncoder();
        return motor;
    }

    @Override
    public void periodic() {
        // Get the wheel speeds
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
                leftFrontMotor.getRate(), rightFrontMotor.getRate(),
                leftBackMotor.getRate(), rightBackMotor.getRate());

        // Update odometry to calculate the current pose
        odometry.updateWithTime(timer.time(), getGyroAngle(), wheelSpeeds);

        Pose2d pose = getPose();
        telemetry.addData("Pose", String.format(
                Locale.getDefault(),
                "(%.3f, %.3f) %.2f degrees",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees()));
        telemetry.addData("Rate Left Front", leftFrontMotor.getRate());
        telemetry.addData("Rate Right Front", rightFrontMotor.getRate());
        telemetry.addData("Rate Left Back", leftBackMotor.getRate());
        telemetry.addData("Rate Right Back", rightBackMotor.getRate());
        telemetry.addData("Distance Left Front", leftFrontMotor.getDistance());
        telemetry.addData("Distance Right Front", rightFrontMotor.getDistance());
        telemetry.addData("Distance Left Back", leftBackMotor.getDistance());
        telemetry.addData("Distance Right Back", rightBackMotor.getDistance());
        telemetry.addData("Rotation Velocity", imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate);
        telemetry.addData("Time", timer.time());
        telemetry.addData("Heading", getGyroAngle().getDegrees());
        telemetry.update();
    }

    /**
     * Drive the robot from the perspective of the robot itself.
     * @param strafeSpeed
     * @param forwardSpeed
     * @param rotation
     */
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double rotation) {
        drive(new ChassisSpeeds(forwardSpeed, strafeSpeed, rotation));
    }

    public void driveFieldCentric(
            double strafeSpeed, double forwardSpeed, double rotation) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                forwardSpeed, strafeSpeed, rotation, getGyroAngle());

        leftFrontMotor.setVeloCoefficients(kP, kI, kD);
        leftBackMotor.setVeloCoefficients(kP, kI, kD);
        rightFrontMotor.setVeloCoefficients(kP, kI, kD);
        rightBackMotor.setVeloCoefficients(kP, kI, kD);
        leftFrontMotor.setFeedforwardCoefficients(kS, kV);
        leftBackMotor.setFeedforwardCoefficients(kS, kV);
        rightFrontMotor.setFeedforwardCoefficients(kS, kV);
        rightBackMotor.setFeedforwardCoefficients(kS, kV);
        drive(chassisSpeeds);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        wheelSpeeds.normalize(MAX_VELOCITY);

        leftFrontMotor.set(wheelSpeeds.frontLeftMetersPerSecond);
        leftBackMotor.set(wheelSpeeds.rearLeftMetersPerSecond);
        rightFrontMotor.set(wheelSpeeds.frontRightMetersPerSecond);
        rightBackMotor.set(wheelSpeeds.rearRightMetersPerSecond);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getGyroAngle() {
        return new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    public void resetYaw() {
        imu.resetYaw();
    }
}
