package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
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

public class DrivetrainSubsystem extends SubsystemBase {
    private static final double WHEEL_DIAMETER = 0.098; // 98mm
    private static final double DISTANCE_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER ;
    private final MotorEx leftFrontMotor;
    private final MotorEx leftBackMotor;
    private final MotorEx rightFrontMotor;
    private final MotorEx rightBackMotor;
    private final BHI260IMU imu;
    private final MecanumDrive mecanumDrive;
    private final MecanumDriveOdometry odometry;
    private final ElapsedTime timer = new ElapsedTime();
    private final Telemetry telemetry;
    private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
            new Translation2d(0.1275,0.205),
            new Translation2d(0.1275, -0.205),
            new Translation2d(-0.1275, 0.205),
            new Translation2d(-0.1275, -0.205));

    public DrivetrainSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
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
        leftFrontMotor  = new MotorEx(hardwareMap, "left_front_drive");
        leftBackMotor  = new MotorEx(hardwareMap,"left_back_drive");
        rightFrontMotor = new MotorEx(hardwareMap, "right_front_drive");
        rightBackMotor = new MotorEx(hardwareMap, "right_back_drive");

        double distancePerPulse = DISTANCE_PER_REVOLUTION / leftFrontMotor.getCPR();
        leftFrontMotor.setDistancePerPulse(-distancePerPulse);
        leftBackMotor.setDistancePerPulse(-distancePerPulse);
        rightFrontMotor.setDistancePerPulse(distancePerPulse);
        rightBackMotor.setDistancePerPulse(distancePerPulse);
        leftFrontMotor.resetEncoder();
        leftBackMotor.resetEncoder();
        rightFrontMotor.resetEncoder();
        rightBackMotor.resetEncoder();

        // Create the drive object with the motors
        mecanumDrive = new MecanumDrive(
                leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);

        // Create the odometry object that will track the robot's pose on the field
        odometry = new MecanumDriveOdometry(kinematics, getGyroAngle(), new Pose2d());
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
        telemetry.addData("Left Front Rate", leftFrontMotor.getRate());
        telemetry.addData("Right Front Rate", rightFrontMotor.getRate());
        telemetry.addData("Left Back Rate", leftBackMotor.getRate());
        telemetry.addData("Right Back Rate", rightBackMotor.getRate());
        telemetry.addData("Left Front Distance", leftFrontMotor.getDistance());
        telemetry.addData("Right Front Distance", rightFrontMotor.getDistance());
        telemetry.addData("Left Back Distance", leftBackMotor.getDistance());
        telemetry.addData("Right Back Distance", rightBackMotor.getDistance());
        telemetry.addData("Time", timer.time());
        telemetry.addData("Heading", getGyroAngle().getDegrees());
        telemetry.update();
    }

    /**
     * Drive the robot from the perspective of the robot itself.
     * @param strafeSpeed
     * @param forwardSpeed
     * @param rotation
     * @param squareInputs
     */
    public void driveRobotCentric(
            double strafeSpeed, double forwardSpeed, double rotation, boolean squareInputs) {
        mecanumDrive.driveRobotCentric(strafeSpeed, forwardSpeed, rotation, squareInputs);
    }

    public void driveFieldCentric(
            double strafeSpeed, double forwardSpeed, double rotation, boolean squareInputs) {

        mecanumDrive.driveFieldCentric(
                strafeSpeed, forwardSpeed, rotation, getGyroAngle().getDegrees(), squareInputs);
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
