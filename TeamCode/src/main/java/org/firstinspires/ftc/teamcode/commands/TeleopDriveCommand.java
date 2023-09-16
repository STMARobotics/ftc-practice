package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem.MAX_ROTATION_VELOCITY;
import static org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem.MAX_VELOCITY;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class TeleopDriveCommand extends CommandBase {

    private final DrivetrainSubsystem drivetrainSubsystem;

    private final GamepadEx gamepad;

    public TeleopDriveCommand(DrivetrainSubsystem drivetrainSubsystem, GamepadEx gamepad) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.gamepad = gamepad;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double strafeSpeed = modifyAxis(-gamepad.getLeftX()) * MAX_VELOCITY;
        double forwardSpeed = modifyAxis(-gamepad.getLeftY()) * MAX_VELOCITY;
        double rotation = modifyAxis(-gamepad.getRightX()) * MAX_ROTATION_VELOCITY;

        drivetrainSubsystem.driveFieldCentric(strafeSpeed, forwardSpeed, rotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Apply deadband to center of axis and square for finer control at slow speeds
     * @param value raw value
     * @return deadbanded and squared value
     */
    private static double modifyAxis(double value) {
        double abs = Math.abs(value);
        if (abs > 0.5) {
            return value * abs;
        }
        return 0.0;
    }

    @Override
    protected void finalize() throws Throwable {
        drivetrainSubsystem.driveFieldCentric(0.0, 0.0, 0.0);
    }
}
