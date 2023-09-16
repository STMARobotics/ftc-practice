package org.firstinspires.ftc.teamcode.commands;

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
        double strafeSpeed = applyDeadband(-gamepad.getLeftX());
        double forwardSpeed = applyDeadband(-gamepad.getLeftY());
        double rotation = applyDeadband(-gamepad.getRightX());

        drivetrainSubsystem.driveFieldCentric(strafeSpeed, forwardSpeed, rotation, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private static double applyDeadband(double value) {
        if (Math.abs(value) > 0.5) {
            return value;
        }
        return 0.0;
    }

    @Override
    protected void finalize() throws Throwable {
        drivetrainSubsystem.driveFieldCentric(0.0, 0.0, 0.0, false);
    }
}
