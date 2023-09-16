package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@TeleOp(name="Teleop")
public class TeleopOpMode extends CommandOpMode {

    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);

        // Create subsystems
        final DrivetrainSubsystem drivetrainSubsystem =
                new DrivetrainSubsystem(hardwareMap, telemetry);

        // Register subsystems
        register(drivetrainSubsystem);

        // Create and set subsystem default commands, for when nothing else is running
        final TeleopDriveCommand teleopDriveCommand =
                new TeleopDriveCommand(drivetrainSubsystem, driverGamepad);
        drivetrainSubsystem.setDefaultCommand(teleopDriveCommand);

        // Bind gamepad buttons
        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenActive(new InstantCommand(drivetrainSubsystem::resetYaw));
    }

}
