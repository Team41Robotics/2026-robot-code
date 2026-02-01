// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystem.drive.SwerveDriveSubsystem;
import frc.robot.subsystem.drive.TurnSysID;

public class RobotContainer {
	public static SwerveDriveSubsystem drive = new SwerveDriveSubsystem();

	public static CommandJoystick left_js = new CommandJoystick(3);
	public static CommandJoystick right_js = new CommandJoystick(4);
	public static CommandJoystick ds = new CommandJoystick(2);

	public static void init() {
		drive.init();

		// DriveSysID sysid = new DriveSysID();
		TurnSysID sysid = new TurnSysID();
		sysid.init();
	}

	public static Command getAutonomousCommand() {
		return null;
	}
}
