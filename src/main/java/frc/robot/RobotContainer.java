// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drive.SwerveDriveSubsystem;

public class RobotContainer {
	public static SwerveDriveSubsystem drive = new SwerveDriveSubsystem();

	public static void init() {
		drive.init();
	}

	public static Command getAutonomousCommand() {
		return null;
	}
}
