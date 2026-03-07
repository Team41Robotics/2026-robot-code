package frc.robot.commands.shooter;

import static edu.wpi.first.math.MathUtil.*;
import static frc.robot.RobotContainer.*;
import static java.lang.Math.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.Util;

@SuppressWarnings("static-access")
public class ManualShoot extends Command {
	public static final double HOOD_INCREMENT = 5 / 180. * PI;
	public static final double FLYWHEEL_INCREMENT = 100;

	public boolean prevPovUp = false;
	public boolean prevPovDown = false;
	public boolean prevRightPovUp = false;
	public boolean prevRightPovDown = false;

	public ManualShoot() {
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		// Auto turret targeting
		Translation2d hubCenter = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
		Translation2d target = Util.flipIfRed(hubCenter);
		Translation2d virtualTarget = Targetting.shootOnTheFly(target);
		double fieldAngle = Targetting.shotAngle(virtualTarget);
		shooter.targetTurretPos =
				angleModulus(fieldAngle - drive.pose.getRotation().getRadians() + PI);

		// Left POV: hood
		boolean povUp = controls.pov(45).getAsBoolean();
		boolean povDown = controls.pov(180).getAsBoolean();

		if (povUp && !prevPovUp) shooter.targetHoodPos += HOOD_INCREMENT;
		else if (povDown && !prevPovDown) shooter.targetHoodPos -= HOOD_INCREMENT;

		prevPovUp = povUp;
		prevPovDown = povDown;

		// Right POV: flywheel RPM
		boolean rightPovUp = controls.rightPov(0).getAsBoolean();
		boolean rightPovDown = controls.rightPov(180).getAsBoolean();

		if (rightPovUp && !prevRightPovUp) shooter.targetFlywheelRPM += FLYWHEEL_INCREMENT;
		else if (rightPovDown && !prevRightPovDown) shooter.targetFlywheelRPM -= FLYWHEEL_INCREMENT;

		prevRightPovUp = rightPovUp;
		prevRightPovDown = rightPovDown;
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
