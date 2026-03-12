package frc.robot;

import static edu.wpi.first.math.MathUtil.*;
import static java.lang.Math.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Util {
	public static Matrix<N3, N1> buildCov(double stdX, double stdY, double stdTheta) {
		return VecBuilder.fill(stdX, stdY, stdTheta);
	}

	public static double deadband(double x, double db) {
		if (abs(x) < db) return 0;
		return copySign((abs(x) - db) / (1 - db), x);
	}

	public static double squareCurve(double x) {
		return copySign(x * x, x);
	}

	public static double flip(double theta) {
		return angleModulus(theta + PI);
	}

	public static Translation2d flip(Translation2d t) {
		return new Translation2d(FieldConstants.fieldLength - t.getX(), FieldConstants.fieldWidth - t.getY());
	}

	public static Rotation2d flip(Rotation2d r) {
		return r.plus(Rotation2d.kPi);
	}

	public static Pose2d flip(Pose2d p) {
		return new Pose2d(flip(p.getTranslation()), flip(p.getRotation()));
	}

	public static double flipIfRed(double theta) {
		if (RobotContainer.isRed()) {
			return flip(theta);
		}
		return theta;
	}

	public static Translation2d flipIfRed(Translation2d t) {
		if (RobotContainer.isRed()) {
			return flip(t);
		}
		return t;
	}

	public static Rotation2d flipIfRed(Rotation2d r) {
		if (RobotContainer.isRed()) {
			return flip(r);
		}
		return r;
	}

	public static Pose2d flipIfRed(Pose2d p) {
		if (RobotContainer.isRed()) {
			return flip(p);
		}
		return p;
	}
}
