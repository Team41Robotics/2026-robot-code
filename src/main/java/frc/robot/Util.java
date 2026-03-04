package frc.robot;

import static java.lang.Math.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Util {
	/** Build a 3x1 covariance matrix from standard deviations (squares them for you). */
	public static Matrix<N3, N1> buildCov(double stdX, double stdY, double stdTheta) {
		return VecBuilder.fill(stdX * stdX, stdY * stdY, stdTheta * stdTheta);
	}

	public static double deadband(double x, double db) {
		if (abs(x) < db) return 0;
		return copySign((abs(x) - db) / (1 - db), x);
	}

	public static double squareCurve(double x) {
		return copySign(x * x, x);
	}
}
