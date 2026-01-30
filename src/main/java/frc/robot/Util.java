package frc.robot;

import static java.lang.Math.*;

public class Util {
	public static double deadband(double x, double d) {
		if (abs(x) < d) return 0;
		return copySign(((abs(x) - d) / (1 - d)), x);
	}

	public static double squareCurve(double x) {
		return copySign(x * x, x);
	}
}
