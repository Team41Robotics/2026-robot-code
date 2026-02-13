package frc.robot;

import static java.lang.Math.*;

public class Util {
	public static double deadband(double x, double db) {
		if (abs(x) < db) return 0;
		return copySign(((abs(x) - db) / (1 - db)), x);
	}

	public static double squareCurve(double x) {
		return copySign(x * x, x);
	}
}
