package frc.lib.util;

public class SwerveConversions {
    public static double MPSToRPS(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToRPS(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    public static double RPSToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = RPSToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    public static double RPMToRPS(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double RPS = motorRPM / 60.0;
        return RPS;
    }

    public static double RPSToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * 60;        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    public static double degreesToRotations(double degrees, double gearRatio) {
        return degrees / (360.0 / gearRatio);
    }

    public static double rotationsToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / gearRatio);
    }

    public static double rotationsToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * (circumference / gearRatio);
    }
}
