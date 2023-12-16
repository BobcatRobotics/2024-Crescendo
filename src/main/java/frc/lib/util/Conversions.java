package frc.lib.util;

public class Conversions {
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double RPS = motorRPM / 60.0;
        return RPS;
    }

    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * 60;        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / gearRatio);
    }

    public static double falconToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / gearRatio);
    }

    public static double falconToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * (circumference / gearRatio);
    }
}
