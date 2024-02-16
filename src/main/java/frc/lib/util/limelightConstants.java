package frc.lib.util;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class limelightConstants {
    public final String name;
    public final double verticalFOV;
    public final double horizontalFOV;
    public final double limelightMountHeight;
    public final int detectorPiplineIndex; 
    public final int apriltagPipelineIndex;
    public final int horPixels;
    public final double filterTimeConstant; // in seconds, inputs occuring over a time period significantly shorter than this will be thrown out
    public final Vector<N3> visionMeasurementStdDevs;
    public final int movingAverageNumTaps;



    /**
     * Limelight Constants to be used when creating swerve modules.
     * @param name
     * @param verticalFOV
     * @param horizontalFov
     * @param limelightmountheight
     * @param detectorPipelineIndex
     * @param apriltagPiplineIndex
     * @param horpixels
     * @param filterTimeConstant
     * @param visionMeasurementStdDevs
     * @param movingAverageNumTaps
     */

    public limelightConstants(String name, double verticalFOV, double horizontalFOV, double limelightMountHeight, int detectorPiplineIndex, int apriltagPipelineIndex,
    int horPixels, double filterTimeConstant, Vector<N3> visionMeasurementStdDevs, int movingAverageNumTaps) {
        this.name = name;
        this.verticalFOV = verticalFOV; //degrees obviously
        this.horizontalFOV = horizontalFOV;
        this.limelightMountHeight = limelightMountHeight;
        this.detectorPiplineIndex = detectorPiplineIndex; 
        this.apriltagPipelineIndex = apriltagPipelineIndex;
        this.horPixels = horPixels;
        this.filterTimeConstant = filterTimeConstant; // in seconds, inputs occuring over a time period significantly shorter than this will be thrown out
        this.visionMeasurementStdDevs = visionMeasurementStdDevs;
        this.movingAverageNumTaps = movingAverageNumTaps;

    
    }
}
