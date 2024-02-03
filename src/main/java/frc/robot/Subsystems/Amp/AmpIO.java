package frc.robot.Subsystems.Amp;



public interface AmpIO {
    public static class AmpIOinputs{
        public double motorposition = 0.0;

    }
    public default void run(double rotationAmount) {}
    public default void stop() {}
    public default void updateInputs(AMPIOInputs inputs) {}


}
