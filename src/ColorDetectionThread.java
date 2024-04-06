import lejos.hardware.Sound;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class ColorDetectionThread extends Thread {
    private EV3ColorSensor colorSensor;
    private SampleProvider intensityProvider;
    private float[] sample;
    private boolean lineDetected;

    private final float MIN_INTENSITY = 30; // Minimum intensity threshold
    private final float MAX_INTENSITY = 40; // Maximum intensity threshold

    public ColorDetectionThread(EV3ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
        intensityProvider = colorSensor.getRedMode();
        sample = new float[intensityProvider.sampleSize()];
    }

    @Override
    public void run() {
        try {
            while (!Thread.interrupted()) {
                // Read reflected light intensity from the color sensor
                intensityProvider.fetchSample(sample, 0);
                float intensity = sample[0] * 100; // Scale intensity to 0-100 range

                // Update line detection status
                if (intensity >= MIN_INTENSITY && intensity <= MAX_INTENSITY) {
                    setLineDetected(true); // Call setLineDetected on the current instance
                } else {
                    setLineDetected(false); // Call setLineDetected on the current instance
                }

                Delay.msDelay(10);
            }
        } finally {
            // Close the color sensor when the thread ends
            colorSensor.close();
        }
    }

    public synchronized boolean isLineDetected() {
        return lineDetected;
    }

    private synchronized void setLineDetected(boolean lineDetected) {
        this.lineDetected = lineDetected;
    }
    
    public synchronized float getIntensity() {
        return sample[0] * 100; 
    }
}
