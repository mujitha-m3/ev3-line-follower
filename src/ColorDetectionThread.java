import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class ColorDetectionThread extends Thread {
    private EV3ColorSensor colorSensor;
    private int lineColorId;

    public ColorDetectionThread(EV3ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }

    @Override
    public void run() {
        SampleProvider colorProvider = colorSensor.getColorIDMode();
        float[] sample = new float[colorProvider.sampleSize()];

        while (!Thread.currentThread().isInterrupted()) {
            colorProvider.fetchSample(sample, 0);
            setLineColorId((int) sample[0]);
        }
        colorSensor.close();
    }

    public synchronized int getLineColorId() {
        return lineColorId;
    }

    private synchronized void setLineColorId(int lineColorId) {
        this.lineColorId = lineColorId;
    }
}
