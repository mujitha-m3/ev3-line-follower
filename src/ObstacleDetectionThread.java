import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class ObstacleDetectionThread extends Thread {
    private EV3UltrasonicSensor ultrasonicSensor;
    private boolean obstacleDetected;

    public ObstacleDetectionThread(EV3UltrasonicSensor ultrasonicSensor) {
        this.ultrasonicSensor = ultrasonicSensor;
        this.obstacleDetected = false;
    }

    @Override
    public void run() {
        SampleProvider distanceProvider = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distanceProvider.sampleSize()];

        while (!Thread.currentThread().isInterrupted()) {
            // Read distance using the ultrasonic sensor
            distanceProvider.fetchSample(sample, 0);
            int distance = (int) (sample[0] * 100); // Convert to centimeters

            if (distance < 15) {
                setObstacleDetected(true);
                Sound.beep();
                Sound.beep();
            } else {
                setObstacleDetected(false);
            }

            // Add a small delay to control loop execution frequency
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public synchronized boolean isObstacleDetected() {
        return obstacleDetected;
    }

    private synchronized void setObstacleDetected(boolean obstacleDetected) {
        this.obstacleDetected = obstacleDetected;
    }
}
