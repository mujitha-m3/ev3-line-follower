import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class ObstacleDetectionThread extends Thread {
    private EV3UltrasonicSensor ultrasonicSensor;
    private boolean obstacleDetected;
    private int obstacleCount; // Variable to count obstacles
    private static final int OBSTACLE_THRESHOLD = 15; // Threshold distance for obstacle detection

    public ObstacleDetectionThread(EV3UltrasonicSensor ultrasonicSensor) {
        this.ultrasonicSensor = ultrasonicSensor;
        this.obstacleDetected = false;
        this.obstacleCount = 0; // Initialize obstacle count
    }

    @Override
    public void run() {
        SampleProvider distanceProvider = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distanceProvider.sampleSize()];

        while (!Thread.currentThread().isInterrupted()) {
            // Read distance using the ultrasonic sensor
            distanceProvider.fetchSample(sample, 0);
            int distance = (int) (sample[0] * 100); // Convert to centimeters

            if (distance < OBSTACLE_THRESHOLD) {
                setObstacleDetected(true);
                System.out.println("Obstacle detected...");
                obstacleCount++; 
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
    
    public synchronized int getObstacleCount() {
        return obstacleCount; 
    }
}
