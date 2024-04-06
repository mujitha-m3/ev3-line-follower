import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.utility.Delay;
import lejos.hardware.Sound;

public class LineFollowingThread extends Thread {
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private ColorDetectionThread colorDetectionThread;
    private ObstacleDetectionThread obstacleDetectionThread;
    private LineFollowingTimeTracker timeTrackerThread;
    private static final int BASE_SPEED = 50;
    private static final int SEARCH_SPEED = BASE_SPEED / 2;
    private float intensity;

    public LineFollowingThread(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, ColorDetectionThread colorDetectionThread, ObstacleDetectionThread obstacleDetectionThread) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.colorDetectionThread = colorDetectionThread;
        this.obstacleDetectionThread = obstacleDetectionThread;
        this.timeTrackerThread = timeTrackerThread;
       
        // Set motor speeds
        leftMotor.setSpeed(BASE_SPEED);
        rightMotor.setSpeed(BASE_SPEED);
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            boolean lineDetected = colorDetectionThread.isLineDetected();
            boolean obstacleDetected = obstacleDetectionThread.isObstacleDetected();
            int obstacleCount = obstacleDetectionThread.getObstacleCount(); 
            intensity = colorDetectionThread.getIntensity();
            long elapsedTime = timeTrackerThread.getElapsedTime();

            if (obstacleDetected) {
                System.out.println("Obstacle detected");
                if (obstacleCount == 1) {
                    System.out.println("Bypassing the first obstacle...");
                    leftMotor.stop(true);
                    rightMotor.stop();
                    Delay.msDelay(1000); // Stop for 1 second
                    leftMotor.rotate(-90); // Turn left 90 degrees
                    rightMotor.rotate(90);
                    Delay.msDelay(1000); // Move forward for 1 second
                    leftMotor.rotate(90); // Turn right 90 degrees
                    rightMotor.rotate(-90);
                    Delay.msDelay(1000); // Move forward for 1 second
                    System.out.println("Resuming path after bypassing the first obstacle");
                } else if (obstacleCount == 2) {
                    System.out.println("Second obstacle detected. Stopping the robot.");
                    System.out.println("Robot stopped after running for " + elapsedTime / 1000 + " seconds");
                    leftMotor.stop(true);
                    rightMotor.stop();
                }
            } else {
                if (intensity < 30) {
                    // Turn left
                    leftMotor.forward();
                    rightMotor.backward();
                } else if (intensity > 40) {
                    // Turn right
                    leftMotor.backward();
                    rightMotor.forward();
                } else {
                    // Move forward
                    leftMotor.forward();
                    rightMotor.forward();
                }
            }
            

            // Add a small delay to control loop execution frequency
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
    }

            // Add a small delay to control loop execution frequency
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
     	}
}
