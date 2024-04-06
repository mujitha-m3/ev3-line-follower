import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class LineFollowingThread extends Thread {
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private ColorDetectionThread colorDetectionThread;
    private ObstacleDetectionThread obstacleDetectionThread;
    private boolean lineFound;
  
    private static final int BASE_SPEED = 200;
    private static final int SEARCH_SPEED = BASE_SPEED / 2;
  
    public LineFollowingThread(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, ColorDetectionThread colorDetectionThread) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.colorDetectionThread = colorDetectionThread;
        // Set motor speeds
        leftMotor.setSpeed(BASE_SPEED);
        rightMotor.setSpeed(BASE_SPEED);
        this.lineFound = false;
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            boolean lineDetected = colorDetectionThread.isLineDetected();

            if (lineDetected) {
                // Move forward if line is detected
                leftMotor.forward();
                rightMotor.forward();
                lineFound = true;
            } else {
                // No line detected
                if (lineFound) {
                    // Curve handling
                    System.out.println("Curve handling...");
                    // Reduce speed of one motor to turn
                    leftMotor.setSpeed(SEARCH_SPEED);
                    rightMotor.setSpeed(BASE_SPEED);

                    // Turn right
                    leftMotor.forward();
                    rightMotor.forward();
                    // delay for turnin
                    // Turn right
                    leftMotor.forward();
                    rightMotor.forward();

                    Delay.msDelay(500);
                    // Restore speeds for straight line following
                    leftMotor.setSpeed(BASE_SPEED);
                    rightMotor.setSpeed(BASE_SPEED);
                    System.out.println("Curve handling complete");
                    lineFound = false; // Reset line found flag
                } else {
                    // Continue searching for line
                    leftMotor.forward();
                    rightMotor.forward();
                }
            }


            if (obstacleDetected) {
                // Obstacle detected, stop the motors
                System.out.println("Obstacle detected...");
                leftMotor.stop(true);
                rightMotor.stop();
            }

            // Add a small delay to control loop execution frequency
            Delay.msDelay(10);
        }
    }
}
