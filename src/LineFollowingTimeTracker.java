
import lejos.utility.Delay;

public class LineFollowingTimeTracker extends Thread {
    private long startTime;

    public LineFollowingTimeTracker() {
        this.startTime = System.currentTimeMillis();
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            long currentTime = System.currentTimeMillis();
            long elapsedTime = currentTime - startTime;
            System.out.println("Elapsed Time: " + elapsedTime / 1000 + " seconds");

            // Add a small delay to control loop execution frequency
            try {
                Thread.sleep(1000); // Update every second
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
