import lejos.utility.Delay;

public class LineFollowingTimeTracker extends Thread {
    private long startTime;
    private long elapsedTime; // Define elapsedTime as an instance variable

    public LineFollowingTimeTracker() {
        this.startTime = System.currentTimeMillis();
        this.elapsedTime = 0; // Initialize elapsedTime
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            long currentTime = System.currentTimeMillis();
            elapsedTime = currentTime - startTime;
            System.out.println("Elapsed Time: " + elapsedTime / 1000 + " seconds");

            // Add a small delay to control loop execution frequency
            Delay.msDelay(1000); // Update every second
        }
    }

    
    public long getElapsedTime() {
        return elapsedTime;
    }
}
