import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollowing {
    // Define the color ID of the line
    private static final int LINE_COLOR_ID = 7;

    public static void main(String[] args) {
        Brick brick = BrickFinder.getDefault();
        EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
        EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3); 
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1); 

        // Set motor speeds
        int baseSpeed = 320;
        leftMotor.setSpeed(baseSpeed);
        rightMotor.setSpeed(baseSpeed);

        // Create color detection thread
        ColorDetectionThread colorDetectionThread = new ColorDetectionThread(colorSensor);
        colorDetectionThread.start();

        // Create obstacle detection thread
        ObstacleDetectionThread obstacleDetectionThread = new ObstacleDetectionThread(ultrasonicSensor);
        obstacleDetectionThread.start();

        // Create line following thread
        LineFollowingThread lineFollowingThread = new LineFollowingThread(leftMotor, rightMotor, colorDetectionThread, obstacleDetectionThread);
        lineFollowingThread.start();

        // Wait for button press to start the program.
        System.out.println("Press any button to start...");
        Button.waitForAnyPress();

        while (!Button.ESCAPE.isDown()) {
            // Add a small delay to control loop execution frequency
            Delay.msDelay(10);
        }

        // Stop the motors and close the sensors when the program ends
        leftMotor.stop(true);
        rightMotor.stop();
        colorSensor.close();
        ultrasonicSensor.close();
    }

    static class ColorDetectionThread extends Thread {
        private EV3ColorSensor colorSensor;
        private int lineColorId;

        public ColorDetectionThread(EV3ColorSensor colorSensor) {
            this.colorSensor = colorSensor;
        }

        @Override
        public void run() {
            SampleProvider colorProvider = colorSensor.getColorIDMode();
            float[] sample = new float[colorProvider.sampleSize()];

            while (!Button.ESCAPE.isDown()) {
                colorProvider.fetchSample(sample, 0);
                lineColorId = (int) sample[0];
            }
            colorSensor.close();
        }

        public int getLineColorId() {
            return lineColorId;
        }
    }

    static class ObstacleDetectionThread extends Thread {
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

            while (!Button.ESCAPE.isDown()) {
                // Read distance from the ultrasonic sensor
                distanceProvider.fetchSample(sample, 0);
                int distance = (int) (sample[0] * 100); // Convert to centimeters

                if (distance < 10) {
                    obstacleDetected = true;
                } else {
                    obstacleDetected = false;
                }

                // Add a small delay to control loop execution frequency
                Delay.msDelay(50);
            }
        }

        public boolean isObstacleDetected() {
            return obstacleDetected;
        }
    }

    static class LineFollowingThread extends Thread {
        private EV3LargeRegulatedMotor leftMotor;
        private EV3LargeRegulatedMotor rightMotor;
        private ColorDetectionThread colorDetectionThread;
        private ObstacleDetectionThread obstacleDetectionThread;
        private static final int baseSpeed = 200;
        private static final int searchSpeed = baseSpeed / 2;

        public LineFollowingThread(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, ColorDetectionThread colorDetectionThread, ObstacleDetectionThread obstacleDetectionThread) {
            this.leftMotor = leftMotor;
            this.rightMotor = rightMotor;
            this.colorDetectionThread = colorDetectionThread;
            this.obstacleDetectionThread = obstacleDetectionThread;
            // Set motor speeds
            leftMotor.setSpeed(baseSpeed);
            rightMotor.setSpeed(baseSpeed);
        }

        @Override
        public void run() {
            boolean lineFound = false;
            while (!Button.ESCAPE.isDown()) {
                int colorId = colorDetectionThread.getLineColorId();
                boolean obstacleDetected = obstacleDetectionThread.isObstacleDetected();

                if (obstacleDetected && !lineFound) {
                	System.out.println("Obstacle else...");
                	System.out.println("right turn...");
                    
                    leftMotor.forward();
                    rightMotor.stop();
                    leftMotor.stop();
                    System.out.println("right turn");
                    
                    leftMotor.forward();
                    rightMotor.forward();
                    Delay.msDelay(100);
                    rightMotor.stop();
                    leftMotor.stop();
                    System.out.println("forward");
                    
                    
                    rightMotor.forward();
                    leftMotor.stop();
                    rightMotor.stop();
                    System.out.println("left turn");
                    
                    leftMotor.forward();
                    rightMotor.forward();
                    Delay.msDelay(100);
                    
                    
                    System.out.println("stop...");
                } else {
                	 System.out.println("Obstacle found...");
                }

                if (colorId == LINE_COLOR_ID) {
                    // Black line detected, continue moving forward
                    leftMotor.forward();
                    rightMotor.forward();
                    lineFound = true;
                    System.out.println("Line detected");
                } else {
                    if (lineFound) {
                        // Curve handling
                        System.out.println("Line missing, curve handling...");
                        leftMotor.setSpeed((searchSpeed / 2));
                        rightMotor.setSpeed(baseSpeed);
                        leftMotor.forward();
                        rightMotor.forward();
                        Delay.msDelay(100); 
                        leftMotor.setSpeed(baseSpeed);
                        rightMotor.setSpeed(searchSpeed / 2);
                        leftMotor.forward();
                        rightMotor.forward();
                        Delay.msDelay(200); 
                        System.out.println("Curve handling complete");
                        // Continue searching for line
                        leftMotor.backward();
                        rightMotor.forward();
                        lineFound = false;
                    } else  {
                    	 // Continue searching for line
                        leftMotor.backward();
                        rightMotor.forward();
                    }
                }

                // Add a small delay to control loop execution frequency
                Delay.msDelay(10);
            }
        }
    }
}
