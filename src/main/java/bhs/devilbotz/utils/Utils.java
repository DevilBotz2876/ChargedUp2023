package bhs.devilbotz.utils;

import edu.wpi.first.wpilibj.Timer;

public class Utils {
    public static boolean isTrueForX(boolean input, double x) {
        Timer timer = new Timer();
        if (input) {
            System.out.println("Timer Started");
            timer.start();
        } else {
            System.out.println("Timer reset");
            timer.stop();
            timer.reset();
        }
        return timer.get() >= x;
    }
}