package Teamcode;

import static Teamcode.GlobalConstants.intakeArmMid;

import android.graphics.Color;

public class GlobalFunctions {
    Hardware robot = new Hardware();
    double Arm = 0;
    double Slide = 0;

    public void slideLinkage(double POSITION){
        robot.slideL.setPosition(POSITION);
        robot.slideR.setPosition(.90-POSITION);
    }

    public void intake(double POWER){
        robot.rightRoller.setPower(-POWER);
        robot.leftRoller.setPower(POWER);
    }

    public void intakeArm(double POSITION){
        robot.intakeArmL.setPosition(POSITION);
        robot.intakeArmR.setPosition(0.98-POSITION);
    }

    public String colorDetect() {
        float[] hsvValues = new float[3];  // Store HSV values from the sensor
        Color.RGBToHSV((int) (robot.intakeColor.red() * 255),
                (int) (robot.intakeColor.green() * 255),
                (int) (robot.intakeColor.blue() * 255),
                hsvValues);
        float hue = hsvValues[0]; // Get the hue value from HSV
        intakeArm(Arm);
        slideLinkage(Slide);
        // Compare the HSV values to thresholds and set Arm and Slide positions
        if (hue < 30 || hue > 330) {  // Hue close to red
            Arm = intakeArmMid;
            intake(1);
            return "Red";
        } else if (hue > 40 && hue < 80) {  // Hue close to yellow
            Arm = GlobalConstants.intakeArmUp;
            Slide = GlobalConstants.slideBack;
            return "Yellow";
        } else if (hue > 200 && hue < 260) {  // Hue close to blue
            Arm = GlobalConstants.intakeArmUp;
            Slide = GlobalConstants.slideBack;
            return "Blue";
        } else {
            return "Unknown";
        }
    }

}
