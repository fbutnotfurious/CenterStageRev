package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

public class Constants {
    public static final Rect LEFT_ROI_RED = new Rect(
            new Point(40, 45),//25,50
            new Point(115, 115)// 130,120
    );
    public static final Rect CENTER_ROI_RED = new Rect(
            new Point(150, 30),//25,60
            new Point(310, 80)//85 130, 270
    );
    public static final Rect RIGHT_ROI_RED = new Rect(
            new Point(170, 70),
            new Point(235, 145)
    );
    public static final Rect LEFT_ROI_BLUE = new Rect(
            new Point(25, 45),//15,45,10,45//40,74
            new Point(110, 120)//110,125//145
    );
    public static final Rect CENTER_ROI_BLUE = new Rect(
            new Point(150, 35),//235,145
            new Point(300, 75)//270,125
    );
    public static final Rect RIGHT_ROI_BLUE = new Rect(
            new Point(25, 70),
            new Point(100, 150)
    );
    public static final Scalar RED_LOW_HSV = new Scalar(0, 155,  115);
    public static final Scalar RED_HIGH_HSV = new Scalar(25, 255, 255);
    public static final Scalar BLUE_LOW_HSV = new Scalar(105, 85, 40);
    public static final Scalar BLUE_HIGH_HSV = new Scalar(115, 225, 225);
}
