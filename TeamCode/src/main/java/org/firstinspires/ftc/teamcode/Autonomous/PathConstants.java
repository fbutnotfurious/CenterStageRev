package org.firstinspires.ftc.teamcode.Autonomous;

public class PathConstants {

    // Common
    static final double WristDownOffset=0.6;//0.4

    // Common back stage left (blue) BSL Center
    static final double BSL_FowardPoint1=17.0;
    // wrist down
    static final double BSL_FowardPoint2=12.0;
    // Open Gripper to drop 1st pixel
    static final double BSL_BackwardPoint3=-3;
    // wrist down with offset WristDownOffset
    //close gripper
    // wrist up
    static final double BSL_BackwardPoint4=-1;
    // turn left to be ready to drop off
    static final double BSL_TurnLeft5=-12.2;
    // final drop off point
    static final double BSL_BackwardPoint6=-32.0;
    static final double BSL_BackwardPoint6_Offset=4;
    // Arm Score
    // Open Gripper
    // Arm Intake
    // BSL Center Park
    static final double BSL_ParkPoint1= 12.0;
    static final double BSL_ParkTurnLeft2= -7.0;
    static final double BSL_ParkPoint3= -21.5;
    //================================================================

    // back stage left (blue) BSL Left
    //     static final double BSL_FowardPoint1=17.0;
    // wrist Down
    // ForwardPoint 2 + below left offset
    static final double BSL_ForwardPoint2_Left_Offset =1.0;

    // turn left with below left offset
    static final double BSL_TurnLeft5_Left_Offset =1.2;// 0.6
    static final double BSL_Left_BackwardPoint6=-22;
    // Open Gripper
    //     static final double BSL_BackwardPoint3=-3;
    // Wrist Down with WristDownOffset offset
    // Close Gripper
    // Wrist up
    //     static final double BSL_BackwardPoint4=-1;
    // Final Drop off Point
    static final double BSL_Left_BackwardPoint7=-10.5;//-11.5
    // Arm Score
    // Open Pixel
    // Arm Intake

    // BSL Left Park
    static final double BSL_Left_ParkPoint1= 11.0;
    static final double BSL_Left_ParkTurnLeft2= -6.0;//-9.0
    static final double BSL_Left_ParkPoint3= -20.5;
    //================================================================

    // back stage left (blue) BSL Right
    //BSL_FowardPoint1
    //BSL_FowardPoint2 with below right offset
    static final double BSL_ForwardPoint2_Right_Offset =1.0;

    // Turn left with below turn offset
    static final double BSL_TurnLeft5_Right_Offset =0.8;

    static final double BSL_Right_FowardPoint3=3.0;
    // Open Gripper
    //BSL_BackwardPoint3
    // Wrist Down with WristDownOffset offset
    // Close Gripper
    // Wrist up
    //BSL_BackwardPoint4
    static final double BSL_Right_BackwardPoint6=-35.5;
    // Arm Score
    // Gripper Open
    //Arm Intake
    // BSL Right Park
    //BSL_Left_ParkPoint1
    //BSL_Left_ParkTurnLeft2
    //BSL_Left_ParkPoint3
    //================================================================

    // back stage Right (Red) BSR Center
    // turn left to be ready to drop off
    static final double BSR_TurnRight5=11.3;//12.5
    static final double BSR_TurnRight5_offset=0.0;
    static final double BSR_BackwardPoint6=-32.0;
    static final double BSR_BackwardPoint6_Center_offset=7;

    // BSR Center Park
    static final double BSL_ParkTurnRight2= -6.0;

    //================================================================
    // back stage Red BSR Left
    //-PathConstants.BSR_TurnRight5-PathConstants.BSR_TurnRight5_left_Offset
    static final double BSR_TurnRight5_left_Offset=0.0;//0.2 0.5
    static final double BSR_Left_BackwardPoint6=-22.2;
    static final double BSR_Left_FowardPoint3=1;

    // BSR Left Park
    static final double BSR_Left_ParkTurnRight2= -8;//-9

    //================================================================
    // back stage Red BSR Right
    static final double BSR_ForwardPoint2_Right_Offset=1.5;
    static final double BSR_TurnRight5_right_Offset=1.33;// 2
    static final double BSR_Left_BackwardPoint6_right_offset=-1;
    static final double  BSL_Left_BackwardPoint7_right_offset=-2.0;
    //================================================================

    //================================================================
    // front stage Red FSR Right
    //Center
    static final double FSR_TurnRight5=11.6;
    static final double FSR_BackwardPoint6=-84.5;
    static final double FSR_BackwardPoint6_Center_offset=7;
    // Left
    static final double FSR_TurnRight5_left_Offset=0.5;
    static final double FSR_Left_BackwardPoint4=-58.4;//-53
    static final double FSR_Left_TurnLeft=-1.8;
    static final double FSR_Left_BackwardPoint6_left_offset=4;


    //Right
    static final double FSR_Right_BackwardPoint4=-36;//-53



    }






