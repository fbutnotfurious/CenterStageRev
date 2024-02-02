package org.firstinspires.ftc.teamcode.Autonomous;

public class PathConstants {

    // Common
    static final double WristDownOffset=0.6;//0.4

    // Common Blue Back Stage
    // Center Spike Mark
    static final double BlueBackStage_CenterSpike_ForwardPoint1=16.5;
    // wrist down
    static final double BlueBackStage_CenterSpike_ForwardPoint2=12.0;
    // Open Gripper to drop 1st pixel
    static final double BlueBackStage_CenterSpike_BackwardPoint3=-3;
    // wrist down with offset WristDownOffset
    //close gripper
    // wrist up
    static final double BlueBackStage_CenterSpike_BackwardPoint4=-1;
    // turn left to be ready to drop off
    static final double BlueBackStage_CenterSpike_TurnRight5=-12.2;
    // final drop off point
    static final double BlueBackStage_CenterSpike_BackwardPoint6=-37.0;

    // Arm Score
    // Open Gripper
    // Arm Intake
    // BlueBackStage_CenterSpike Center Park
    static final double BlueBackStage_CenterSpike_ParkForwardPoint1= 12.0;
    static final double BlueBackStage_CenterSpike_ParkTurnLeft2= -7.0;
    static final double BlueBackStage_CenterSpike_ParkBackwardPoint3= -26.5;
    static final double BlueBackStage_CenterSpike_ParkBackwardPoint3_Center= -24.5;//-26.5

    //================================================================

    // Common Blue Back Stage
    // Left Spike Mark

    static final double BlueBackStage_LeftSpike_ForwardPoint1=16.5;
    static final double BlueBackStage_LeftSpike_ForwardPoint2=13.0;
    static final double BlueBackStage_LeftSpike_TurnRight3=-11.2;
    static final double BlueBackStage_LeftSpike_BackwardPoint4=-22;

    // Open Gripper
    //     static final double BSL_BackwardPoint3=-3;
    static final double BlueBackStage_LeftSpike_BackwardPoint5=-3;

    // Wrist Down with WristDownOffset offset
    // Close Gripper
    // Wrist up
    static final double BlueBackStage_LeftSpike_BackwardPoint6=-1;
    // Final Drop off Point
    static final double BlueBackStage_LeftSpike_BackwardPoint7=-11;//-11.5
    // Arm Score
    // Open Pixel
    // Arm Intake

    //  Left Park
    static final double BlueBackStage_LeftSpike_ParkForwardPoint1= 11.0;
    static final double BlueBackStage_LeftSpike_ParkTurnLeft2= -7.5;//-6.0
    static final double BlueBackStage_LeftSpike_ParkTurnLeft2_Center= -8.5;//-6.0

    static final double BlueBackStage_LeftSpike_ParkBackwardPoint3= -21.5;//-22.5
    static final double BlueBackStage_LeftSpike_ParkBackwardPoint3_Center= -26.5;

    //================================================================
    //  Blue Back Stage
    // Right Spike Mark
    static final double BlueBackStage_RightSpike_ForwardPoint1=16.5;
    //FowardPoint1
    //FowardPoint2
    static final double BlueBackStage_RightSpike_ForwardPoint2=13.0;
    // Turn right
    static final double BlueBackStage_RightSpike_TurnRight3=13.3;
    static final double BlueBackStage_RightSpike_ForwardPoint4_0=2.0;

    static final double BlueBackStage_RightSpike_ForwardPoint4=3.0;
    // Open Gripper
    //reverse by BlueBackStage_RightSpike_ForwardPoint4
    // Wrist Down with WristDownOffset offset
    // Close Gripper
    // Wrist up
    static final double BlueBackStage_RightSpike_BackwardPoint5=-1.0;
    static final double BlueBackStage_RightSpike_BackwardPoint6=-35.5;//-35.5
    // Arm Score
    // Gripper Open
    //Arm Intake
    // BSL Right Park
    static final double BlueBackStage_RightSpike_ParkForwardPoint1= 11.0;
    static final double BlueBackStage_RightSpike_ParkTurnLeft2= -8.0;
    static final double BlueBackStage_RightSpike_ParkBackwardPoint3= -27.5;//-22.5

    static final double RBlueBackStage_RightSpike_ParkTurnLeft2_Center= -7.0;//-6
    static final double BlueBackStage_RightSpike_ParkBackwardPoint3_Center= -24.5;//-22.5

//===================================================================================
    //  Red Back Stage
    // Center Spike Mark
    static final double RedBackStage_CenterSpike_ForwardPoint1=16.5;
    static final double RedBackStage_CenterSpike_ForwardPoint2=12;

    static final double RedBackStage_CenterSpike_BackwardPoint3=-3;
    // wrist down with offset WristDownOffset
    //close gripper
    // wrist up
    static final double RedBackStage_CenterSpike_BackwardPoint4=-1;
    // turn left to be ready to drop off
    static final double RedBackStage_CenterSpike_TurnRight5=11.4;
    // final drop off point
    static final double RedBackStage_CenterSpike_BackwardPoint6=-39.0;

    // Arm Score
    // Open Gripper
    // Arm Intake
    // BlueBackStage_CenterSpike Center Park
    static final double RedBackStage_CenterSpike_ParkForwardPoint1= 12.0;
    static final double RedBackStage_CenterSpike_ParkTurnLeft2= -7.0;
    static final double RedBackStage_CenterSpike_ParkBackwardPoint3= -26.5;//-21.5
    static final double RedBackStage_CenterSpike_ParkBackwardPoint3_Corner= -24.5;//-26.5

    //  Red Back Stage
    // Left Spike Mark

    static final double RedBackStage_LeftSpike_ForwardPoint1=16.5;
    static final double RedBackStage_LeftSpike_ForwardPoint2=13.0;
    static final double RedBackStage_LeftSpike_TurnRight3=-12.0;
    static final double RedBackStage_LeftSpike_ForwardPoint4=1;
    static final double RedBackStage_LeftSpike_ForwardPoint5=-3;
    static final double RedBackStage_LeftSpike_BackwardPoint6=-1;

    // Open Gripper
    // Wrist Down with WristDownOffset offset
    // Close Gripper
    // Wrist up
    // Final Drop off Point
    static final double RedBackStage_LeftSpike_BackwardPoint7=-36;
    // Arm Score
    // Open Pixel
    // Arm Intake

    //  Left Park
    static final double RedBackStage_LeftSpike_ParkForwardPoint1= 12.0;
    static final double RedBackStage_LeftSpike_ParkTurnLeft2= -8.0;
    static final double RedBackStage_LeftSpike_ParkBackwardPoint3= -24.5;
    static final double RedBackStage_LeftSpike_ParkBackwardPoint3_Corner= -28.5;

    //================================================================
    //  Red Back Stage
    // Right Spike Mark

    static final double RedBackStage_RightSpike_ForwardPoint1=16.5;
    //FowardPoint1
    //FowardPoint2
    static final double RedBackStage_RightSpike_ForwardPoint2=13.5;
    // Turn right
    static final double RedBackStage_RightSpike_TurnRight3=10.7;
    static final double RedBackStage_RightSpike_BackwardPoint4=-23.2;
    static final double RedBackStage_RightSpike_BackwardPoint5=-4.0;
    // Open Gripper
    //reverse by RedBackStage_RightSpike_ForwardPoint4
    // Wrist Down with WristDownOffset offset
    // Close Gripper
    // Wrist up
    static final double RedBackStage_RightSpike_BackwardPoint6=-1.0;
    static final double RedBackStage_RightSpike_BackwardPoint7=-12.5;

    // Park
    static final double RedBackStage_RightSpike_ParkForwardPoint1= 11.0;
    static final double RedBackStage_RightSpike_ParkTurnLeft2= -9.0;//-6
    static final double RedBackStage_RightSpike_ParkBackwardPoint3= -25.5;//-20.5

    static final double RedBackStage_RightSpike_ParkTurnLeft2_Corner= -7.0;//-6
    static final double RedBackStage_RightSpike_ParkBackwardPoint3_Corner= -22.5;//-20.5

    //================================================================
    // front stage Red
    //Center
    static final double RedFrontStage_CenterSpike_ForwardPoint1=16.5;
    static final double RedFrontStage_CenterSpike_ForwardPoint2=12.0;
    static final double RedFrontStage_CenterSpike_BackwardPoint3=-3.0;
    static final double RedFrontStage_CenterSpike_BackwardPoint4=-22.0;//-26
    static final double RedFrontStage_CenterSpike_TurnRight1=-12.2;
    static final double RedFrontStage_CenterSpike_ForwardPoint3=55;
    static final double RedFrontStage_CenterSpike_TurnLeft1=4;
    static final double RedFrontStage_CenterSpike_ForwardPoint4=36;
    // Left
    static final double RedFrontStage_LeftSpike_ForwardPoint1=4;
    static final double RedFrontStage_LeftSpike_TurnLeft1=3;
    static final double RedFrontStage_LeftSpike_ForwardPoint2=14.0;//11.0
    static final double RedFrontStage_LeftSpike_TurnRight1=-3.5;
    static final double RedFrontStage_LeftSpike_ForwardPoint3=5;
    static final double RedFrontStage_LeftSpike_BackwardPoint3 = -3.0;
    static final double RedFrontStage_LeftSpike_BackwardPoint4=-9.5;//11.0
    static final double RedFrontStage_LeftSpike_BackwardPoint5=-4.0;
    static final double RedFrontStage_LeftSpike_TurnRight6=-12.2;
    static final double RedFrontStage_LeftSpike_ForwardPoint7=55;
    static final double RedFrontStage_LeftSpike_TurnLeft8=4;
    static final double RedFrontStage_LeftSpike_ForwardPoint9=36;

    // Right
    static final double RedFrontStage_RightSpike_ForwardPoint1=13.5;// 13 11
    static final double RedFrontStage_RightSpike_TurnRight1=-4.8;//4.7 -5
    static final double RedFrontStage_RightSpike_ForwardPoint2=6.0;//5
    static final double RedFrontStage_RightSpike_TurnLeft1=6;
    static final double RedFrontStage_RightSpike_BackwardPoint3=-3;
    static final double RedFrontStage_RightSpike_ForwardPoint3=7;
    static final double RedFrontStage_RightSpike_BackwardPoint4=-11.0;//-12
    static final double RedFrontStage_RightSpike_TurnLeft1_2 =3;
    static final double RedFrontStage_RightSpike_BackwardPoint5=-5.5;
    static final double RedFrontStage_RightSpike_TurnRight2=-11.2;//12.2
    static final double RedFrontStage_RightSpike_ForwardPoint3_2=55;
    static final double RedFrontStage_RightSpike_TurnLeft1_3=4;
    static final double RedFrontStage_RightSpike_ForwardPoint4=36;
    //================================================================
    // front stage Blue
    //Center
    static final double BlueFrontStage_CenterSpike_ForwardPoint1=16.5;
    static final double BlueFrontStage_CenterSpike_ForwardPoint2=12.0;
    static final double BlueFrontStage_CenterSpike_BackwardPoint3=-3.0;
    static final double BlueFrontStage_CenterSpike_BackwardPoint4=-26.0;
    static final double BlueFrontStage_CenterSpike_TurnLeft1=11;
    static final double BlueFrontStage_CenterSpike_ForwardPoint3=55;
    static final double BlueFrontStage_CenterSpike_TurnRight1=-6;
    static final double BlueFrontStage_CenterSpike_ForwardPoint4=36;
    // Left
    static final double BlueFrontStage_LeftSpike_ForwardPoint1=8;
    static final double BlueFrontStage_LeftSpike_TurnLeft1=3;
    static final double BlueFrontStage_LeftSpike_ForwardPoint2=11.0;
    static final double BlueFrontStage_LeftSpike_TurnRight1=-2.5;
    static final double BlueFrontStage_LeftSpike_ForwardPoint3=5;
    static final double BlueFrontStage_LeftSpike_BackwardPoint4=-11.0;
    static final double BlueFrontStage_LeftSpike_BackwardPoint5=-4.0;
    // Right
    static final double BlueFrontStage_RightSpike_ForwardPoint1=11;
    static final double BlueFrontStage_RightSpike_TurnRight1=-5;
    static final double BlueFrontStage_RightSpike_ForwardPoint2=5.0;
    static final double BlueFrontStage_RightSpike_TurnLeft1=8;
    static final double BlueFrontStage_RightSpike_ForwardPoint3=7;
    static final double BlueFrontStage_RightSpike_BackwardPoint4=-12.0;
    static final double BlueFrontStage_RightSpike_BackwardPoint5=-4.0;

}






