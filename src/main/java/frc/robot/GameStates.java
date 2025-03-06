
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GameStates {
    public enum GameState {

        // blue is the left side of the robot, yellow is right (intake) side

        HOME,
        TRANSITION,
        CORAL_SOURCE,
        CB1,
        CB2,
        CB3,
        CB4,
        CY1,
        CY2,
        CY3,
        CY4,
        ALGAE_GROUND_B,
        AB2,
        AB3,
        ALGAE_NET_B,
        ALGAE_GROUND_Y,
        AY2,
        AY3,
        ALGAE_NET_Y,
        CLIMB_MODE;

    }

    public static void displayGameState(GameState state) {
        switch (state) {
            case HOME:
                SmartDashboard.putString("Game State", "HOME");
                break;
            case TRANSITION:
                SmartDashboard.putString("Game State", "TRANSITION");
                break;
            case CORAL_SOURCE:
                SmartDashboard.putString("Game State", "CORAL_SOURCE");
                break;
            case CB1:
                SmartDashboard.putString("Game State", "CB1");
                break;
            case CB2:
                SmartDashboard.putString("Game State", "CB2");
                break;
            case CB3:
                SmartDashboard.putString("Game State", "CB3");
                break;
            case CB4:
                SmartDashboard.putString("Game State", "CB4");
                break;
            case CY1:
                SmartDashboard.putString("Game State", "CY1");
                break;
            case CY2:
                SmartDashboard.putString("Game State", "CY2");
                break;
            case CY3:
                SmartDashboard.putString("Game State", "CY3");
                break;
            case CY4:
                SmartDashboard.putString("Game State", "CY4");
                break;
            case ALGAE_GROUND_B:
                SmartDashboard.putString("Game State", "ALGAE_GROUND_B");
                break;
            case AB2:
                SmartDashboard.putString("Game State", "AB2");
                break;
            case AB3:
                SmartDashboard.putString("Game State", "AB3");
                break;
            case ALGAE_NET_B:
                SmartDashboard.putString("Game State", "ALGAE_NET_B");
                break;
            case ALGAE_GROUND_Y:
                SmartDashboard.putString("Game State", "ALGAE_GROUND_Y");
                break;
            case AY2:
                SmartDashboard.putString("Game State", "AY2");
                break;
            case AY3:
                SmartDashboard.putString("Game State", "AY3");
                break;
            case ALGAE_NET_Y:
                SmartDashboard.putString("Game State", "ALGAE_NET_Y");
                break;
            case CLIMB_MODE:
                SmartDashboard.putString("Game State", "CLIMB_MODE");
                break;
        }
    }
}
