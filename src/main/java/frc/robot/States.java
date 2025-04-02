
package frc.robot;

import frc.robot.Constants.Windmill.WindmillState;

public class States {

    public States() {
    }

    public class DriveState {

        public static double elevatorMultiplier = 1.0;

        public DriveState() {

        }

        public DriveState getInstance() {
            return this;
        }

        public double getElevatorMultiplier() {
            return elevatorMultiplier;
        }

        public void setElevatorMultiplier(double Multiplier) {
            elevatorMultiplier = Multiplier;
        }
    }

    public class WindmillStates {
        public static WindmillState CurrentWindmillState = WindmillState.Home;
        public static boolean mirrored = false;

        public WindmillStates() {

        }

        public WindmillStates getInstance() {
            return this;
        }

        public void setWindmillState(WindmillState state) {
            CurrentWindmillState = state;
        }

        public void setMirrored(boolean mirror) {
            mirrored = mirror;
        }

        public WindmillState getWindmillState() {
            return CurrentWindmillState;
        }

        public boolean getMirrored() {
            return mirrored;
        }
    }
}
