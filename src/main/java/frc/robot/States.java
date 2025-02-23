
package frc.robot;

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
}
