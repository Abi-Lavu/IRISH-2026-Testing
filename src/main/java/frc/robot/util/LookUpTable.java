package frc.robot.util;

public class LookUpTable {
    private static class LookUpTableEntry{
        final double distance;
        final double rpm;

        public LookUpTableEntry(double distance, double rpm){
            this.distance = distance;
            this.rpm = rpm;
        }

        }
        private static final LookUpTableEntry[] LOOKUP_TABLE = {
            // new LookUpTableEntry(2.5, 3000),

    };
}







