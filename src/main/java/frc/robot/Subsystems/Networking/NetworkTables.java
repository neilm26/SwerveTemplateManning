package frc.robot.Subsystems.Networking;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NetworkTables {

    private ShuffleboardTab shuffleboardTab;
    private HashMap<String, GenericEntry> genericEntries = new HashMap<>();
    
    public NetworkTables(String tabName, HashMap<String, ?> inherited) {
        shuffleboardTab = Shuffleboard.getTab(tabName);

        for (int i=0;i<inherited.size();i++) {
            GetSendable getSendable = new GetSendable(tabName, inherited);

            SendableRegistry.add(getSendable, getSendable.GetSendableName(i));
            
            genericEntries.putIfAbsent(getSendable.GetSendableName(i), shuffleboardTab.add(getSendable.GetSendableName(i), 
            getSendable.GetSendableValue(i)).getEntry()); //widget still broken :(
        }
    }

    public Object getEntry(String key) {
        return genericEntries.get(key).get().getValue();
    }

    public class GetSendable implements Sendable {

        private String tabName;
        private HashMap<String, ?> setup;

        public GetSendable(String tabName, HashMap<String, ?> setup) {
            this.tabName = tabName;
            this.setup = setup;
        }

        public String GetSendableName(int index) {
            return setup.keySet().toArray()[index].toString();
        }

        public Object GetSendableValue(int index) {
            Object value = setup.values().toArray()[index];

            if (value instanceof Translation2d) {
                Translation2d vector2d = (Translation2d) value;
                Double[] vectorArray2d = new Double[] {vector2d.getX(), vector2d.getY()};

                return vectorArray2d;
            }
            return value;
        }



        @Override
        public void initSendable(SendableBuilder builder) {
            // TODO Auto-generated method stub
            if (tabName == null) {
                return;
            }
            builder.setSmartDashboardType(tabName);
        }        
    }
}
