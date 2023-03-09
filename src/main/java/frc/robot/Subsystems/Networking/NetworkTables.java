package frc.robot.Subsystems.Networking;

import java.util.HashMap;
import java.util.TreeSet;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class NetworkTables {

    private ShuffleboardTab shuffleboardTab;
    private HashMap<String, GenericEntry> genericEntries = new HashMap<>();
    
    public NetworkTables(String tabName, TreeSet<NetworkEntry> inherited) {
        shuffleboardTab = Shuffleboard.getTab(tabName);
        GetSendable getSendable = new GetSendable(tabName, inherited);

        for (int i=0;i<inherited.size();i++) {
            String childName = getSendable.GetSendableName(i);
            Object childVal = getSendable.GetSendableValue(i);

            SendableRegistry.add(getSendable, childName);

            try {
                BuiltInWidgets childWidget = getSendable.GetSendableWidget(i);
    
                GenericEntry entry = shuffleboardTab.add(childName, 
                childVal).withWidget(childWidget).getEntry();
                
                genericEntries.put(childName, entry); //widget still broken :(              
            } catch (NullPointerException e) {
                // TODO: handle exception
                System.out.println("Widget cannot be constructed!");
            }
        }

    }

    public Object getEntry(String key) {
        return genericEntries.get(key).get().getValue();
    }

    public class GetSendable implements Sendable {

        private String tabName;
        private TreeSet<NetworkEntry> setup;

        private NetworkEntry sendables[];

        public GetSendable(String tabName, TreeSet<NetworkEntry> setup) {
            this.tabName = tabName;
            this.setup = setup;

            this.sendables = new NetworkEntry[this.setup.size()];
            this.sendables = this.setup.toArray(this.sendables);
        }

        public String GetSendableName(int index) {
            return sendables[index].GetName();
        }

        public BuiltInWidgets GetSendableWidget(int index) {
            return sendables[index].GetWidget();
        }

        public Object GetSendableValue(int index) {
            Object value = sendables[index].GetValue();

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
