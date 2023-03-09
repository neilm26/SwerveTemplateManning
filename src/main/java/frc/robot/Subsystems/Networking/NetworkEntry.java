package frc.robot.Subsystems.Networking;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

public class NetworkEntry implements Comparable<NetworkEntry>{
    
    private String entryName;
    private Object val;
    private BuiltInWidgets widget;
    private int thisID;
    public static int ID = 0;

    public NetworkEntry(String entryName, BuiltInWidgets widget, Object val) {
        this.entryName = entryName;
        this.val = val;
        this.widget = widget;
        thisID = ID += 1;
    }

    public Object GetValue() {
        return val;
    }

    public BuiltInWidgets GetWidget() {
        return widget;
    }

    public String GetName() {
        return entryName;
    }

    @Override
    public int compareTo(NetworkEntry arg0) {
        // TODO Auto-generated method stub
        return thisID;
    }
}
