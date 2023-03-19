package frc.robot.Subsystems.Networking;

import java.util.HashMap;
import java.util.Objects;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;

public class NetworkTables {

    public NetworkTables() {

    }

    private static HashMap<String, GenericEntry> genericEntries = new HashMap<>();

    public static HashMap<String, GenericEntry> getEntries() {
        return genericEntries;
    }

    public static void addEntries(NetworkEntry... entries) {
        for (NetworkEntry entry: entries) {
            genericEntries.put(entry.getEntryName(), entry.getEntry());
        }
    }
}
