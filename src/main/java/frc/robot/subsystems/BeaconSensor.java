package frc.robot.subsystems;

import field_elements.Beacon;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeaconSensor extends SubsystemBase {

    private HashMap<Integer,Beacon> _beacons = new HashMap<Integer,Beacon>();
   
    public BeaconSensor() {
        // initialize sensor
    }

    // returns info about a specific beacon
    public Beacon getBeacon(Integer id) {
        return _beacons.get(id);
    }

    // Returns the nearest unset beacon. If all beacons have been set, returns an unspecified set beacon.
    // Returns null if there are no known beacons. Returns 
    public Beacon getNearestBeacon() {
        Beacon nearest = null;
        Beacon compare = null;
        while( _beacons.keySet().iterator().hasNext() ) {
            compare = _beacons.get(_beacons.keySet().iterator().next());
            if( nearest == null ) {
                nearest = compare;
            } else if( (compare.getDistance() < nearest.getDistance()) && !compare.isColorSet() ) {
                nearest = compare;
            }
        }
        return nearest;
    }

    // return all visible beacons, keyed on arbitrary ID
    public HashMap<Integer,Beacon> getAllBeacons() {
        return _beacons;
    }

    // scan once for any visible beacons
    private void scan() {
        // storing duplicate id will replace value, which is good because we want
        // the Beacon objects to stay updated with the most current info
        return;
    }

    @Override
    public void periodic() {
        scan();
    }
}