package bhs.devilbotz.lib;

/** The community location where the robot is starting from */
public enum CommunityLocation {
  WALL, // In the community, closest to the wall
  CHARGE_STATION, // In the community, in front of the charge station (middle)
  HUMAN // In the community, closed to the human/loading zone
}
