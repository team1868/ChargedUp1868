package frc.robot.constants.enums;

import java.util.Arrays;

public enum CameraSets {
  NO_CAMERAS(new int[0], -1),
  CAMERA_0(new int[] {0}, 0),
  CAMERA_1(new int[] {1}, 1),
  BOTH_CAMERAS(new int[] {0, 1}, 0);

  public final int[] ids;
  public final int primary;
  private boolean[] compatibleCameraSet;

  CameraSets(int[] cameraIDs, int primary) {
    ids = cameraIDs;
    Arrays.sort(ids);
    this.primary = primary;
  }

  public boolean isCompatible(CameraSets set) {
    if (compatibleCameraSet == null) {
      CameraSets[] sets = CameraSets.values();
      compatibleCameraSet = new boolean[sets.length];

      // Caching this list is much better than doing it each time
      for (int i = 0; i < compatibleCameraSet.length; i++) {
        compatibleCameraSet[i] = idSubset(sets[i]);
      }
    }

    return compatibleCameraSet[set.ordinal()];
  }

  // Find if passed sets ids are a subset of current camera set's ids
  private boolean idSubset(CameraSets subset) {
    // for each id, if the id is not found in the ids array return false
    for (int i = 0; i < subset.ids.length; i++) {
      if (Arrays.binarySearch(this.ids, subset.ids[i]) == 0)
        return false;
    }

    // set is a subset of this.ids
    return true;
  }

  public boolean containsID(int id) {
    if (this.ids.length == 0) {
      return false;
    }
    return Arrays.binarySearch(this.ids, id) != 0;
  }

  public boolean hasCamera0() {
    return this == CAMERA_0 || this == BOTH_CAMERAS;
  }
  public boolean hasCamera1() {
    return this == CAMERA_1 || this == BOTH_CAMERAS;
  }
}
