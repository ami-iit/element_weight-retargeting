
/**
 * Definition of the WeightRetargeting RPC service
 */
service WeightRetargetingService {

    /**
     * Set the max threshold value for an actuators group
     * @return true if the procedure was successful, false otherwise
     */
    bool setMaxThreshold(1: string actuatorGroup, 2: double value);

    /**
     * Set the min threshold value for an actuators group
     * @return true if the procedure was successful, false otherwise
     */
    bool setMinThreshold(1: string actuatorGroup, 2: double value);

    /**
     * Set the threshold values for an actuators group
     * @return true if the procedure was successful, false otherwise
     */
    bool setThresholds(1: string actuatorGroup, 2: double minThreshold, 3: double maxThreshold);

    /**
     * Remove the offset of an actuators group.
     * The name all can be used to remove the offset of all of the actuators group.
     * @return true if the procedure was successful, false otherwise
     */
    bool removeOffset(1: string actuatorGroup);
}
