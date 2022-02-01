
/**
 * Definition of the WeightRetargeting RPC service
 */
service WeightRetargetingService {

    /**
     * Set the max threshold value for a joint axes
     * @return true if the procedure was successful, false otherwise
     */
    bool setMaxThreshold(1: string actuatorGroup, 2: double value);

    /**
     * Set the max threshold value for a joint axes
     * @return true if the procedure was successful, false otherwise
     */
    bool setMinThreshold(1: string actuatorGroup, 2: double value);

    /**
     * Set the max threshold value for a joint axes
     * @return true if the procedure was successful, false otherwise
     */
    bool setThresholds(1: string actuatorGroup, 2: double minThreshold, 3: double maxThreshold);

    /**
     * Remove the offset of an actuators group
     * @return true if the procedure was successful, false otherwise
     */
    bool removeOffset(1: string actuatorGroup);
}
