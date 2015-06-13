package org.apache.commons.math4.filter;

import org.apache.commons.math4.linear.RealMatrix;


public interface MeasurementModelKF extends MeasurementModel {
    /**
     * Returns the measurement matrix.
     *
     * @return the measurement matrix
     */
    RealMatrix getMeasurementMatrix();

}
