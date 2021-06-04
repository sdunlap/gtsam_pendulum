import gtsam
import numpy as np
from typing import List



def error_func(this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
    # Get the variable values from `v`
    key0 = this.keys()[0]
    key1 = this.keys()[1]
    
    # Calculate non-linear error
    gT1, gT2 = v.atPose2(key0), v.atPose2(key1)
    error = gtsam.Pose2(0, 0, 0).localCoordinates(gT1.between(gT2))

    # If we need Jacobian
    if H is not None:
        # Fill the Jacobian arrays
        # Note we have two vars, so two entries
        result = gT1.between(gT2)
        H[0] = -result.inverse().AdjointMap()
        H[1] = np.eye(3)
    
    # Return the error
    return error