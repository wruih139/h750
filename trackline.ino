void GetLineHitCount() {
    int count = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (rec_data[i] == 1) count++; // Change detection for white line
    }
}

void ComputeLineError() {
    int error = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (rec_data[i] == 1) {
            error += (i - (SENSOR_COUNT / 2)); // Change detection for white line
        }
    }
}

bool IsIntersectionDetected() {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (rec_data[i] == 1) { // Change detection for white line
            return true;
        }
    }
    return false;
}

void SetSensorStateLost() {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (rec_data[i] != 1) { // Change condition for white line
            // Handle lost sensor state
        }
    }
}