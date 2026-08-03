#include "Filter_IIR.h"

template <typename dataType>
void Filter_IIR<dataType>::set_alpha(float alpha) {
    if (alpha > 1.0) {
        this->alpha = 1.0;
    }
    else if (alpha < 0.0) {
        this->alpha = 0.0;
    }
    else
    {
        this->alpha = alpha;
    }
}

template <typename dataType>
dataType Filter_IIR<dataType>::filtered_result(dataType new_val) {
    prev_reading = (1 - alpha) * new_val + alpha * prev_reading;

    return prev_reading;
}

