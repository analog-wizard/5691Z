int16_t srlimit_update(int16_t x_raw_input, 
                       int16_t x_prev_limit_out, 
                       int16_t maxchange)
{
    // 1. Compute change in input
    int32_t delta = (int32_t)x_raw_input - x_prev_limit_out;
    // 2. Limit this change to within acceptable limits
    if (delta > maxchange)
        delta = maxchange;
    if (delta < -maxchange)
        delta = -maxchange;
    // 3. Apply the limited change to
    //    compute the new adjusted output.
    //    Use this as the next value
    //    of x_prev_limit_out
    return delta/10;
}