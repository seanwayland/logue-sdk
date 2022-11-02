
#include "usermodfx.h"
#include "osc_api.h"


float main_right_amount = 0.1f;

float sub_right_amount = 0.1f;

/* Implementation of the initialization callback */
void MODFX_INIT(uint32_t platform, uint32_t api)
{
main_right_amount = 0.1f;
sub_right_amount = 0.1f;
}


/**
 * Implementation of the process callback
 * Ref: https://korginc.github.io/logue-sdk/ref/minilogue-xd/v1.1-0/html/group__modfx__inst.html#gaea822f104394d2f1d9139568d18d7994
 *
 * @param main_xn Input sample buffer for main timbre
 * @param main_yn Output sample buffer for main timbre
 * @param sub_xn  Input sample buffer for sub timbre
 * @param sub_yn  Output sample buffer for sub timbre
 * @param frames  Size of buffers. (2 samples per frames)
 *
 * Note: sub_xn and sub_yn are not used here, as the Minilogue XD has no sub timbre
 */
void MODFX_PROCESS(const float *main_xn, float *main_yn,
                   const float *sub_xn,  float *sub_yn,
                   uint32_t frames)
{

  // main input
  const float * mx = main_xn;

  const float * sx = sub_xn;

  // main output
  float * __restrict my = main_yn;
  float * __restrict sy = sub_yn;

  // one sample per channel (L/R)
  const float * my_e = my + 2*frames;
  const float * sy_e = sy + 2*frames;

  float processed_mxl;
  float processed_mxr;

  float processed_sxl;
  float processed_sxr;

  float right_gain = main_right_amount;
  float sub_right_gain = sub_right_amount;

  for (; my != my_e; ) {

    processed_mxl = *(mx++); // use L channel sample as input (mono)

    processed_mxr = *(mx++); // use L channel sample as input (mono)

    processed_sxl = *(sx++); // use L channel sample as input (mono)

    processed_sxr = *(sx++); // use L channel sample as input (mono)

    // write to both channels (L/R)
    *(my++) = 2*processed_mxl*(1-main_right_amount);
    *(my++) = (2*processed_mxr*main_right_amount);

    *(sy++) = 2*processed_sxl*(1-sub_right_amount);
    *(sy++) = 2*processed_sxr*sub_right_amount;

  
  }

}

/**
 * Implementation of the param callback that gets called when the user changes the time or depth knob
 */
void MODFX_PARAM(uint8_t index, int32_t value)
{
  const float valf = q31_to_f32(value); // valf is in range (0.0 - 1.0)
  switch (index) {
  case k_user_modfx_param_time:
    main_right_amount = valf;
    break;
  case k_user_modfx_param_depth:
   sub_right_amount = valf;

    break;
  default:
    break;
  }
}