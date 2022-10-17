
/*
 * File: bpmdelay.cpp
 * hammondeggsmusic.ca 2021
 */

#include "userdelfx.h" 

// Defines
#define NUM_DELAY_DIVISIONS      15       // # of bpm divisions in table
#define DELAY_LINE_SIZE          0x40000  // Delay line size (*must be a power of 2)
#define DELAY_LINE_SIZE_MASK     0x3FFFF  // Mask for the delay line size for rollover
#define DELAY_GLIDE_RATE         12000    //  this value must not be lower than 1. larger values = slower glide rates for delay time
#define MIN_BPM                  56       // failsafe, likely never used
#define NUM_NOTES_PER_BEAT       4        // The xd/prologue use quarter notes, hence '4'.
#define SAMPLE_RATE              48000    // 48KHz is our fixed sample rate (the const k_samplerate is only listed in the osc_api.h not the fx_api.h)

#define PSEUDO_STEREO_OFFSET (float)SAMPLE_RATE * .01f    // How much time to offset the right channel in seconds for pseudo stereo(.01 = 10ms) 


// Delay lines for left / right channel
__sdram float delayLine_L[DELAY_LINE_SIZE];
__sdram float delayLine_R[DELAY_LINE_SIZE];

// Current position in the delay line we are writing to:
// (integer value as it is per-sample)
uint32_t delayLine_Wr1 = 0;
uint32_t delayLine_Wr2 = 0;
uint32_t delayLine_Wr3 = 0;
uint32_t delayLine_Wr4 = 0;

// Smoothing (glide) for delay time:
// This is the current delay time as we smooth it
float currentDelayTime1 = 48000; 
float currentDelayTime2= 48000; 
float currentDelayTime3= 48000; 
float currentDelayTime4= 48000; 

// This is the delay time we actually wish to set to
float targetDelayTime1 = 48000;
float targetDelayTime2 = 48000;
float targetDelayTime3 = 48000;
float targetDelayTime4 = 48000;

// Depth knob value from 0-1
float valDepth = 0;

// Time value knob from 0-1
float valTime = 0;

// Delay time multiplier (will be pulled from delayDivisions table)
float multiplier = 1;

// Wet/Dry signal levels
float wet = .2;
float dry = .6;
float wet_mix = .5;

 
////////////////////////////////////////////////////////////////////////
// DELFX_INIT
// - initialize the effect variables, including clearing the delay lines
////////////////////////////////////////////////////////////////////////
void DELFX_INIT(uint32_t platform, uint32_t api)
{
   // Initialize the variables used
   delayLine_Wr1 = delayLine_Wr2 = delayLine_Wr3 = delayLine_Wr3=0;

   // Clear the delay lines. If you don't do this, it is entirely possible that "something" will already be there, and you might
   // get either old delay sounds, or very unpleasant noises from a previous effects. 
   for (int i=0;i<DELAY_LINE_SIZE;i++)
   {
      delayLine_L[i] = 0;
      delayLine_R[i] = 0;
   }

   currentDelayTime1 = currentDelayTime2 = currentDelayTime3 = currentDelayTime4 = SAMPLE_RATE; 
   targetDelayTime1 = targetDelayTime2 = targetDelayTime3 =  targetDelayTime3 =  SAMPLE_RATE;

   valDepth = 0;
   valTime = 0;
   multiplier = 1;


   wet = 0.2f;
   dry = 0.5f;
   wet_mix = 0.5f;
   
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
// readFrac
// 
// fractionally read from a buffer
// That is, this allows you to read 'between' two points in a table
// using a floating point index.
//  - buffer size must be a power of 2
//
// this is from the korg example (slightly modified)
////////////////////////////////////////////////////////////////////////////////////////////////////////
// compiler conditions to a: compile this code 'inline' and b: set a specific optimization for this routine.
// compiling inline saves you a few cycles but can result in larger code.
inline __attribute__((optimize("Ofast"),always_inline)) 
float readFrac(const float pos, float *pDelayLine) 
{
   // Get the 'base' value - that is, the integer value of the position
   // e.g. if we're looking for value at position 1423.6, this will yield an integer of 1426
   uint32_t base = (uint32_t)pos;

   // Get the fraction (decimal) portion of the index
   const float frac = pos - base;	

   // Get the sample at the base index - note by masking the base index with the delay line mask we don't have
   // to do any modulus / manual checks for overflow. This requries the buffer size to be a power of 2.
   const float s0 = pDelayLine[base & DELAY_LINE_SIZE_MASK];

   // Get the next sample at the base index + 1. Again, by masking with the delay line size mask, we don't have 
   // to worry about rolling over the buffer index.
   base++;
   const float s1 = pDelayLine[base & DELAY_LINE_SIZE_MASK];

   // Using the logue-sdk linear interpolation function, get the linearly-interpolated result of the two sample values.
   float r = linintf(frac, s0, s1);
   return r;    
}


 

////////////////////////////////////////////////////////////////////////
// DELFX_PROCESS
// - Called for every buffer , process your samples here
////////////////////////////////////////////////////////////////////////
void DELFX_PROCESS(float *xn, uint32_t frames)
{

   float * __restrict x = xn; // Local pointer, pointer xn copied here. 
   const float * x_e = x + 2*frames; // End of data buffer address


   // Calculate our delay time (as a float) by taking:
   //   The # of samples per second * the # of beats per second * the number of notes per second * our multiplier.
   //   note, the multiplier is 1 or lower, so this will result in a reduction only.
   //targetDelayTime = SAMPLE_RATE * bpm_s * NUM_NOTES_PER_BEAT * multiplier;
   targetDelayTime1 = 11000;
   targetDelayTime2 = 16000;
   targetDelayTime3 = 19200;
   targetDelayTime4 = 25000;
   
          
   // Loop through the samples - for delay effects, you replace the value at *xn with your new value
   // This data is interleaved with left/right data
   for (; x != x_e; ) 
   {
      // Smoothly transition the delay time
      // - This gives the same effect as exponential 'glide'

      // Calculate the difference between the target and the current delay time
      float delta1 = targetDelayTime1 - currentDelayTime1;
      float delta2 = targetDelayTime2 - currentDelayTime2;
      float delta3 = targetDelayTime3 - currentDelayTime3;
      float delta4 = targetDelayTime3 - currentDelayTime3;

      // Divide this by the glide rate (larger glide rates = longer glide times.)
      // Glide rate cannot be lower than 1!
      delta1 /= DELAY_GLIDE_RATE;
      delta2 /= DELAY_GLIDE_RATE;
      delta3 /= DELAY_GLIDE_RATE;
      delta4 /= DELAY_GLIDE_RATE;

      // Add to our current delay time this delta1. 
      currentDelayTime1 += delta1;   
      currentDelayTime2 += delta2;   
      currentDelayTime3 += delta3; 
      currentDelayTime4 += delta4;   

      //Get our input signal values to the effect

      float sigInL = *x; // get the value pointed at x (Left channel)
      float sigInR = *(x+1); // get the value pointed at x + 1(right channel)
      
      // Declare some storage for our output signals
      float sigOutL;
      float sigOutR;

      // The way this delay will work, is we will continually write to the delay line
      // with the new incoming audio directly into the delay line (per sample). 
      // We will read 'behind' this index using a floating point value to allow us
      // to read sub-sample values from this delay line.

      // Calculate the read index (floating point so can have a fraction)
      float readIndex1 = (float)delayLine_Wr1 - currentDelayTime1;
      float readIndex2 = (float)delayLine_Wr2 - currentDelayTime2;
      float readIndex3 = (float)delayLine_Wr3 - currentDelayTime3;
      float readIndex4 = (float)delayLine_Wr4 - currentDelayTime4;

      // Since this is a float we can't just mask it to account for rollover
      // - since we subtracted the index it could be negative - roll this value over
      // around the delay line
      if (readIndex1 < 0)
      {
         // Roll the pointer back to the end of the buffer (index)
         readIndex1 += DELAY_LINE_SIZE_MASK;
      }

      if (readIndex2 < 0)
      {
         // Roll the pointer back to the end of the buffer (index)
         readIndex2 += DELAY_LINE_SIZE_MASK;
      }

      if (readIndex3 < 0)
      {
         // Roll the pointer back to the end of the buffer (index)
         readIndex3 += DELAY_LINE_SIZE_MASK;
      }


      if (readIndex4 < 0)
      {
         // Roll the pointer back to the end of the buffer (index)
         readIndex4 += DELAY_LINE_SIZE_MASK;
      }

      //Read the delayed (behind) signal for the left channel
      float delayLineSig_L1 = readFrac(readIndex1, delayLine_L);

      // Read the delayed (behind) signal for the right channel
      float delayLineSig_R1 = readFrac(readIndex1, delayLine_R);

            //Read the delayed (behind) signal for the left channel
      float delayLineSig_L2 = readFrac(readIndex2, delayLine_L);

      // Read the delayed (behind) signal for the right channel
      float delayLineSig_R2 = readFrac(readIndex2, delayLine_R);

                  //Read the delayed (behind) signal for the left channel
      float delayLineSig_L3 = readFrac(readIndex3, delayLine_L);

      // Read the delayed (behind) signal for the right channel
      float delayLineSig_R3 = readFrac(readIndex3, delayLine_R);

      //Read the delayed (behind) signal for the left channel
      float delayLineSig_L4 = readFrac(readIndex4, delayLine_L);

      // Read the delayed (behind) signal for the right channel
      float delayLineSig_R4 = readFrac(readIndex4, delayLine_R);

      // Write to the delay line our input signal + the delayed signal * the feedback amount (valDepth)      
      // Left delay:
      delayLine_L[delayLine_Wr1] = sigInL + delayLineSig_L1 * valDepth; 
      // Right delay:
      delayLine_R[delayLine_Wr1] = sigInR + delayLineSig_R1 * valDepth; 

      // Write to the delay line our input signal + the delayed signal * the feedback amount (valDepth)      
      // Left delay:
      delayLine_L[delayLine_Wr2] = sigInL + delayLineSig_L2 * valDepth; 
      // Right delay:
      delayLine_R[delayLine_Wr2] = sigInR + delayLineSig_R2 * valDepth; 

            // Write to the delay line our input signal + the delayed signal * the feedback amount (valDepth)      
      // Left delay:
      delayLine_L[delayLine_Wr3] = sigInL + delayLineSig_L3 * valDepth; 
      // Right delay:
      delayLine_R[delayLine_Wr3] = sigInR + delayLineSig_R3 * valDepth; 

      // Write to the delay line our input signal + the delayed signal * the feedback amount (valDepth)      
      // Left delay:
      delayLine_L[delayLine_Wr4] = sigInL + delayLineSig_L4 * valDepth; 
      // Right delay:
      delayLine_R[delayLine_Wr4] = sigInR + delayLineSig_R4 * valDepth; 

      // Increment and roll over our write index for the delay line 
      // This is an integer, and a power of 2 so we can simply mask the value by the DELAY_LINE_SIZE_MASK.
      delayLine_Wr1++;
      delayLine_Wr1 &= DELAY_LINE_SIZE_MASK; 

      delayLine_Wr2++;
      delayLine_Wr2 &= DELAY_LINE_SIZE_MASK; 

      delayLine_Wr3++;
      delayLine_Wr3 &= DELAY_LINE_SIZE_MASK; 


      delayLine_Wr4++;
      delayLine_Wr4 &= DELAY_LINE_SIZE_MASK; 


    
      // Generate our output signal:
      // That is, the input signal * the dry level + (mixed with) the delayed signal * the wet level.
      sigOutL = sigInL * dry + ((delayLineSig_L1 + delayLineSig_R1 + delayLineSig_L2 ) * wet/3);

      // And again for the right channel
      sigOutR = sigInR * dry + (delayLineSig_R3 + delayLineSig_L4 ) * wet/3;

      // Store this result into the output buffer
      *x = sigOutL;

      // Move to the next channel
      x++;

      // Store this result into the output buffer
      *x = sigOutR; 

      // Move to the next interleaved sample
      x++;		
   }
   
}



////////////////////////////////////////////////////////////////////////////////////
//		PARAM
//
// On changing any of the knobs this is called. If you have any math to perform
// on the parameters, do it here vs in the audio callback to save a ton of time
// BPM is NOT sent here - you have to pull it manually via:
// fx_get_bpmf() for float or
// fx_get_bpm() for integer * 10 
//
// We'll use fx_get_bpmf(), which returns a floating point value.
//
// If there are values to be calculated based on these knob values, it is ideal to 
// put those calculations in here and not in the DSP loop as you will be wasting 
// CPU time calculating those values every time. 
//
////////////////////////////////////////////////////////////////////////////////////
void DELFX_PARAM(uint8_t index, int32_t value)
{
   const float valf = q31_to_f32(value);

   switch (index) 
   {

         case k_user_delfx_param_depth:      
         ////////////////////////////
         // "B" / DEPTH KNOB
         ////////////////////////////
         // Set the delay feedback (0-1, tbd if i use an exp table)   
         // Just store this value for the DSP loop to use.
         valDepth = valf;
         break;

         case k_user_delfx_param_time:
         ////////////////////////////
         // "A" / TIME KNOB
         ////////////////////////////
         // Calculate the coarse delay time (via the divisions table)

         //Store this 0-1 value in case we need it for something else (currently we do not)
         wet_mix = valf;
         dry = 1.0f - wet_mix; 
         wet = wet_mix;    


         break;

         case k_user_delfx_param_shift_depth: 

         break;
         default:
         // no default handling, there is no case for this.         
         break;
   }
   
}












