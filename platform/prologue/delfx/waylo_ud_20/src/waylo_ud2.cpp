
/*
 * File: bpmdelay.cpp
 * hammondeggsmusic.ca 2021
 */
#include <stdlib.h>
#include "userdelfx.h" 
#include "simplelfo.hpp"

// Defines


#define DELAY_LINE_SIZE          0x20000 // Delay line size (*must be a power of 2)
#define MATH_PI                  3.14159
#define DELAY_LINE_SIZE_MASK     0x1FFFF // doobee do 
#define DELAY_GLIDE_RATE         12000    //  this value must not be lower than 1. larger values = slower glide rates for delay time
#define SAMPLE_RATE              48000    // 48KHz is our fixed sample rate (the const k_samplerate is only listed in the osc_api.h not the fx_api.h)

#define PSEUDO_STEREO_OFFSET (float)SAMPLE_RATE * .01f    // How much time to offset the right channel in seconds for pseudo stereo(.01 = 10ms)

// LFO STUFF 
static dsp::SimpleLFO s_lfo1;
static dsp::SimpleLFO s_lfo2;
static dsp::SimpleLFO s_lfo3;
static dsp::SimpleLFO s_lfo4;
static dsp::SimpleLFO s_lfo5;
static dsp::SimpleLFO s_lfo6;
static dsp::SimpleLFO s_lfo7;
static dsp::SimpleLFO s_lfo8;


static uint8_t s_lfo_wave1;
static uint8_t s_lfo_wave2;
static uint8_t s_lfo_wave3;
static uint8_t s_lfo_wave4;
static uint8_t s_lfo_wave5;
static uint8_t s_lfo_wave6;
static uint8_t s_lfo_wave7;
static uint8_t s_lfo_wave8;

static float s_param_z, s_param;
static const float s_fs_recip = 1.f / 48000.f;

float wave1;
float wave2;
float wave3;
float wave4;
float wave5;
float wave6;
float wave7;
float wave8;

int leadSound = 0;


// Delay lines for left / right channel
__sdram float delayLine_L[DELAY_LINE_SIZE];
__sdram float delayLine_R[DELAY_LINE_SIZE];
__sdram float delayLine_L2[DELAY_LINE_SIZE];
__sdram float delayLine_R2[DELAY_LINE_SIZE];



// Current position in the delay line we are writing to:
// (integer value as it is per-sample)
uint32_t delayLine_Wr1 = 0;
uint32_t delayLine_Wr2 = 0;
uint32_t delayLine_Wr3 = 0;
uint32_t delayLine_Wr4 = 0;

uint32_t delayLine_Wr5 = 0;
uint32_t delayLine_Wr6 = 0;
uint32_t delayLine_Wr7 = 0;
uint32_t delayLine_Wr8 = 0;

// Smoothing (glide) for delay time:
// This is the current delay time as we smooth it
float currentDelayTime1 = 48000; 
float currentDelayTime2= 48000; 
float currentDelayTime3= 48000; 
float currentDelayTime4= 48000; 

float currentDelayTime5 = 48000; 
float currentDelayTime6= 48000; 
float currentDelayTime7= 48000; 
float currentDelayTime8= 48000; 

// This is the delay time we actually wish to set to
float targetDelayTime1 = 48000;
float targetDelayTime2 = 48000;
float targetDelayTime3 = 48000;
float targetDelayTime4 = 48000;
float targetDelayTime5 = 48000;
float targetDelayTime6 = 48000;
float targetDelayTime7 = 48000;
float targetDelayTime8 = 48000;

// Depth knob value from 0-1
float valDepth = 0;

// Time value knob from 0-1
float valTime = 0;

// shift depth knob value from 0-1
float valShift = 0;


float feedBack1 = 0;
float feedBack2 = 0;
float feedBack3 = 0;
float feedBack4 = 0;
float feedBack5 = 4.5f;
float feedBack6= 3.5;
float feedBack7 = 4.3;
float feedBack8 = 3.4;


float delaySpeed1 = 2.4f;
float delaySpeed2 = 2.1f;
float delaySpeed3 = 2.8f;
float delaySpeed4 = 1.9f;
float delaySpeed5 = 3.5f;
float delaySpeed6 = 3.8f;
float delaySpeed7 = 4.7f;
float delaySpeed8 = 3.3f;

float modDepth1 = 4.2;
float modDepth2 = 4.3;
float modDepth3 = 3.8;
float modDepth4 = 4.1;
float modDepth5 = 2.5;
float modDepth6 = 2.5;
float modDepth7 = 2.5;
float modDepth8 = 2.5;



float feedbackScale = 0.0;

float reverbLeft = 1.f;
float reverbRight = 1.f;
float chorusLeft = 1.f;
float chorusRight = 1.f;

// Wet/Dry signal levels
float wet = .8;
float dry = .7;

float wet_mix = .1;
// scale from 0 to 0.5 ie val / 2 
float reverb_mix = 0.5;

float speed_param = 17.f;
float mod_depth = 16.f;

float timeval = .5f;

float depth_scale = 1.0;

static float state = 0;
static const float cutoff_frequency = 400.0;
static const float gain = cutoff_frequency / (2 * MATH_PI * SAMPLE_RATE);

////////////////////////////////////////////////////////////////////////
// DELFX_INIT
// - initialize the effect variables, including clearing the delay lines
////////////////////////////////////////////////////////////////////////
void DELFX_INIT(uint32_t platform, uint32_t api)
{


   // lfo stuff 
   // change from 10 to 50 with time dial ie 5 + val * 5 
   

   s_lfo1.reset();
   s_lfo1.setF0(delaySpeed1*speed_param,s_fs_recip);

   s_lfo2.reset();
   s_lfo2.setF0(delaySpeed2*speed_param,s_fs_recip);

   s_lfo3.reset();
   s_lfo3.setF0(delaySpeed3*speed_param,s_fs_recip);

   s_lfo4.reset();
   s_lfo4.setF0(delaySpeed4*speed_param,s_fs_recip);

   s_lfo5.reset();
   s_lfo5.setF0(delaySpeed5*speed_param,s_fs_recip);

   s_lfo6.reset();
   s_lfo6.setF0(delaySpeed6*speed_param,s_fs_recip);

   s_lfo7.reset();
   s_lfo7.setF0(delaySpeed7*speed_param,s_fs_recip);

   s_lfo8.reset();
   s_lfo8.setF0(delaySpeed8*speed_param,s_fs_recip);


   // Initialize the variables used
   delayLine_Wr1 = delayLine_Wr2 = delayLine_Wr3 = delayLine_Wr3=0;

   delayLine_Wr5 = delayLine_Wr6 = delayLine_Wr7 = delayLine_Wr8=0;

   // Clear the delay lines. If you don't do this, it is entirely possible that "something" will already be there, and you might
   // get either old delay sounds, or very unpleasant noises from a previous effects. 
   for (int i=0;i<DELAY_LINE_SIZE;i++)
   {
      delayLine_L[i] = 0;
      delayLine_R[i] = 0;
   }

     for (int i=0;i<DELAY_LINE_SIZE;i++)
   {
      delayLine_L2[i] = 0;
      delayLine_R2[i] = 0;
   }



   currentDelayTime1 = currentDelayTime2 = currentDelayTime3 = currentDelayTime4 = SAMPLE_RATE; 
   targetDelayTime1 = targetDelayTime2 = targetDelayTime3 =  targetDelayTime3 =  SAMPLE_RATE;

   currentDelayTime5 = currentDelayTime6 = currentDelayTime7 = currentDelayTime8 = SAMPLE_RATE; 
   targetDelayTime5 = targetDelayTime6 = targetDelayTime7 =  targetDelayTime8 =  SAMPLE_RATE;

   valDepth = 0;
   valTime = 0;
   valShift = .5;


   feedBack1 = 0;
   feedBack2 = 0;
   feedBack3 = 0;
   feedBack4 = 0;
   feedBack5 = 4.5;
   feedBack6 = 3.5;
   feedBack7 = 4.3;
   feedBack8 = 3.4;

   delaySpeed1 = 4.7;
   delaySpeed2 = 5.8;
   delaySpeed3 = 6.8;
   delaySpeed4 = 7.2;
   delaySpeed5 = 3.5;
   delaySpeed6 = 3.8;
   delaySpeed7 = 4.7;
   delaySpeed8 = 3.3;

   modDepth1 = 2.3;
   modDepth2 = 2.3;
   modDepth3 = 1;
   modDepth4 = 1;
   modDepth5 = 2.5;
   modDepth6 = 2.5;
   modDepth7 = 2.5;
   modDepth8 = 2.5;

   depth_scale = 1;
   feedbackScale = 0.0;

   leadSound = 1;



   wet = 0.3f;
   dry = 0.7f; 
   wet_mix = 0.4f;
   reverb_mix = 0.5f;


   
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




inline __attribute__((optimize("Ofast"),always_inline)) 
float highpass(float input) {
    float retval = input - state;
    state += gain * retval;
    return retval;
}


////////////////////////////////////////////////////////////////////////
// DELFX_PROCESS
// - Called for every buffer , process your samples here
////////////////////////////////////////////////////////////////////////
void DELFX_PROCESS(float *xn, uint32_t frames)
{

   // lfo stuff 

   s_lfo1.cycle();
   s_lfo2.cycle();
   s_lfo3.cycle();
   s_lfo4.cycle();
   s_lfo5.cycle();
   s_lfo6.cycle();
   s_lfo7.cycle();
    s_lfo8.cycle();



      wave1 = s_lfo1.sine_bi();
      wave2 = s_lfo2.sine_bi();
      wave3 = s_lfo3.sine_bi();
      wave4 = s_lfo4.sine_bi();
      wave5 = s_lfo5.sine_bi();
      wave6 = s_lfo6.sine_bi();
      wave7 = s_lfo7.sine_bi();
      wave8 = s_lfo8.sine_bi();

   // change from 15 to 30 with a number ie 15 + val * 2 
   
    wave1 *= (modDepth1/10)*0.1*(1 + depth_scale);
    wave2 *= (modDepth2/10)*0.1*(1 + depth_scale);
    wave3 *= (modDepth3/10)*0.1*(1 + depth_scale);
    wave4 *= (modDepth4/10)*0.1*(1 + depth_scale);
    wave5 *= (modDepth5/10)*0.05*(1 + depth_scale);
    wave6 *= (modDepth6/10)*0.05*(1 + depth_scale);
    wave7 *= (modDepth7/10)*0.05*(1 + depth_scale);
    wave8 *= (modDepth8/10)*0.05*(1 + depth_scale);
     //wave8 = wave8/mod_depth;
   



   // end lfo stuff

   float * __restrict x = xn; // Local pointer, pointer xn copied here. 
   const float * x_e = x + 2*frames; // End of data buffer address


   // Calculate our delay time (as a float) by taking:
   //   The # of samples per second * the # of beats per second * the number of notes per second * our multiplier.
   //   note, the multiplier is 1 or lower, so this will result in a reduction only.
   //targetDelayTime = SAMPLE_RATE * bpm_s * NUM_NOTES_PER_BEAT * multiplier;
   float delay_time_adjust = 1.0;



//0.0151,0.029,0.0411,0.037,0.3,0.4,0.355,0.463,

// 1 ON IN NOR 1->1 OFF SIN 22 OFF OFF 0 100 4.7 2.3 L10 10
// 2 ON IN NOR 2->2 OFF SIN 27.1 OFF OFF 0 100 5.8 2.3 R10 10
// 3 ON IN NOR 3->3 OFF SIN 30 OFF OFF 0 100 6.8 1 L10 10
// 4 ON IN NOR 4->4 OFF SIN 40 OFF OFF 0 100 7.2 1 R10 10


   // targetDelayTime1 = 1104*(1+ wave1)*delay_time_adjust;
   // targetDelayTime2 = 1300*(1+ wave2)*delay_time_adjust;
   // targetDelayTime3 = 1440*(1+ wave3)*delay_time_adjust;
   // targetDelayTime4 = 1920*(1+ wave4)*delay_time_adjust;
   // targetDelayTime5 = 10500;
   // targetDelayTime6 = 13500;
   // targetDelayTime7 = 19200;
   // targetDelayTime8 = 23500;


   // targetDelayTime1 = 1404*(1+ wave1)*delay_time_adjust;
   // targetDelayTime2 = 1600*(1+ wave2)*delay_time_adjust;
   // targetDelayTime3 = 1740*(1+ wave3)*delay_time_adjust;
   // targetDelayTime4 = 2220*(1+ wave4)*delay_time_adjust;
   // targetDelayTime5 = 10500;
   // targetDelayTime6 = 13500;
   // targetDelayTime7 = 19200;
   // targetDelayTime8 = 23500;

   targetDelayTime1 = 1036.8*(1+ wave1)*delay_time_adjust;
   targetDelayTime2 = 1392*(1+ wave2)*delay_time_adjust;
   targetDelayTime3 = 2400*(1+ wave3)*delay_time_adjust;
   targetDelayTime4 = 1920*(1+ wave4)*delay_time_adjust;
   targetDelayTime5 = 10500;
   targetDelayTime6 = 13500;
   targetDelayTime7 = 19200;
   targetDelayTime8 = 23500;






   
   
          
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

      float delta5 = targetDelayTime5 - currentDelayTime5;
      float delta6 = targetDelayTime6 - currentDelayTime6;
      float delta7 = targetDelayTime7 - currentDelayTime7;
      float delta8 = targetDelayTime8 - currentDelayTime8;

      // Divide this by the glide rate (larger glide rates = longer glide times.)
      // Glide rate cannot be lower than 1!
      delta1 /= DELAY_GLIDE_RATE;
      delta2 /= DELAY_GLIDE_RATE;
      delta3 /= DELAY_GLIDE_RATE;
      delta4 /= DELAY_GLIDE_RATE;

      delta5 /= DELAY_GLIDE_RATE;
      delta6 /= DELAY_GLIDE_RATE;
      delta7 /= DELAY_GLIDE_RATE;
      delta8 /= DELAY_GLIDE_RATE;

      // Add to our current delay time this delta1. 
      currentDelayTime1 += delta1;   
      currentDelayTime2 += delta2;   
      currentDelayTime3 += delta3; 
      currentDelayTime4 += delta4;   

      currentDelayTime5+= delta5;   
      currentDelayTime6 += delta6;   
      currentDelayTime7 += delta7; 
      currentDelayTime8 += delta8; 

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

      float readIndex5 = (float)delayLine_Wr5 - currentDelayTime5;
      float readIndex6 = (float)delayLine_Wr6 - currentDelayTime6;
      float readIndex7 = (float)delayLine_Wr7 - currentDelayTime7;
      float readIndex8 = (float)delayLine_Wr8 - currentDelayTime8;

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


            if (readIndex5 < 0)
      {
         // Roll the pointer back to the end of the buffer (index)
         readIndex5 += DELAY_LINE_SIZE_MASK;
      }

      if (readIndex6 < 0)
      {
         // Roll the pointer back to the end of the buffer (index)
         readIndex6 += DELAY_LINE_SIZE_MASK;
      }

      if (readIndex7 < 0)
      {
         // Roll the pointer back to the end of the buffer (index)
         readIndex7 += DELAY_LINE_SIZE_MASK;
      }


      if (readIndex8 < 0)
      {
         // Roll the pointer back to the end of the buffer (index)
         readIndex8 += DELAY_LINE_SIZE_MASK;
      }

      //Read the delayed (behind) signal for the left channel
      float delayLineSig_L1 = readFrac(readIndex1, delayLine_L2);

      // Read the delayed (behind) signal for the right channel
      float delayLineSig_R1 = readFrac(readIndex1, delayLine_R2);

      //Read the delayed (behind) signal for the left channel
      float delayLineSig_L2 = readFrac(readIndex2, delayLine_L2);

      // Read the delayed (behind) signal for the right channel
      float delayLineSig_R2 = readFrac(readIndex2, delayLine_R2);

                  //Read the delayed (behind) signal for the left channel
      float delayLineSig_L3 = readFrac(readIndex3, delayLine_L2);

      // Read the delayed (behind) signal for the right channel
      float delayLineSig_R3 = readFrac(readIndex3, delayLine_R2);

      //Read the delayed (behind) signal for the left channel
      float delayLineSig_L4 = readFrac(readIndex4, delayLine_L2);

      // Read the delayed (behind) signal for the right channel
      float delayLineSig_R4 = readFrac(readIndex4, delayLine_R2);

            //Read the delayed (behind) signal for the left channel
      float delayLineSig_L5 = readFrac(readIndex5, delayLine_L);

      // Read the delayed (behind) signal for the right channel
      float delayLineSig_R5 = readFrac(readIndex5, delayLine_R);

            //Read the delayed (behind) signal for the left channel
      float delayLineSig_L6 = readFrac(readIndex6, delayLine_L);

      // Read the delayed (behind) signal for the right channel
      float delayLineSig_R6 = readFrac(readIndex6, delayLine_R);

      //Read the delayed (behind) signal for the left channel
      float delayLineSig_L7 = readFrac(readIndex7, delayLine_L);

      // Read the delayed (behind) signal for the right channel
      float delayLineSig_R7 = readFrac(readIndex7, delayLine_R);

      //Read the delayed (behind) signal for the left channel
      float delayLineSig_L8 = readFrac(readIndex8, delayLine_L);

      // Read the delayed (behind) signal for the right channel
      float delayLineSig_R8 = readFrac(readIndex8, delayLine_R);

      //Write to the delay line our input signal + the delayed signal * the feedback amount (valDepth)      
      //Left delay:


      delayLine_L2[delayLine_Wr1] = sigInL + delayLineSig_L1*feedBack1*feedbackScale ; 
      // Right delay:
      delayLine_R2[delayLine_Wr1] = sigInR  + delayLineSig_R1*feedBack1*feedbackScale; 

      delayLine_L2[delayLine_Wr2] = sigInL + delayLineSig_L2*feedBack2*feedbackScale ; 
      // Right delay:
      delayLine_R2[delayLine_Wr2] = sigInR  + delayLineSig_R2*feedBack2*feedbackScale; 



            // Write to the delay line our input signal + the delayed signal * the feedback amount (valDepth)      
      // Left delay:
      delayLine_L2[delayLine_Wr3] = sigInL + (delayLineSig_L3 * feedBack3*feedbackScale); 
      // Right delay:
      delayLine_R2[delayLine_Wr3] = sigInR + (delayLineSig_R3 * feedBack3*feedbackScale); 

      // Write to the delay line our input signal + the delayed signal * the feedback amount (valDepth)      
      // Left delay:
      delayLine_L2[delayLine_Wr4] = sigInL + (delayLineSig_L4 * feedBack4*feedbackScale); 
      // Right delay:
      delayLine_R2[delayLine_Wr4] = sigInR + (delayLineSig_R4 * feedBack4*feedbackScale); 

         //    // Write to the delay line our input signal + the delayed signal * the feedback amount (valDepth)      
   //    // Left delay:
      delayLine_L[delayLine_Wr5] = sigInL + delayLineSig_L5*feedBack5*feedbackScale ; 
      // Right delay:
      delayLine_R[delayLine_Wr5] = sigInR  + delayLineSig_R5*feedBack5*feedbackScale; 

         //    // Write to the delay line our input signal + the delayed signal * the feedback amount (valDepth)      
   //    // Left delay:
      delayLine_L[delayLine_Wr6] = sigInL + delayLineSig_L6*feedBack6*feedbackScale ; 
      // Right delay:
      delayLine_R[delayLine_Wr6] = sigInR  + delayLineSig_R6*feedBack6*feedbackScale; 

         //    // Write to the delay line our input signal + the delayed signal * the feedback amount (valDepth)      
   //    // Left delay:
      delayLine_L[delayLine_Wr7] = sigInL + delayLineSig_L7*feedBack7*feedbackScale ; 
      // Right delay:
      delayLine_R[delayLine_Wr7] = sigInR  + delayLineSig_R7*feedBack8*feedbackScale; 




   //    // Write to the delay line our input signal + the delayed signal * the feedback amount (valDepth)      
   //    // Left delay:
      delayLine_L[delayLine_Wr8] = sigInL + delayLineSig_L8*feedBack8*feedbackScale ; 
      // Right delay:
      delayLine_R[delayLine_Wr8] = sigInR  + delayLineSig_R8*feedBack8*feedbackScale; 

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

      delayLine_Wr5++;
      delayLine_Wr5 &= DELAY_LINE_SIZE_MASK; 

      delayLine_Wr6++;
      delayLine_Wr6 &= DELAY_LINE_SIZE_MASK; 

      delayLine_Wr7++;
      delayLine_Wr7 &= DELAY_LINE_SIZE_MASK; 


      delayLine_Wr8++;
      delayLine_Wr8 &= DELAY_LINE_SIZE_MASK; 

      

      chorusRight = (delayLineSig_L1 + delayLineSig_R1 + delayLineSig_L3 + delayLineSig_R3);
      chorusLeft = (delayLineSig_L2 + delayLineSig_R2 + delayLineSig_L4 + delayLineSig_R4);


      //chorusRight = 0;
      reverbLeft = (delayLineSig_L5  + delayLineSig_R5  + delayLineSig_L7 + delayLineSig_R7)*reverb_mix;
      reverbRight = (delayLineSig_L6 + delayLineSig_R6 +  (delayLineSig_L8/2 ) + (delayLineSig_R8/2 ) )*reverb_mix;


      // reverbRight = (delayLineSig_R3 + delayLineSig_L3 + delayLineSig_R4 + delayLineSig_L4)*0.5;
      // chorusLeft = (delayLineSig_L5   +  delayLineSig_R5 + delayLineSig_R6  + delayLineSig_L6);
      // chorusRight = (delayLineSig_R7  +  delayLineSig_L7 + delayLineSig_R8  + delayLineSig_L8);
      
      // That is, the input signal * the dry level + (mixed with) the delayed signal * the wet level.
      sigOutL = sigInL * dry + highpass((reverbLeft + chorusLeft)*wet);
      //sigOutL = sigInL * dry + (reverbLeft)*wet;
     // sigOutL =  (reverbLeft + chorusLeft)*wet;

      // And again for the right channel
      sigOutR = sigInR * dry + highpass((reverbRight + chorusRight)*wet);
      //sigOutR = sigInR * dry + (reverbRight)*wet;
      //sigOutR = (reverbRight + chorusRight)*wet;

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
  switch (index) {
  case k_user_delfx_param_time:
   // s_lfo_wave = si_roundf(valf * (k_wave_count - 1));
   s_lfo1.setF0(0.1f + 240.f * valf, s_fs_recip);
   s_lfo2.setF0(0.1f + 240.f * valf, s_fs_recip);
   s_lfo3.setF0(0.1f + 240.f * valf, s_fs_recip);
   s_lfo4.setF0(0.1f + 240.f * valf, s_fs_recip);
   s_lfo5.setF0(0.1f + 240.f * valf, s_fs_recip);
   s_lfo6.setF0(0.1f + 240.f * valf, s_fs_recip);
   s_lfo7.setF0(0.1f + 240.f * valf, s_fs_recip);
   s_lfo8.setF0(0.1f + 240.f * valf, s_fs_recip);

   depth_scale = (valf + 0.001)*10;
    break;
  case k_user_delfx_param_depth:
           ////////////////////////////
         // "B" / DEPTH KNOB
         ////////////////////////////
         // Set the delay feedback (0-1, tbd if i use an exp table)   
         // Just store this value for the DSP loop to use.

      wet_mix = valf;
      dry = 1.0f - wet_mix; 
      wet = wet_mix;   

          
   
    break;
  case k_user_delfx_param_shift_depth:
    feedbackScale =  + valf/5;
    reverb_mix = valf/5;
    
    
    break;
  default:
    break;
  }
}





// void DELFX_PARAM(uint8_t index, int32_t value)
// {
//    const float valf = q31_to_f32(value);

//    switch (index) 
//    {

//          case k_user_delfx_param_depth:      
//          ////////////////////////////
//          // "B" / DEPTH KNOB
//          ////////////////////////////
//          // Set the delay feedback (0-1, tbd if i use an exp table)   
//          // Just store this value for the DSP loop to use.

//          wet_mix = valf;
//          dry = 1.0f - wet_mix; 
//          wet = wet_mix; 
          

//          break;

//          case k_user_delfx_param_time:
//          ////////////////////////////
//          // "A" / TIME KNOB
//          ////////////////////////////
//          // Calculate the coarse delay time (via the divisions table)

//          //Store this 0-1 value in case we need it for something else (currently we do not)
         
//          // change from 10 to 50 with time dial ie 5 + val * 5 
//          // timeval = valf;
//          // speed_param = 5 + timeval*10;

//          // s_lfo1.setF0(3.5f*speed_param,s_fs_recip);


//          // s_lfo2.setF0(3.8f*speed_param,s_fs_recip);

 
//          // s_lfo3.setF0(4.7f*speed_param,s_fs_recip);

   
//          // s_lfo4.setF0(3.3f*speed_param,s_fs_recip);

//          // s_lfo6.setF0(2.4f*speed_param,s_fs_recip);


//          // s_lfo6.setF0(2.1f*speed_param,s_fs_recip);

 
//          // s_lfo7.setF0(2.8f*speed_param,s_fs_recip);

   
//          // s_lfo8.setF0(1.9f*speed_param,s_fs_recip);

//          // mod_depth = 10 + timeval*2;
//          // // change from 15 to 30 with a number ie 1 + val * 2 
  
//          break;

//          case k_user_delfx_param_shift_depth: 

//          // valShift = valf*3;

         


//          break;
//          default:
//          // no default handling, there is no case for this.         
//          break;
//    }
   
// }


/*
        {0.29 ,0.0236,0.03,0.036,0.028,0.00,0.00,0.00,0.00, -- delay times 
            0.9,0.9,0.9,0.9,0.0,0.0,0.0,0.0,
            0.9,0.9,0.9,0.9,0.4,0.4,0.4,0.4,
            0.65,0.57,0.48,0.44,0.0,0.0,0.0,0.0, -- mod rate 
            0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.5,1.0, 1.0,
            0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0 // pans


*/
