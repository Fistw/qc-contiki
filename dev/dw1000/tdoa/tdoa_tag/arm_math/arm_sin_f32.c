/* ----------------------------------------------------------------------
* Copyright (C) 2010-2014 ARM Limited. All rights reserved.
*
* $Date:        21. September 2015
* $Revision:    V.1.4.5 a
*
* Project:      CMSIS DSP Library
* Title:        arm_sin_f32.c
*
* Description:  Fast sine calculation for floating-point values.
*
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* -------------------------------------------------------------------- */

#include "arm_math.h"
#include <math.h>

/**
 * @ingroup groupFastMath
 */

/**
 * @defgroup sin Sine
 *
 * Computes the trigonometric sine function using a combination of table lookup
 * and linear interpolation.  There are separate functions for
 * Q15, Q31, and floating-point data types.
 * The input to the floating-point version is in radians while the
 * fixed-point Q15 and Q31 have a scaled input with the range
 * [0 +0.9999] mapping to [0 2*pi).  The fixed-point range is chosen so that a
 * value of 2*pi wraps around to 0.
 *
 * The implementation is based on table lookup using 256 values together with linear interpolation.
 * The steps used are:
 *  -# Calculation of the nearest integer table index
 *  -# Compute the fractional portion (fract) of the table index.
 *  -# The final result equals <code>(1.0f-fract)*a + fract*b;</code>
 *
 * where
 * <pre>
 *    b=Table[index+0];
 *    c=Table[index+1];
 * </pre>
 */

/**
 * @addtogroup sin
 * @{
 */

/**
 * @brief  Fast approximation to the trigonometric sine function for floating-point data.
 * @param[in] x input value in radians.
 * @return  sin(x).
 */

float arm_sin_f32(
  float x)
{
  float sinVal, fract, in;                           /* Temporary variables for input, output */
  uint16_t index;                                        /* Index variable */
  float a, b;                                        /* Two nearest output values */
  int32_t n;
  float findex;

  /* input x is in radians */
  /* Scale the input to [0 1] range from [0 2*PI] , divide input by 2*pi */
  in = x * 0.159154943092f;

  /* Calculation of floor value of input */
  n = (int32_t) in;

  /* Make negative values towards -infinity */
  if(x < 0.0f)
  {
    n--;
  }

  /* Map input value to [0 1] */
  in = in - (float) n;

  /* Calculation of index of the table */
  findex = (float) FAST_MATH_TABLE_SIZE * in;
  if (findex >= 512.0f) {
    findex -= 512.0f;
  }

  index = ((uint16_t)findex) & 0x1ff;

  /* fractional value calculation */
  fract = findex - (float) index;

  /* Read two nearest values of input value from the sin table */
  a = sinTable_f32[index];
  b = sinTable_f32[index+1];

  /* Linear interpolation process */
  sinVal = (1.0f-fract)*a + fract*b;

  /* Return the output value */
  return (sinVal);
}

/**
 * @} end of sin group
 */