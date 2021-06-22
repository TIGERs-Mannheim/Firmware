/* ----------------------------------------------------------------------   
* $Date:        19. May 2012  
* $Revision:    V1.0.0  
*   
* Project:      CMSIS DSP Library   
* Title:        arm_atan2_f32.c
*   
* Description:  Fast atan2 calculation for floating-point values.  
*   
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0  
* -------------------------------------------------------------------- */

#include "arm_math.h"

#define TABLE_SIZE_64 64

/**   
 * @ingroup groupFastMath   
 */

/**   
 * @defgroup atan2 arc tangent
 *   
 * Computes the trigonometric atan2 function using a combination of table lookup  
 * and cubic interpolation.
 * atan2(y, x) = atan( y / x )
 *
 * The lookup table only contains values for angles [0  pi/4],
 * other values are calculated case sensitive depending on magnitude proportion
 * and negativeness.
 *  
 * The implementation is based on table lookup using 64 values together with cubic interpolation.  
 * The steps used are:  
 *  -# Calculation of the nearest integer table index  
 *  -# Fetch the four table values a, b, c, and d    
 *  -# Compute the fractional portion (fract) of the table index.  
 *  -# Calculation of wa, wb, wc, wd   
 *  -# The final result equals <code>a*wa + b*wb + c*wc + d*wd</code>  
 *  
 * where  
 * <pre>   
 *    a=Table[index-1];   
 *    b=Table[index+0];   
 *    c=Table[index+1];   
 *    d=Table[index+2];   
 * </pre>  
 * and  
 * <pre>   
 *    wa=-(1/6)*fract.^3 + (1/2)*fract.^2 - (1/3)*fract;   
 *    wb=(1/2)*fract.^3 - fract.^2 - (1/2)*fract + 1;   
 *    wc=-(1/2)*fract.^3+(1/2)*fract.^2+fract;   
 *    wd=(1/6)*fract.^3 - (1/6)*fract;   
 * </pre>   
 */

/**   
 * @addtogroup atan2   
 * @{   
 */

 /* pi value is  3.14159265358979 */


static const float32_t atanTable[68] = {
  -0.015623728620477f,
  0.000000000000000f,  // = 0 for in = 0.0
  0.015623728620477f,
  0.031239833430268f,
  0.046840712915970f,
  0.062418809995957f,
  0.077966633831542f,
  0.093476781158590f,
  0.108941956989866f,
  0.124354994546761f,
  0.139708874289164f,
  0.154996741923941f,
  0.170211925285474f,
  0.185347949995695f,
  0.200398553825879f,
  0.215357699697738f,
  0.230219587276844f,
  0.244978663126864f,
  0.259629629408258f,
  0.274167451119659f,
  0.288587361894077f,
  0.302884868374971f,
  0.317055753209147f,
  0.331096076704132f,
  0.345002177207105f,
  0.358770670270572f,
  0.372398446676754f,
  0.385882669398074f,
  0.399220769575253f,
  0.412410441597387f,
  0.425449637370042f,
  0.438336559857958f,
  0.451069655988523f,
  0.463647609000806f,
  0.476069330322761f,
  0.488333951056406f,
  0.500440813147294f,
  0.512389460310738f,
  0.524179628782913f,
  0.535811237960464f,
  0.547284380987437f,
  0.558599315343562f,
  0.569756453482978f,
  0.580756353567670f,
  0.591599710335111f,
  0.602287346134964f,
  0.612820202165241f,
  0.623199329934066f,
  0.633425882969145f,
  0.643501108793284f,
  0.653426341180762f,
  0.663202992706093f,
  0.672832547593763f,
  0.682316554874748f,
  0.691656621853200f,
  0.700854407884450f,
  0.709911618463525f,
  0.718829999621625f,
  0.727611332626511f,
  0.736257428981428f,
  0.744770125716075f,
  0.753151280962194f,
  0.761402769805578f,
  0.769526480405658f,
  0.777524310373348f,
  0.785398163397448f,  // = pi/4 for in = 1.0
  0.793149946109655f,
  0.800781565178043f
};


/**  
 * @brief  Fast approximation to the trigonometric atan2 function for floating-point data.  
 * @param[in] x input value, y input value.
 * @return  atan2(y, x) = atan(y/x) as radians.
 */

float32_t arm_atan2_f32(float32_t y, float32_t x)
{
  float32_t atan2Val, fract, in;                 /* Temporary variables for input, output */
  uint32_t index;                                /* Index variable */
  uint32_t tableSize = (uint32_t) TABLE_SIZE_64; /* Initialise tablesize */
  float32_t wa, wb, wc, wd;                      /* Cubic interpolation coefficients */
  float32_t a, b, c, d;                          /* Four nearest output values */
  float32_t *tablePtr;                           /* Pointer to table */
  uint8_t flags = 0;                             /* flags providing information about input values: 
                                                    Bit0 = 1 if |x| < |y|
                                                    Bit1 = 1 if x < 0
                                                    Bit2 = 1 if y < 0 */

  /* calculate magnitude of input values */
  if (x < 0.0f) {
    x = -x;
    flags |= 0x02;
  }

  if (y < 0.0f) {
    y = -y;
    flags |= 0x04;
  }

  /* calculate in value for LUT [0 1] */
  if (x < y) {
    in = x / y;
    flags |= 0x01;
  }
  else /* x >= y */ {
    if (x > 0.0f)
      in = y / x;
    else /* both are 0.0 */
      in = 0.0; /* prevent division by 0 */
  }

  /* Calculation of index of the table */
  index = (uint32_t) (tableSize * in);

  /* fractional value calculation */
  fract = ((float32_t) tableSize * in) - (float32_t) index;

  /* Initialise table pointer */
  tablePtr = (float32_t *) & atanTable[index];

  /* Read four nearest values of output value from the atan table */
  a = *tablePtr++;
  b = *tablePtr++;
  c = *tablePtr++;
  d = *tablePtr++;

  /* Cubic interpolation process */
  wa = -(((0.166666667f) * (fract * (fract * fract))) +
         ((0.3333333333333f) * fract)) + ((0.5f) * (fract * fract));
  wb = (((0.5f) * (fract * (fract * fract))) -
        ((fract * fract) + ((0.5f) * fract))) + 1.0f;
  wc = (-((0.5f) * (fract * (fract * fract))) +
        ((0.5f) * (fract * fract))) + fract;
  wd = ((0.166666667f) * (fract * (fract * fract))) -
    ((0.166666667f) * fract);

  /* Calculate atan2 value */
  atan2Val = ((a * wa) + (b * wb)) + ((c * wc) + (d * wd));

  /* exchanged input values? */
  if (flags & 0x01)
    /* output = pi/2 - output */
    atan2Val = 1.5707963267949f - atan2Val;

  /* negative x input? Quadrant 2 or 3 */
  if (flags & 0x02)
    atan2Val = 3.14159265358979f - atan2Val;

  /* negative y input? Quadrant 3 or 4 */
  if (flags & 0x04)
    atan2Val = - atan2Val;

  /* Return the output value */
  return (atan2Val);
}

/**   
 * @} end of atan2 group   
 */
