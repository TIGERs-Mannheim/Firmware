#pragma once

/**
 * This file was generated automatically. The values were
 * fitted by using a simple feed-forward neural network.
 */

static inline float irPredictBallDistance(float* x)
{

	/* Layer: dense_3 */
	float after_layer0[8] = { 0. };
	/* Updating for Row 0 */
	after_layer0[0] += -0.08305301*x[0];
	after_layer0[1] += 0.27733684*x[0];
	after_layer0[2] += 0.17470638*x[0];
	after_layer0[3] += -0.3245281*x[0];
	after_layer0[4] += 0.41164494*x[0];
	after_layer0[5] += -0.17286175*x[0];
	after_layer0[6] += -0.064268366*x[0];
	after_layer0[7] += -0.0028433837*x[0];
	/* Updating for Row 1 */
	after_layer0[0] += 0.16458191*x[1];
	after_layer0[1] += -0.36647394*x[1];
	after_layer0[2] += 0.11775182*x[1];
	after_layer0[3] += 0.14008771*x[1];
	after_layer0[4] += -0.20240043*x[1];
	after_layer0[5] += 0.15178603*x[1];
	after_layer0[6] += 0.3083653*x[1];
	after_layer0[7] += -0.01109991*x[1];
	/* Updating for Row 2 */
	after_layer0[0] += -0.023187537*x[2];
	after_layer0[1] += -0.13888009*x[2];
	after_layer0[2] += -0.100584306*x[2];
	after_layer0[3] += 0.0045476626*x[2];
	after_layer0[4] += 0.08070421*x[2];
	after_layer0[5] += -0.10006642*x[2];
	after_layer0[6] += 0.006927722*x[2];
	after_layer0[7] += 0.17901354*x[2];
	/* Updating for Row 3 */
	after_layer0[0] += 0.03297198*x[3];
	after_layer0[1] += -0.112886384*x[3];
	after_layer0[2] += -0.088600755*x[3];
	after_layer0[3] += 0.175467*x[3];
	after_layer0[4] += -0.13631342*x[3];
	after_layer0[5] += 0.16333789*x[3];
	after_layer0[6] += 0.08190505*x[3];
	after_layer0[7] += 0.11231053*x[3];
	/* Updating for Row 4 */
	after_layer0[0] += -0.014366996*x[4];
	after_layer0[1] += 0.043696452*x[4];
	after_layer0[2] += 0.036067657*x[4];
	after_layer0[3] += -0.048763316*x[4];
	after_layer0[4] += 0.06585059*x[4];
	after_layer0[5] += -0.06661976*x[4];
	after_layer0[6] += -0.037242044*x[4];
	after_layer0[7] += -0.040572736*x[4];
	/* Updating for Row 5 */
	after_layer0[0] += 0.014658581*x[5];
	after_layer0[1] += -0.04660083*x[5];
	after_layer0[2] += -0.03780005*x[5];
	after_layer0[3] += 0.064768896*x[5];
	after_layer0[4] += -0.08175139*x[5];
	after_layer0[5] += 0.09446787*x[5];
	after_layer0[6] += 0.039142895*x[5];
	after_layer0[7] += 0.043819*x[5];
	/* Updating for Row 6 */
	after_layer0[0] += 0.13168855*x[6];
	after_layer0[1] += -0.31296495*x[6];
	after_layer0[2] += -0.25549373*x[6];
	after_layer0[3] += 0.3893344*x[6];
	after_layer0[4] += -0.5054045*x[6];
	after_layer0[5] += 0.49222276*x[6];
	after_layer0[6] += 0.28983328*x[6];
	after_layer0[7] += 0.24401122*x[6];
	/* Updating for Row 7 */
	after_layer0[0] += -0.06717396*x[7];
	after_layer0[1] += 0.031116745*x[7];
	after_layer0[2] += -0.00031038543*x[7];
	after_layer0[3] += 0.14420882*x[7];
	after_layer0[4] += 0.22402938*x[7];
	after_layer0[5] += -0.22116421*x[7];
	after_layer0[6] += -0.032136284*x[7];
	after_layer0[7] += -0.0017893285*x[7];
	/* Updating for Row 8 */
	after_layer0[0] += -0.1190765*x[8];
	after_layer0[1] += -0.027213428*x[8];
	after_layer0[2] += -0.04198213*x[8];
	after_layer0[3] += 0.06065752*x[8];
	after_layer0[4] += 0.16296183*x[8];
	after_layer0[5] += 0.08116885*x[8];
	after_layer0[6] += -0.07087704*x[8];
	after_layer0[7] += 0.13780762*x[8];
	/* Updating for Row 9 */
	after_layer0[0] += 0.030810628*x[9];
	after_layer0[1] += -0.12748891*x[9];
	after_layer0[2] += -0.06024428*x[9];
	after_layer0[3] += 0.13439359*x[9];
	after_layer0[4] += -0.22331747*x[9];
	after_layer0[5] += 0.18287039*x[9];
	after_layer0[6] += 0.104867116*x[9];
	after_layer0[7] += 0.08766726*x[9];
	/* Updating for Row 10 */
	after_layer0[0] += 0.0041134804*x[10];
	after_layer0[1] += -0.021757746*x[10];
	after_layer0[2] += -0.014943833*x[10];
	after_layer0[3] += 0.04972034*x[10];
	after_layer0[4] += -0.041689973*x[10];
	after_layer0[5] += 0.04963132*x[10];
	after_layer0[6] += 0.014108303*x[10];
	after_layer0[7] += 0.017956961*x[10];
	/* Updating for Row 11 */
	after_layer0[0] += 0.040457375*x[11];
	after_layer0[1] += -0.12933053*x[11];
	after_layer0[2] += -0.10449144*x[11];
	after_layer0[3] += 0.18308951*x[11];
	after_layer0[4] += -0.22586127*x[11];
	after_layer0[5] += 0.2640137*x[11];
	after_layer0[6] += 0.10790407*x[11];
	after_layer0[7] += 0.12222208*x[11];
	/* Updating for Row 12 */
	after_layer0[0] += 0.041938692*x[12];
	after_layer0[1] += -0.18977465*x[12];
	after_layer0[2] += -0.14558926*x[12];
	after_layer0[3] += 0.3545872*x[12];
	after_layer0[4] += -0.3252892*x[12];
	after_layer0[5] += 0.25237188*x[12];
	after_layer0[6] += 0.145513*x[12];
	after_layer0[7] += 0.16738683*x[12];
	/* Updating for Row 13 */
	after_layer0[0] += -0.045965567*x[13];
	after_layer0[1] += 0.2459732*x[13];
	after_layer0[2] += 0.0397688*x[13];
	after_layer0[3] += -0.14964256*x[13];
	after_layer0[4] += 0.25103828*x[13];
	after_layer0[5] += -0.24381216*x[13];
	after_layer0[6] += -0.20290731*x[13];
	after_layer0[7] += -0.13625999*x[13];
	/* Updating for Row 14 */
	after_layer0[0] += -0.009016969*x[14];
	after_layer0[1] += -0.062249143*x[14];
	after_layer0[2] += 0.058338027*x[14];
	after_layer0[3] += -0.00070477865*x[14];
	after_layer0[4] += -0.16499658*x[14];
	after_layer0[5] += -0.40818217*x[14];
	after_layer0[6] += 0.00044340134*x[14];
	after_layer0[7] += -0.2981761*x[14];
	/* Updating for Row 15 */
	after_layer0[0] += 0.03418754*x[15];
	after_layer0[1] += -6.0911712e-05*x[15];
	after_layer0[2] += -0.10258937*x[15];
	after_layer0[3] += 0.29132968*x[15];
	after_layer0[4] += -0.36024594*x[15];
	after_layer0[5] += 0.31726012*x[15];
	after_layer0[6] += 0.031214567*x[15];
	after_layer0[7] += -0.031240797*x[15];
	/* Updating for Row 16 */
	after_layer0[0] += -0.050731346*x[16];
	after_layer0[1] += 0.12994021*x[16];
	after_layer0[2] += 0.13642655*x[16];
	after_layer0[3] += -0.23179032*x[16];
	after_layer0[4] += 0.12015174*x[16];
	after_layer0[5] += -0.19989711*x[16];
	after_layer0[6] += -0.09458945*x[16];
	after_layer0[7] += -0.14198165*x[16];
	/* Updating for Row 17 */
	after_layer0[0] += 0.05999219*x[17];
	after_layer0[1] += -0.18935432*x[17];
	after_layer0[2] += -0.15333386*x[17];
	after_layer0[3] += 0.28334036*x[17];
	after_layer0[4] += -0.30526906*x[17];
	after_layer0[5] += 0.38242862*x[17];
	after_layer0[6] += 0.15670948*x[17];
	after_layer0[7] += 0.18719122*x[17];
	/* Updating for Row 18 */
	after_layer0[0] += 0.07988953*x[18];
	after_layer0[1] += -0.2553011*x[18];
	after_layer0[2] += -0.20717373*x[18];
	after_layer0[3] += 0.36666855*x[18];
	after_layer0[4] += -0.39384007*x[18];
	after_layer0[5] += 0.5019246*x[18];
	after_layer0[6] += 0.2095088*x[18];
	after_layer0[7] += 0.2374452*x[18];
	/* Updating for Row 19 */
	after_layer0[0] += 0.0023268608*x[19];
	after_layer0[1] += 0.035707425*x[19];
	after_layer0[2] += 0.0043249703*x[19];
	after_layer0[3] += -0.013441*x[19];
	after_layer0[4] += -0.019758621*x[19];
	after_layer0[5] += -0.034150604*x[19];
	after_layer0[6] += 0.011393195*x[19];
	after_layer0[7] += 0.06748019*x[19];
	/* Updating for Row 20 */
	after_layer0[0] += 0.056778878*x[20];
	after_layer0[1] += -0.2402483*x[20];
	after_layer0[2] += -0.3314847*x[20];
	after_layer0[3] += 0.16899852*x[20];
	after_layer0[4] += -0.08477899*x[20];
	after_layer0[5] += 0.22635174*x[20];
	after_layer0[6] += 0.067216285*x[20];
	after_layer0[7] += 0.018452508*x[20];
	/* Updating for Row 21 */
	after_layer0[0] += 0.098556146*x[21];
	after_layer0[1] += -0.14586182*x[21];
	after_layer0[2] += -0.056953337*x[21];
	after_layer0[3] += 0.28089133*x[21];
	after_layer0[4] += 0.29935896*x[21];
	after_layer0[5] += -0.060113735*x[21];
	after_layer0[6] += 0.10754623*x[21];
	after_layer0[7] += -0.0021479342*x[21];
	/* Updating for Row 22 */
	after_layer0[0] += -0.031912643*x[22];
	after_layer0[1] += 0.21777843*x[22];
	after_layer0[2] += 0.12040893*x[22];
	after_layer0[3] += -0.05537945*x[22];
	after_layer0[4] += 0.10733569*x[22];
	after_layer0[5] += -0.19701551*x[22];
	after_layer0[6] += -0.1588619*x[22];
	after_layer0[7] += 0.04967501*x[22];
	/* Updating for Row 23 */
	after_layer0[0] += 0.08198657*x[23];
	after_layer0[1] += -0.26555088*x[23];
	after_layer0[2] += -0.22194642*x[23];
	after_layer0[3] += 0.43022764*x[23];
	after_layer0[4] += -0.37109601*x[23];
	after_layer0[5] += 0.6391734*x[23];
	after_layer0[6] += 0.26226223*x[23];
	after_layer0[7] += 0.22519322*x[23];
	/* Updating for Row 24 */
	after_layer0[0] += 0.0148055265*x[24];
	after_layer0[1] += -0.047036946*x[24];
	after_layer0[2] += -0.03784945*x[24];
	after_layer0[3] += 0.067149326*x[24];
	after_layer0[4] += -0.08385513*x[24];
	after_layer0[5] += 0.099858575*x[24];
	after_layer0[6] += 0.039115503*x[24];
	after_layer0[7] += 0.044429403*x[24];
	/* Updating for Row 25 */
	after_layer0[0] += 0.0030523236*x[25];
	after_layer0[1] += -0.014183238*x[25];
	after_layer0[2] += -0.009096347*x[25];
	after_layer0[3] += 0.040407356*x[25];
	after_layer0[4] += -0.03380987*x[25];
	after_layer0[5] += 0.032443997*x[25];
	after_layer0[6] += 0.009705909*x[25];
	after_layer0[7] += 0.018833948*x[25];
	/* Updating for Row 26 */
	after_layer0[0] += 0.04030235*x[26];
	after_layer0[1] += -0.17002565*x[26];
	after_layer0[2] += -0.097376205*x[26];
	after_layer0[3] += 0.2544863*x[26];
	after_layer0[4] += -0.21706948*x[26];
	after_layer0[5] += 0.146346*x[26];
	after_layer0[6] += 0.108575776*x[26];
	after_layer0[7] += 0.14012453*x[26];
	/* Updating for Row 27 */
	after_layer0[0] += -0.038376223*x[27];
	after_layer0[1] += 0.07025343*x[27];
	after_layer0[2] += -0.10265439*x[27];
	after_layer0[3] += -0.10717103*x[27];
	after_layer0[4] += 0.15729183*x[27];
	after_layer0[5] += -0.047926646*x[27];
	after_layer0[6] += -0.17537151*x[27];
	after_layer0[7] += -0.18757848*x[27];
	/* Updating for Row 28 */
	after_layer0[0] += -0.03462011*x[28];
	after_layer0[1] += 0.17044592*x[28];
	after_layer0[2] += 0.2034021*x[28];
	after_layer0[3] += -0.021028668*x[28];
	after_layer0[4] += 0.051077437*x[28];
	after_layer0[5] += 0.037705112*x[28];
	after_layer0[6] += 0.083994895*x[28];
	after_layer0[7] += -0.22863759*x[28];
	/* Updating for Row 29 */
	after_layer0[0] += 0.11011339*x[29];
	after_layer0[1] += -0.17568204*x[29];
	after_layer0[2] += -0.28924048*x[29];
	after_layer0[3] += 0.28090024*x[29];
	after_layer0[4] += -0.5440706*x[29];
	after_layer0[5] += 0.67323107*x[29];
	after_layer0[6] += 0.42063457*x[29];
	after_layer0[7] += 0.3075822*x[29];
	/* Updating for Row 30 */
	after_layer0[0] += 0.0009111105*x[30];
	after_layer0[1] += -0.0018124933*x[30];
	after_layer0[2] += -0.001593168*x[30];
	after_layer0[3] += 0.002222994*x[30];
	after_layer0[4] += -0.0026870093*x[30];
	after_layer0[5] += 0.0031087177*x[30];
	after_layer0[6] += 0.0016258947*x[30];
	after_layer0[7] += 0.001670733*x[30];
	/* Updating for Row 31 */
	after_layer0[0] += 0.003156567*x[31];
	after_layer0[1] += -0.010982409*x[31];
	after_layer0[2] += -0.008535728*x[31];
	after_layer0[3] += 0.019897614*x[31];
	after_layer0[4] += -0.023164956*x[31];
	after_layer0[5] += 0.029001169*x[31];
	after_layer0[6] += 0.008725622*x[31];
	after_layer0[7] += 0.010781786*x[31];
	/* Updating for Row 32 */
	after_layer0[0] += 0.027498946*x[32];
	after_layer0[1] += -0.0918857*x[32];
	after_layer0[2] += -0.07693695*x[32];
	after_layer0[3] += 0.12756285*x[32];
	after_layer0[4] += -0.15028068*x[32];
	after_layer0[5] += 0.21349552*x[32];
	after_layer0[6] += 0.076994784*x[32];
	after_layer0[7] += 0.094466545*x[32];
	/* Updating for Row 33 */
	after_layer0[0] += -0.05866766*x[33];
	after_layer0[1] += -0.048030023*x[33];
	after_layer0[2] += 0.16797481*x[33];
	after_layer0[3] += -0.15991722*x[33];
	after_layer0[4] += 0.17040572*x[33];
	after_layer0[5] += -0.092876956*x[33];
	after_layer0[6] += -0.18762606*x[33];
	after_layer0[7] += -0.21344669*x[33];
	/* Updating for Row 34 */
	after_layer0[0] += 0.06892773*x[34];
	after_layer0[1] += 0.060526393*x[34];
	after_layer0[2] += -0.24000098*x[34];
	after_layer0[3] += 0.10631979*x[34];
	after_layer0[4] += -0.17068715*x[34];
	after_layer0[5] += 0.39177626*x[34];
	after_layer0[6] += 0.27842984*x[34];
	after_layer0[7] += 0.31149885*x[34];
	/* Updating for Row 35 */
	after_layer0[0] += -0.03427255*x[35];
	after_layer0[1] += 0.103256285*x[35];
	after_layer0[2] += 0.10058201*x[35];
	after_layer0[3] += -0.093303934*x[35];
	after_layer0[4] += 0.38553107*x[35];
	after_layer0[5] += -0.5824013*x[35];
	after_layer0[6] += -0.05203003*x[35];
	after_layer0[7] += -0.05919079*x[35];
	/* Applying Bias*/
	after_layer0[0] += 0.19167809;
	after_layer0[1] += -0.1441257;
	after_layer0[2] += -0.15986964;
	after_layer0[3] += 0.13770485;
	after_layer0[4] += -0.14011563;
	after_layer0[5] += 0.16988541;
	after_layer0[6] += 0.16507642;
	after_layer0[7] += 0.13038667;
	/*******************************************************/

	/* Layer: dense_4 */
	float after_layer1[4] = { 0. };
	/* Updating for Row 0 */
	after_layer1[0] += -0.004729009*after_layer0[0];
	after_layer1[1] += 0.22294877*after_layer0[0];
	after_layer1[2] += -0.76508415*after_layer0[0];
	after_layer1[3] += -0.0044361735*after_layer0[0];
	/* Updating for Row 1 */
	after_layer1[0] += -0.3925563*after_layer0[1];
	after_layer1[1] += 0.7525281*after_layer0[1];
	after_layer1[2] += 0.82364607*after_layer0[1];
	after_layer1[3] += -0.15412062*after_layer0[1];
	/* Updating for Row 2 */
	after_layer1[0] += -0.41662344*after_layer0[2];
	after_layer1[1] += 0.32436582*after_layer0[2];
	after_layer1[2] += 0.1541284*after_layer0[2];
	after_layer1[3] += -0.8856523*after_layer0[2];
	/* Updating for Row 3 */
	after_layer1[0] += 0.48664874*after_layer0[3];
	after_layer1[1] += -0.65424144*after_layer0[3];
	after_layer1[2] += -0.8651054*after_layer0[3];
	after_layer1[3] += 0.85948*after_layer0[3];
	/* Updating for Row 4 */
	after_layer1[0] += -0.4264667*after_layer0[4];
	after_layer1[1] += 1.1512091*after_layer0[4];
	after_layer1[2] += 1.0375868*after_layer0[4];
	after_layer1[3] += -1.0503079*after_layer0[4];
	/* Updating for Row 5 */
	after_layer1[0] += 0.559439*after_layer0[5];
	after_layer1[1] += -1.3933512*after_layer0[5];
	after_layer1[2] += -1.2862836*after_layer0[5];
	after_layer1[3] += 0.9692855*after_layer0[5];
	/* Updating for Row 6 */
	after_layer1[0] += 0.15801497*after_layer0[6];
	after_layer1[1] += -0.73843974*after_layer0[6];
	after_layer1[2] += -0.38499197*after_layer0[6];
	after_layer1[3] += 0.42911395*after_layer0[6];
	/* Updating for Row 7 */
	after_layer1[0] += 0.09417332*after_layer0[7];
	after_layer1[1] += -0.43031672*after_layer0[7];
	after_layer1[2] += -0.618606*after_layer0[7];
	after_layer1[3] += 0.66268677*after_layer0[7];
	/* Applying Bias*/
	after_layer1[0] += 0.12890418;
	after_layer1[1] += -0.14599833;
	after_layer1[2] += -0.124214366;
	after_layer1[3] += 0.12351139;
	/*******************************************************/

	/* Layer: dense_5 */
	float after_layer2[1] = { 0. };
	/* Updating for Row 0 */
	after_layer2[0] += 0.28873855*after_layer1[0];
	/* Updating for Row 1 */
	after_layer2[0] += -0.6629163*after_layer1[1];
	/* Updating for Row 2 */
	after_layer2[0] += -0.7527812*after_layer1[2];
	/* Updating for Row 3 */
	after_layer2[0] += 0.7172274*after_layer1[3];
	/* Applying Bias*/
	after_layer2[0] += 0.10978819;
	/*******************************************************/

	return after_layer2[0];
}
