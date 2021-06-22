/*
 * arm_mat_util_f32.c
 *
 *  Created on: 18.03.2013
 *      Author: AndreR
 */

#include "arm_mat_util_f32.h"
#include "util/console.h"
#include <string.h>

#define SVD_MAX_SIZE 10

#define SIGNF(a, b) ((b) >= 0.0 ? fabsf(a) : -fabsf(a))
#define MAX(x,y) ((x)>(y)?(x):(y))
static float PYTHAGF(float a, float b);

void arm_mat_zero_f32(arm_matrix_instance_f32* pMat)
{
	memset(pMat->pData, 0, sizeof(float) * pMat->numCols * pMat->numRows);
}

void arm_mat_fill_f32(arm_matrix_instance_f32* pMat, float value)
{
	for(uint32_t i = 0; i < pMat->numCols * pMat->numRows; i++)
		pMat->pData[i] = value;
}

float arm_mat_abs_max_f32(arm_matrix_instance_f32* pMat)
{
	float max = 0;
	for(uint32_t i = 0; i < pMat->numCols * pMat->numRows; i++)
	{
		float val = fabsf(pMat->pData[i]);
		if(val > max)
			max = val;
	}

	return max;
}

arm_status arm_mat_identity_f32(arm_matrix_instance_f32* pMat)
{
#ifdef ARM_MATH_MATRIX_CHECK
	if(pMat->numCols != pMat->numRows)
	return ARM_MAT_SIZE_MISMATCH;
#endif /*      #ifdef ARM_MATH_MATRIX_CHECK    */

	arm_mat_zero_f32(pMat);

	for(uint16_t i = 0; i < pMat->numCols; i++)
		pMat->pData[i * pMat->numCols + i] = 1.0f;

	return ARM_MATH_SUCCESS;
}

/**
 * Danger! Assumes that pData storage is large enough.
 */
void arm_mat_copy_f32(const arm_matrix_instance_f32* pSrc, arm_matrix_instance_f32* pDst)
{
	pDst->numCols = pSrc->numCols;
	pDst->numRows = pSrc->numRows;

	memcpy(pDst->pData, pSrc->pData, sizeof(float) * pSrc->numCols * pSrc->numRows);
}

void arm_mat_clip_f32(arm_matrix_instance_f32* pMat, float min, float max)
{
	for(uint32_t i = 0; i < pMat->numCols * pMat->numRows; i++)
	{
		if(pMat->pData[i] > max)
			pMat->pData[i] = max;
		if(pMat->pData[i] < min)
			pMat->pData[i] = min;
	}
}

void arm_mat_print(const arm_matrix_instance_f32* pMat)
{
	for(uint16_t r = 0; r < pMat->numRows; r++)
	{
		for(uint16_t c = 0; c < pMat->numCols; c++)
		{
			ConsolePrint("% 3.7f  ", MAT_ELEMENT(*pMat, r, c));
		}

		ConsolePrint("\r\n");
	}
}

#define DET2(a, b, c, d) (a*d-b*c)

arm_status arm_mat_inv_2x2_f32(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* B)
{
#ifdef ARM_MATH_MATRIX_CHECK
	if(A->numCols != 2 || A->numRows != 2)
		return ARM_MAT_SIZE_MISMATCH;
#endif

	float det = DET2(A->pData[0], A->pData[1], A->pData[2], A->pData[3]);
	if(det == 0.0f)
		return ARM_MATH_SINGULAR;

	float f = 1.0f/det;

	B->pData[0] =  f*A->pData[3];
	B->pData[1] = -f*A->pData[1];
	B->pData[2] = -f*A->pData[2];
	B->pData[3] =  f*A->pData[0];

	return ARM_MATH_SUCCESS;
}

uint8_t arm_mat_is_nan_f32(const arm_matrix_instance_f32* A)
{
	for(uint32_t i = 0; i < A->numCols*A->numRows; i++)
	{
		if(isnanf(A->pData[i]))
			return 1;
	}

	return 0;
}

arm_status arm_mat_inv_3x3_f32(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* B)
{
#ifdef ARM_MATH_MATRIX_CHECK
	if(A->numCols != 3 || A->numRows != 3)
		return ARM_MAT_SIZE_MISMATCH;
#endif

	const float* a = A->pData;
	const float* b = A->pData+3;
	const float* c = A->pData+6;

	float det = a[0]*b[1]*c[2] - a[0]*b[2]*c[1] - a[1]*b[0]*c[2] + a[1]*b[2]*c[0] + a[2]*b[0]*c[1] - a[2]*b[1]*c[0];
	if(det == 0.0f)
		return ARM_MATH_SINGULAR;

	float f = 1.0f/det;

	a = A->pData;

	B->pData[0] = f*DET2(a[4], a[5], a[7], a[8]);
	B->pData[1] = f*DET2(a[2], a[1], a[8], a[7]);
	B->pData[2] = f*DET2(a[1], a[2], a[4], a[5]);
	B->pData[3] = f*DET2(a[5], a[3], a[8], a[6]);
	B->pData[4] = f*DET2(a[0], a[2], a[6], a[8]);
	B->pData[5] = f*DET2(a[2], a[1], a[5], a[3]);
	B->pData[6] = f*DET2(a[3], a[4], a[6], a[7]);
	B->pData[7] = f*DET2(a[1], a[0], a[7], a[6]);
	B->pData[8] = f*DET2(a[0], a[1], a[3], a[4]);

	return ARM_MATH_SUCCESS;
}

arm_status arm_mat_pinv(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* B)
{
	// stack usage 4*10*10 = 400B
	// maximum matrix size is SVD_MAX_SIZE * SVD_MAX_SIZE
	uint8_t transposed = 0;
	arm_matrix_instance_f32 U = {A->numRows, A->numCols, (float[SVD_MAX_SIZE*SVD_MAX_SIZE]){}};

	if(A->numRows < A->numCols)
	{
		U.numRows = A->numCols;
		U.numCols = A->numRows;
		arm_mat_trans_f32(A, &U);
		transposed = 1;
	}
	else
	{
		U.numRows = A->numRows;
		U.numCols = A->numCols;
		memcpy(U.pData, A->pData, sizeof(float)*A->numRows*A->numCols);
	}

	arm_matrix_instance_f32 S = {U.numCols, U.numCols, (float[SVD_MAX_SIZE*SVD_MAX_SIZE]){}};
	arm_matrix_instance_f32 V = {U.numCols, U.numCols, (float[SVD_MAX_SIZE*SVD_MAX_SIZE]){}};
	arm_matrix_instance_f32 tmp1 = {U.numCols, U.numRows, (float[SVD_MAX_SIZE*SVD_MAX_SIZE]){}};

	arm_status svdResult = arm_mat_svd(&U, &S, &V);
	if(svdResult)
		return svdResult;

	// Moore-Penrose pseudo-inverse
	// >>> V * inv(S) * U'

	// S = inv(S)
	for(uint16_t i = 0; i < S.numCols; i++)
	{
		if(MAT_ELEMENT(S, i, i) != 0.0f)
			MAT_ELEMENT(S, i, i) = 1.0f/MAT_ELEMENT(S, i, i);
	}

	// tmp1 = U'
	arm_mat_trans_f32(&U, &tmp1);

	// U = S*tmp1
	float tmp = U.numCols;
	U.numCols = U.numRows;
	U.numRows = tmp;
	arm_mat_mult_f32(&S, &tmp1, &U);

	// tmp1 = V*U
	tmp1.numRows = U.numRows;
	tmp1.numCols = U.numCols;
	arm_mat_mult_f32(&V, &U, &tmp1);

	if(transposed)
		arm_mat_trans_f32(&tmp1, B);
	else
		memcpy(B->pData, tmp1.pData, sizeof(float)*A->numRows*A->numCols);

	return ARM_MATH_SUCCESS;
}

/**
 * Singular Value Decomposition
 * A = U*S*V'
 *
 * A (m x n)
 * U (m x n)
 * S (n x n)
 * V (n x n)
 *
 * Note: m >= n must be true, if not, transpose matrix first
 */
arm_status arm_mat_svd(arm_matrix_instance_f32* U, arm_matrix_instance_f32* S, arm_matrix_instance_f32* V)
{
    int16_t flag, i, its, j, jj, k;
    float c, f, h, s, x, y, z;
    float anorm = 0.0, g = 0.0, scale = 0.0;
    float rv1[SVD_MAX_SIZE];
    int16_t l = 0;
    int16_t nm = 0;
    uint16_t m = U->numRows;
    uint16_t n = U->numCols;

    if (m < n || n > SVD_MAX_SIZE)
        return ARM_MATH_SIZE_MISMATCH;

/* Householder reduction to bidiagonal form */
    for (i = 0; i < n; i++)
    {
        /* left-hand reduction */
        l = i + 1;
        rv1[i] = scale * g;
        g = s = scale = 0.0;
        if (i < m)
        {
            for (k = i; k < m; k++)
                scale += fabsf(MAT_ELEMENT(*U,k,i));
            if (scale)
            {
                for (k = i; k < m; k++)
                {
                    MAT_ELEMENT(*U,k,i) = (MAT_ELEMENT(*U,k,i)/scale);
                    s += (MAT_ELEMENT(*U,k,i) * MAT_ELEMENT(*U,k,i));
                }
                f = MAT_ELEMENT(*U,i,i);
                g = -SIGNF(sqrtf(s), f);
                h = f * g - s;
                MAT_ELEMENT(*U,i,i) = (f - g);
                if (i != n - 1)
                {
                    for (j = l; j < n; j++)
                    {
                        for (s = 0.0, k = i; k < m; k++)
                            s += (MAT_ELEMENT(*U,k,i) * MAT_ELEMENT(*U,k,j));
                        f = s / h;
                        for (k = i; k < m; k++)
                            MAT_ELEMENT(*U,k,j) += (f * MAT_ELEMENT(*U,k,i));
                    }
                }
                for (k = i; k < m; k++)
                    MAT_ELEMENT(*U,k,i) = (MAT_ELEMENT(*U,k,i)*scale);
            }
        }
        MAT_ELEMENT(*S,i,i) = (scale * g);

        /* right-hand reduction */
        g = s = scale = 0.0;
        if (i < m && i != n - 1)
        {
            for (k = l; k < n; k++)
                scale += fabsf(MAT_ELEMENT(*U,i,k));
            if (scale)
            {
                for (k = l; k < n; k++)
                {
                    MAT_ELEMENT(*U,i,k) = (MAT_ELEMENT(*U,i,k)/scale);
                    s += (MAT_ELEMENT(*U,i,k) * MAT_ELEMENT(*U,i,k));
                }
                f = MAT_ELEMENT(*U,i,l);
                g = -SIGNF(sqrtf(s), f);
                h = f * g - s;
                MAT_ELEMENT(*U,i,l) = (f - g);
                for (k = l; k < n; k++)
                    rv1[k] = MAT_ELEMENT(*U,i,k) / h;
                if (i != m - 1)
                {
                    for (j = l; j < m; j++)
                    {
                        for (s = 0.0, k = l; k < n; k++)
                            s += (MAT_ELEMENT(*U,j,k) * MAT_ELEMENT(*U,i,k));
                        for (k = l; k < n; k++)
                            MAT_ELEMENT(*U,j,k) += (s * rv1[k]);
                    }
                }
                for (k = l; k < n; k++)
                    MAT_ELEMENT(*U,i,k) = (MAT_ELEMENT(*U,i,k)*scale);
            }
        }
        anorm = MAX(anorm, (fabsf(MAT_ELEMENT(*S,i,i)) + fabsf(rv1[i])));
    }

    /* accumulate the right-hand transformation */
    for (i = n - 1; i >= 0; i--)
    {
        if (i < n - 1)
        {
            if (g)
            {
                for (j = l; j < n; j++)
                    MAT_ELEMENT(*V,j,i) = ((MAT_ELEMENT(*U,i,j) / MAT_ELEMENT(*U,i,l)) / g);
                    /* float division to avoid underflow */
                for (j = l; j < n; j++)
                {
                    for (s = 0.0, k = l; k < n; k++)
                        s += (MAT_ELEMENT(*U,i,k) * MAT_ELEMENT(*V,k,j));
                    for (k = l; k < n; k++)
                        MAT_ELEMENT(*V,k,j) += (s * MAT_ELEMENT(*V,k,i));
                }
            }
            for (j = l; j < n; j++)
                MAT_ELEMENT(*V,i,j) = MAT_ELEMENT(*V,j,i) = 0.0;
        }
        MAT_ELEMENT(*V,i,i) = 1.0;
        g = rv1[i];
        l = i;
    }

    /* accumulate the left-hand transformation */
    for (i = n - 1; i >= 0; i--)
    {
        l = i + 1;
        g = MAT_ELEMENT(*S,i,i);
        if (i < n - 1)
            for (j = l; j < n; j++)
                MAT_ELEMENT(*U,i,j) = 0.0;
        if (g)
        {
            g = 1.0 / g;
            if (i != n - 1)
            {
                for (j = l; j < n; j++)
                {
                    for (s = 0.0, k = l; k < m; k++)
                        s += (MAT_ELEMENT(*U,k,i) * MAT_ELEMENT(*U,k,j));
                    f = (s / MAT_ELEMENT(*U,i,i)) * g;
                    for (k = i; k < m; k++)
                        MAT_ELEMENT(*U,k,j) += (f * MAT_ELEMENT(*U,k,i));
                }
            }
            for (j = i; j < m; j++)
                MAT_ELEMENT(*U,j,i) = (MAT_ELEMENT(*U,j,i)*g);
        }
        else
        {
            for (j = i; j < m; j++)
                MAT_ELEMENT(*U,j,i) = 0.0;
        }
        ++MAT_ELEMENT(*U,i,i);
    }

    /* diagonalize the bidiagonal form */
    for (k = n - 1; k >= 0; k--)
    {                             /* loop over singular values */
        for (its = 0; its < 30; its++)
        {                         /* loop over allowed iterations */
            flag = 1;
            for (l = k; l >= 0; l--)
            {                     /* test for splitting */
                nm = l - 1;
                if (fabsf(rv1[l]) + anorm == anorm)
                {
                    flag = 0;
                    break;
                }
                if (fabsf(MAT_ELEMENT(*S,nm,nm)) + anorm == anorm)
                    break;
            }
            if (flag)
            {
                c = 0.0;
                s = 1.0;
                for (i = l; i <= k; i++)
                {
                    f = s * rv1[i];
                    if (fabsf(f) + anorm != anorm)
                    {
                        g = MAT_ELEMENT(*S,i,i);
                        h = PYTHAGF(f, g);
                        MAT_ELEMENT(*S,i,i) = h;
                        h = 1.0 / h;
                        c = g * h;
                        s = (- f * h);
                        for (j = 0; j < m; j++)
                        {
                            y = MAT_ELEMENT(*U,j,nm);
                            z = MAT_ELEMENT(*U,j,i);
                            MAT_ELEMENT(*U,j,nm) = (y * c + z * s);
                            MAT_ELEMENT(*U,j,i) = (z * c - y * s);
                        }
                    }
                }
            }
            z = MAT_ELEMENT(*S,k,k);
            if (l == k)
            {                  /* convergence */
                if (z < 0.0)
                {              /* make singular value nonnegative */
                    MAT_ELEMENT(*S,k,k) = (-z);
                    for (j = 0; j < n; j++)
                        MAT_ELEMENT(*V,j,k) = (-MAT_ELEMENT(*V,j,k));
                }
                break;
            }

            if (its >= 30)
                return ARM_MATH_ARGUMENT_ERROR; // No convergence after 30,000! iterations

            /* shift from bottom 2 x 2 minor */
            x = MAT_ELEMENT(*S,l,l);
            nm = k - 1;
            y = MAT_ELEMENT(*S,nm,nm);
            g = rv1[nm];
            h = rv1[k];
            f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
            g = PYTHAGF(f, 1.0);
            f = ((x - z) * (x + z) + h * ((y / (f + SIGNF(g, f))) - h)) / x;

            /* next QR transformation */
            c = s = 1.0;
            for (j = l; j <= nm; j++)
            {
                i = j + 1;
                g = rv1[i];
                y = MAT_ELEMENT(*S,i,i);
                h = s * g;
                g = c * g;
                z = PYTHAGF(f, h);
                rv1[j] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y = y * c;
                for (jj = 0; jj < n; jj++)
                {
                    x = MAT_ELEMENT(*V,jj,j);
                    z = MAT_ELEMENT(*V,jj,i);
                    MAT_ELEMENT(*V,jj,j) = (x * c + z * s);
                    MAT_ELEMENT(*V,jj,i) = (z * c - x * s);
                }
                z = PYTHAGF(f, h);
                MAT_ELEMENT(*S,j,j) = z;
                if (z)
                {
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                }
                f = (c * g) + (s * y);
                x = (c * y) - (s * g);
                for (jj = 0; jj < m; jj++)
                {
                    y = MAT_ELEMENT(*U,jj,j);
                    z = MAT_ELEMENT(*U,jj,i);
                    MAT_ELEMENT(*U,jj,j) = (y * c + z * s);
                    MAT_ELEMENT(*U,jj,i) = (z * c - y * s);
                }
            }
            rv1[l] = 0.0;
            rv1[k] = f;
            MAT_ELEMENT(*S,k,k) = x;
        }
    }

    return ARM_MATH_SUCCESS;
}

/**
 * Cholesky Decomposition
 * A = L*L^T
 *
 * L is lower left triangular matrix of the decomposition (cholesky factor)
 */
arm_status arm_mat_chol(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* L)
{
	if(A->numCols != A->numRows || L->numCols != L->numRows || A->numCols != L->numCols)
		return ARM_MATH_SIZE_MISMATCH;

	uint16_t n = A->numCols;

	for(uint16_t i = 0; i < n; i++)
	{
		for(uint16_t j = 0; j < (i + 1); j++)
		{
			float s = 0;
			for(uint16_t k = 0; k < j; k++)
				s += L->pData[i * n + k] * L->pData[j * n + k];

			L->pData[i * n + j] = (i == j) ? sqrtf(A->pData[i * n + i] - s) : (1.0f / L->pData[j * n + j] * (A->pData[i * n + j] - s));
		}
	}

	return ARM_MATH_SUCCESS;
}

static void arm_mat_tri_inv(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* B)
{
	uint16_t n = A->numCols;

	arm_mat_zero_f32(B);

	// B = iL, A = L
	for(uint16_t j = 0; j < n; j++)
	{
		MAT_ELEMENT(*B,j,j) = 1.0f/MAT_ELEMENT(*A,j,j);

		for(uint16_t i = 0; i < j; i++)
		{
			for(uint16_t k = 0; k < j; k++)
				MAT_ELEMENT(*B,j,i) += MAT_ELEMENT(*B,k,i)*MAT_ELEMENT(*A,j,k);
		}

		for(uint16_t k = 0; k < j; k++)
			MAT_ELEMENT(*B,j,k) /= -MAT_ELEMENT(*A,j,j);
	}
}

/**
 * Fast inverse for positive definite and symmetric matrices
 * using cholesky decomposition.
 *
 * Warning: this function uses A for temporary storage and thereby destroys its content
 */
arm_status arm_mat_pds_inv(arm_matrix_instance_f32* A, arm_matrix_instance_f32* B)
{
	arm_status status = arm_mat_chol(A, B);
	if(status != ARM_MATH_SUCCESS)
		return status;

	arm_mat_tri_inv(B, A);

	uint16_t n = A->numCols;

	for(uint16_t c = 0; c < n; c++)
	{
		for(uint16_t r = 0; r < n; r++)
		{
			float sum = 0.0f;

			for(uint16_t i = r; i < n; i++)
				sum += MAT_ELEMENT(*A,i,c)*MAT_ELEMENT(*A,i,r);

			MAT_ELEMENT(*B,r,c) = sum;
		}
	}

	return ARM_MATH_SUCCESS;
}

static float PYTHAGF(float a, float b)
{
	float at = fabsf(a), bt = fabsf(b), ct, result;

    if (at > bt)       { ct = bt / at; result = at * sqrtf(1.0f + ct * ct); }
    else if (bt > 0.0f) { ct = at / bt; result = bt * sqrtf(1.0f + ct * ct); }
    else result = 0.0f;
    return(result);
}

static inline float signf(float x)
{
	return x < 0.0f ? -1.0f : 1.0f;
}

void arm_mat_qr(const arm_matrix_instance_f32* pA, arm_matrix_instance_f32* pQ, arm_matrix_instance_f32* pR,
		arm_matrix_instance_f32* pH, arm_matrix_instance_f32* pTmp)
{
	uint16_t m = pA->numRows;
	uint16_t n = pA->numCols;

	pH->numRows = m;
	pH->numCols = m;

	arm_mat_copy_f32(pA, pR);

	for(uint16_t c = 0; c < n; c++)
	{
		pTmp->numRows = m;
		pTmp->numCols = n;
		arm_mat_copy_f32(pR, pTmp);

		// update pTmp
		float xNorm = 0;
		for(uint16_t i = c; i < m; i++)
			xNorm += MAT_ELEMENT(*pTmp, i, c)*MAT_ELEMENT(*pTmp, i, c);
		xNorm = sqrtf(xNorm);

		float x1Sign = signf(MAT_ELEMENT(*pTmp, c, c));

		MAT_ELEMENT(*pTmp, c, c) -= x1Sign*xNorm;

		float uNorm = 0;
		for(uint16_t i = c; i < m; i++)
			uNorm += MAT_ELEMENT(*pTmp, i, c)*MAT_ELEMENT(*pTmp, i, c);
		uNorm = sqrtf(uNorm);

		if(fabsf(uNorm) < 0.000001f)
		{
			for(uint16_t i = c; i < m; i++)
				MAT_ELEMENT(*pTmp, i, c) = 0;
		}
		else
		{
			for(uint16_t i = c; i < m; i++)
				MAT_ELEMENT(*pTmp, i, c) /= uNorm;
		}

		// construct householder reflection
		arm_mat_identity_f32(pH);
		for(uint16_t i = c; i < m; i++)
		{
			for(uint16_t j = c; j < m; j++)
			{
				MAT_ELEMENT(*pH, i, j) -= 2.0f*MAT_ELEMENT(*pTmp, i, c)*MAT_ELEMENT(*pTmp, j, c);
			}
		}

		// apply H to tmp
		for(uint16_t i = c; i < m; i++)
		{
			for(uint16_t j = c+1; j < n; j++)
			{
				MAT_ELEMENT(*pTmp, i, j) = 0;
				for(uint16_t k = c; k < m; k++)
				{
					MAT_ELEMENT(*pTmp, i, j) += MAT_ELEMENT(*pH, i, k)*MAT_ELEMENT(*pR, k, j);
				}
			}
		}

		MAT_ELEMENT(*pTmp, c, c) = x1Sign*xNorm;
		for(uint16_t r = c+1; r < m; r++)
			MAT_ELEMENT(*pTmp, r, c) = 0;

		arm_mat_copy_f32(pTmp, pR);

		// construct Q
		if(pQ)
		{
			if(c == 0)
			{
				arm_mat_trans_f32(pH, pQ);
			}
			else
			{
				pTmp->numRows = m;
				pTmp->numCols = m;
				arm_mat_copy_f32(pQ, pTmp);

				for(uint16_t i = 0; i < m; i++)
				{
					for(uint16_t j = c; j < m; j++)
					{
						MAT_ELEMENT(*pTmp, i, j) = 0;
						for(uint16_t k = c; k < m; k++)
							MAT_ELEMENT(*pTmp, i, j) += MAT_ELEMENT(*pQ, i, k)*MAT_ELEMENT(*pH, k, j);
					}
				}

				arm_mat_copy_f32(pTmp, pQ);
			}
		}
	}
}

void arm_mat_cholupdate(arm_matrix_instance_f32* pL, float* pX)
{
	uint16_t n = pL->numRows;

	for(uint16_t k = 0; k < n; k++)
	{
		float l = MAT_ELEMENT(*pL, k, k);
		float x = pX[k];
		float r = -sqrtf(l*l + x*x);
		float c = r/l;
		float s = x/l;
		if(fabsf(l) < 0.000001f)
		{
			c = 0;
			s = 0;
		}

		MAT_ELEMENT(*pL, k, k) = r;

		for(uint16_t i = k+1; i < n; i++)
		{
			MAT_ELEMENT(*pL, k, i) = (MAT_ELEMENT(*pL, k, i)+s*pX[i]);
			if(fabsf(c) < 0.000001f)
				MAT_ELEMENT(*pL, k, i) = 0;
			else
				MAT_ELEMENT(*pL, k, i) /= c;
			pX[i] = c*pX[i] - s*MAT_ELEMENT(*pL, k, i);
		}
	}
}

void arm_mat_choldowndate(arm_matrix_instance_f32* pL, float* pX)
{
	uint16_t n = pL->numRows;

	for(uint16_t k = 0; k < n; k++)
	{
		float l = MAT_ELEMENT(*pL, k, k);
		float x = pX[k];
		float r = l*l - x*x;
		if(r < 0)
			r = 0;
		else
			r = -sqrtf(r);
		float c = r/l;
		float s = x/l;
		if(fabsf(l) < 0.000001f)
		{
			c = 0;
			s = 0;
		}

		MAT_ELEMENT(*pL, k, k) = r;

		for(uint16_t i = k+1; i < n; i++)
		{
			MAT_ELEMENT(*pL, k, i) = (MAT_ELEMENT(*pL, k, i)-s*pX[i]);
			if(fabsf(c) < 0.000001f)
				MAT_ELEMENT(*pL, k, i) = 0;
			else
				MAT_ELEMENT(*pL, k, i) /= c;
			pX[i] = c*pX[i] - s*MAT_ELEMENT(*pL, k, i);
		}
	}
}

void arm_mat_tri_back_substitution(const arm_matrix_instance_f32* pU, const arm_matrix_instance_f32* pB, arm_matrix_instance_f32* pX)
{
	uint16_t m = pB->numRows;
	uint16_t n = pB->numCols;

	for(int16_t r = 0; r < m; r++)
	{
		for(int16_t c = 0; c < n; c++)
		{
			MAT_ELEMENT(*pX, r, c) = MAT_ELEMENT(*pB, r, c);

			for(int16_t i = 0; i < c-1; i++)
				MAT_ELEMENT(*pX, r, c) -= MAT_ELEMENT(*pX, r, i)*MAT_ELEMENT(*pU, i, c);

			if(MAT_ELEMENT(*pU, c, c) != 0.0f)
				MAT_ELEMENT(*pX, r, c) /= MAT_ELEMENT(*pU, c, c);
		}
	}
}

void arm_mat_tri_forward_substitution(const arm_matrix_instance_f32* pL, const arm_matrix_instance_f32* pB, arm_matrix_instance_f32* pX)
{
	uint16_t m = pB->numRows;
	uint16_t n = pB->numCols;

	for(int16_t r = 0; r < m; r++)
	{
		for(int16_t c = n-1; c >= 0; c--)
		{
			MAT_ELEMENT(*pX, r, c) = MAT_ELEMENT(*pB, r, c);

			for(int16_t i = c+1; i < n; i++)
				MAT_ELEMENT(*pX, r, c) -= MAT_ELEMENT(*pX, r, i)*MAT_ELEMENT(*pL, i, c);

			if(MAT_ELEMENT(*pL, c, c) != 0.0f)
				MAT_ELEMENT(*pX, r, c) /= MAT_ELEMENT(*pL, c, c);
		}
	}
}
