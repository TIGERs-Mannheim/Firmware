/*
 * arm_mat_util_f32.h
 *
 *  Created on: 23.12.2014
 *      Author: AndreR
 */

#include "arm_math.h"

#define MAT_ELEMENT(mat,r,c) ((mat).pData[(mat).numCols*(r)+(c)])

void arm_mat_zero_f32(arm_matrix_instance_f32* pMat);
arm_status arm_mat_identity_f32(arm_matrix_instance_f32* pMat);
void arm_mat_copy_f32(const arm_matrix_instance_f32* pSrc, arm_matrix_instance_f32* pDst);
void arm_mat_clip_f32(arm_matrix_instance_f32* pMat, float min, float max);
void arm_mat_fill_f32(arm_matrix_instance_f32* pMat, float value);
void arm_mat_print(const arm_matrix_instance_f32* pMat);
uint8_t arm_mat_is_nan_f32(const arm_matrix_instance_f32* A);
arm_status arm_mat_pinv(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* B);
arm_status arm_mat_svd(arm_matrix_instance_f32* U, arm_matrix_instance_f32* S, arm_matrix_instance_f32* V);
arm_status arm_mat_chol(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* L);
arm_status arm_mat_pds_inv(arm_matrix_instance_f32* A, arm_matrix_instance_f32* B);
arm_status arm_mat_inv_2x2_f32(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* B);
arm_status arm_mat_inv_3x3_f32(const arm_matrix_instance_f32* A, arm_matrix_instance_f32* B);

/**
 * QR Decomposition using Householder reflections (without pivoting)
 * A (m x n): input
 * Q (m x m): Q output (can be null)
 * R (m x n): R output
 * H (m x m): householder reflection temporary storage
 * tmp (m x m): temp storage
 */
void arm_mat_qr(const arm_matrix_instance_f32* pA, arm_matrix_instance_f32* pQ, arm_matrix_instance_f32* pR,
		arm_matrix_instance_f32* pH, arm_matrix_instance_f32* pTmp);

void arm_mat_cholupdate(arm_matrix_instance_f32* pL, float* pX);
void arm_mat_choldowndate(arm_matrix_instance_f32* pL, float* pX);

/**
 * Solve X*U=B for X where U is an upper triangular matrix
 */
void arm_mat_tri_back_substitution(const arm_matrix_instance_f32* pU, const arm_matrix_instance_f32* pB, arm_matrix_instance_f32* pX);


/**
 * Solve X*L=B for X where L is a lower triangular matrix
 */
void arm_mat_tri_forward_substitution(const arm_matrix_instance_f32* pL, const arm_matrix_instance_f32* pB, arm_matrix_instance_f32* pX);

float arm_mat_abs_max_f32(arm_matrix_instance_f32* pMat);

float32_t arm_atan2_f32(float32_t y, float32_t x);

static inline float* arm_mat_row_ptr(const arm_matrix_instance_f32* pMat, uint16_t row)
{
	return pMat->pData+pMat->numCols*row;
}
