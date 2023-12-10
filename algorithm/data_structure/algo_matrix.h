#ifndef _ALGO_MATRIX_H__
#define _ALGO_MATRIX_H__
#endif

#include "stdint.h"
#ifndef PI
#define PI 3.14159265358979f
#endif

/*库中某些函数返回的错误状态*/
typedef enum
{
    ARM_MATH_SUCCESS = 0,         /**< 无错误 */
    ARM_MATH_ARGUMENT_ERROR = -1, /**< 一个或多个参数不正确 */
    ARM_MATH_LENGTH_ERROR = -2,   /**< 数据缓冲区的长度不正确 */
    ARM_MATH_SIZE_MISMATCH = -3,  /**< 矩阵的大小与操作不兼容 */
    ARM_MATH_NANINF = -4,         /**< 生成非数字 （NaN） 或无穷大 */
    ARM_MATH_SINGULAR = -5,       /**< 如果输入矩阵是奇异且不能反转，则由矩阵反演生成 */
    ARM_MATH_TEST_FAILURE = -6    /**< 测试失败  */
} arm_status;

/*32 位浮点类型定义*/
typedef float float32_t;

/*浮点矩阵结构的实例结构*/
typedef struct
{
    uint16_t numRows; /**< 矩阵的行数 */
    uint16_t numCols; /**< 矩阵的列数 */
    float32_t *pData; /**< 指向矩阵的数据 */
} arm_matrix_instance_f32;

void arm_mat_init_f32(
    arm_matrix_instance_f32 *S,
    uint16_t nRows,
    uint16_t nColumns,
    float32_t *pData);

arm_status arm_mat_add_f32(
    const arm_matrix_instance_f32 *pSrcA,
    const arm_matrix_instance_f32 *pSrcB,
    arm_matrix_instance_f32 *pDst);

arm_status arm_mat_sub_f32(
    const arm_matrix_instance_f32 *pSrcA,
    const arm_matrix_instance_f32 *pSrcB,
    arm_matrix_instance_f32 *pDst);

arm_status arm_mat_mult_f32(
    const arm_matrix_instance_f32 *pSrcA,
    const arm_matrix_instance_f32 *pSrcB,
    arm_matrix_instance_f32 *pDst);

arm_status arm_mat_trans_f32(
    const arm_matrix_instance_f32 *pSrc,
    arm_matrix_instance_f32 *pDst);

arm_status arm_mat_inverse_f32(
    const arm_matrix_instance_f32 *src,
    arm_matrix_instance_f32 *dst);
