#ifndef _ALGO_MATRIX_H__
#define _ALGO_MATRIX_H__

#include "stdint.h"

/*库中某些函数返回的错误状态*/
typedef enum
{
    RFL_MATRIX_SUCCESS = 0,         /**< 无错误 */
    RFL_MATRIX_ARGUMENT_ERROR = -1, /**< 一个或多个参数不正确 */
    RFL_MATRIX_LENGTH_ERROR = -2,   /**< 数据缓冲区的长度不正确 */
    RFL_MATRIX_SIZE_MISMATCH = -3,  /**< 矩阵的大小与操作不兼容 */
    RFL_MATRIX_NANINF = -4,         /**< 生成非数字 （NaN） 或无穷大 */
    RFL_MATRIX_SINGULAR = -5,       /**< 如果输入矩阵是奇异且不能反转，则由矩阵反演生成 */
    RFL_MATRIX_TEST_FAILURE = -6    /**< 测试失败  */
} rfl_matrix_status;

/*浮点矩阵结构的实例结构*/
typedef struct
{
    uint16_t numRows; /**< 矩阵的行数 */
    uint16_t numCols; /**< 矩阵的列数 */
    float *pData;     /**< 指向矩阵的数据 */
} rfl_matrix_instance;

void rflMatrixInit(rfl_matrix_instance *S, uint16_t nRows, uint16_t nColumns, float *pData);

rfl_matrix_status rflMatrixAdd(const rfl_matrix_instance *pSrcA, const rfl_matrix_instance *pSrcB,
                               rfl_matrix_instance *pDst);

rfl_matrix_status rflMatrixSub(const rfl_matrix_instance *pSrcA, const rfl_matrix_instance *pSrcB,
                               rfl_matrix_instance *pDst);

rfl_matrix_status rflMatrixMult(const rfl_matrix_instance *pSrcA, const rfl_matrix_instance *pSrcB,
                                rfl_matrix_instance *pDst);

rfl_matrix_status rflMatrixTrans(const rfl_matrix_instance *pSrc, rfl_matrix_instance *pDst);

rfl_matrix_status rflMatrixInverse(const rfl_matrix_instance *src, rfl_matrix_instance *dst);

#endif /* _ALGO_MATRIX_H__ */
