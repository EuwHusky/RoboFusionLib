#include "algo_matrix.h"

void rflMatrixInit(rfl_matrix_instance *S, uint16_t nRows, uint16_t nColumns, float *pData)
{
    /* 分配行数 */
    S->numRows = nRows;
    /* 分配列数 */
    S->numCols = nColumns;
    /* 分配数据指针 */
    S->pData = pData;
}

rfl_matrix_status rflMatrixAdd(const rfl_matrix_instance *pSrcA, const rfl_matrix_instance *pSrcB,
                               rfl_matrix_instance *pDst)
{
    float *pIn1 = pSrcA->pData; /* input data matrix pointer A  */
    float *pIn2 = pSrcB->pData; /* input data matrix pointer B  */
    float *pOut = pDst->pData;  /* output data matrix pointer   */

    float inA1, inA2, inB1, inB2, out1, out2; /* temporary variables */

    uint32_t numSamples;      /* total number of elements in the matrix  */
    uint32_t blkCnt;          /* loop counters */
    rfl_matrix_status status; /* status of matrix addition */

    {

        /* Total number of samples in the input matrix */
        numSamples = (uint32_t)pSrcA->numRows * pSrcA->numCols;

        /* Loop unrolling */
        blkCnt = numSamples >> 2u;

        /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
         ** a second loop below computes the remaining 1 to 3 samples. */
        while (blkCnt > 0u)
        {
            /* C(m,n) = A(m,n) + B(m,n) */
            /* Add and then store the results in the destination buffer. */
            /* Read values from source A */
            inA1 = pIn1[0];

            /* Read values from source B */
            inB1 = pIn2[0];

            /* Read values from source A */
            inA2 = pIn1[1];

            /* out = sourceA + sourceB */
            out1 = inA1 + inB1;

            /* Read values from source B */
            inB2 = pIn2[1];

            /* Read values from source A */
            inA1 = pIn1[2];

            /* out = sourceA + sourceB */
            out2 = inA2 + inB2;

            /* Read values from source B */
            inB1 = pIn2[2];

            /* Store result in destination */
            pOut[0] = out1;
            pOut[1] = out2;

            /* Read values from source A */
            inA2 = pIn1[3];

            /* Read values from source B */
            inB2 = pIn2[3];

            /* out = sourceA + sourceB */
            out1 = inA1 + inB1;

            /* out = sourceA + sourceB */
            out2 = inA2 + inB2;

            /* Store result in destination */
            pOut[2] = out1;

            /* Store result in destination */
            pOut[3] = out2;

            /* update pointers to process next sampels */
            pIn1 += 4u;
            pIn2 += 4u;
            pOut += 4u;
            /* Decrement the loop counter */
            blkCnt--;
        }

        /* If the numSamples is not a multiple of 4, compute any remaining output samples here.
         ** No loop unrolling is used. */
        blkCnt = numSamples % 0x4u;

        while (blkCnt > 0u)
        {
            /* C(m,n) = A(m,n) + B(m,n) */
            /* Add and then store the results in the destination buffer. */
            *pOut++ = (*pIn1++) + (*pIn2++);

            /* Decrement the loop counter */
            blkCnt--;
        }

        /* set status as RFL_MATRIX_SUCCESS */
        status = RFL_MATRIX_SUCCESS;
    }

    /* Return to application */
    return (status);
}

rfl_matrix_status rflMatrixSub(const rfl_matrix_instance *pSrcA, const rfl_matrix_instance *pSrcB,
                               rfl_matrix_instance *pDst)
{
    float *pIn1 = pSrcA->pData; /* input data matrix pointer A */
    float *pIn2 = pSrcB->pData; /* input data matrix pointer B */
    float *pOut = pDst->pData;  /* output data matrix pointer  */

    float inA1, inA2, inB1, inB2, out1, out2; /* temporary variables */

    uint32_t numSamples;      /* total number of elements in the matrix  */
    uint32_t blkCnt;          /* loop counters */
    rfl_matrix_status status; /* status of matrix subtraction */

    {
        /* Total number of samples in the input matrix */
        numSamples = (uint32_t)pSrcA->numRows * pSrcA->numCols;

        /* Run the below code for Cortex-M4 and Cortex-M3 */

        /* Loop Unrolling */
        blkCnt = numSamples >> 2u;

        /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
         ** a second loop below computes the remaining 1 to 3 samples. */
        while (blkCnt > 0u)
        {
            /* C(m,n) = A(m,n) - B(m,n) */
            /* Subtract and then store the results in the destination buffer. */
            /* Read values from source A */
            inA1 = pIn1[0];

            /* Read values from source B */
            inB1 = pIn2[0];

            /* Read values from source A */
            inA2 = pIn1[1];

            /* out = sourceA - sourceB */
            out1 = inA1 - inB1;

            /* Read values from source B */
            inB2 = pIn2[1];

            /* Read values from source A */
            inA1 = pIn1[2];

            /* out = sourceA - sourceB */
            out2 = inA2 - inB2;

            /* Read values from source B */
            inB1 = pIn2[2];

            /* Store result in destination */
            pOut[0] = out1;
            pOut[1] = out2;

            /* Read values from source A */
            inA2 = pIn1[3];

            /* Read values from source B */
            inB2 = pIn2[3];

            /* out = sourceA - sourceB */
            out1 = inA1 - inB1;

            /* out = sourceA - sourceB */
            out2 = inA2 - inB2;

            /* Store result in destination */
            pOut[2] = out1;

            /* Store result in destination */
            pOut[3] = out2;

            /* update pointers to process next sampels */
            pIn1 += 4u;
            pIn2 += 4u;
            pOut += 4u;

            /* Decrement the loop counter */
            blkCnt--;
        }

        /* If the numSamples is not a multiple of 4, compute any remaining output samples here.
         ** No loop unrolling is used. */
        blkCnt = numSamples % 0x4u;

        while (blkCnt > 0u)
        {
            /* C(m,n) = A(m,n) - B(m,n) */
            /* Subtract and then store the results in the destination buffer. */
            *pOut++ = (*pIn1++) - (*pIn2++);

            /* Decrement the loop counter */
            blkCnt--;
        }

        /* Set status as RFL_MATRIX_SUCCESS */
        status = RFL_MATRIX_SUCCESS;
    }

    /* Return to application */
    return (status);
}

rfl_matrix_status rflMatrixMult(const rfl_matrix_instance *pSrcA, const rfl_matrix_instance *pSrcB,
                                rfl_matrix_instance *pDst)
{
    float *pIn1 = pSrcA->pData;         /* input data matrix pointer A */
    float *pIn2 = pSrcB->pData;         /* input data matrix pointer B */
    float *pInA = pSrcA->pData;         /* input data matrix pointer A  */
    float *pOut = pDst->pData;          /* output data matrix pointer */
    float *px;                          /* Temporary output data matrix pointer */
    float sum;                          /* Accumulator */
    uint16_t numRowsA = pSrcA->numRows; /* number of rows of input matrix A */
    uint16_t numColsB = pSrcB->numCols; /* number of columns of input matrix B */
    uint16_t numColsA = pSrcA->numCols; /* number of columns of input matrix A */

    /* Run the below code for Cortex-M4 and Cortex-M3 */

    float in1, in2, in3, in4;
    uint16_t col, i = 0u, j, row = numRowsA, colCnt; /* loop counters */
    rfl_matrix_status status;                        /* status of matrix multiplication */

    {
        /* The following loop performs the dot-product of each row in pSrcA with each column in pSrcB */
        /* row loop */
        do
        {
            /* Output pointer is set to starting address of the row being processed */
            px = pOut + i;

            /* For every row wise process, the column loop counter is to be initiated */
            col = numColsB;

            /* For every row wise process, the pIn2 pointer is set
             ** to the starting address of the pSrcB data */
            pIn2 = pSrcB->pData;

            j = 0u;

            /* column loop */
            do
            {
                /* Set the variable sum, that acts as accumulator, to zero */
                sum = 0.0f;

                /* Initiate the pointer pIn1 to point to the starting address of the column being processed */
                pIn1 = pInA;

                /* Apply loop unrolling and compute 4 MACs simultaneously. */
                colCnt = numColsA >> 2u;

                /* matrix multiplication        */
                while (colCnt > 0u)
                {
                    /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
                    in3 = *pIn2;
                    pIn2 += numColsB;
                    in1 = pIn1[0];
                    in2 = pIn1[1];
                    sum += in1 * in3;
                    in4 = *pIn2;
                    pIn2 += numColsB;
                    sum += in2 * in4;

                    in3 = *pIn2;
                    pIn2 += numColsB;
                    in1 = pIn1[2];
                    in2 = pIn1[3];
                    sum += in1 * in3;
                    in4 = *pIn2;
                    pIn2 += numColsB;
                    sum += in2 * in4;
                    pIn1 += 4u;

                    /* Decrement the loop count */
                    colCnt--;
                }

                /* If the columns of pSrcA is not a multiple of 4, compute any remaining MACs here.
                 ** No loop unrolling is used. */
                colCnt = numColsA % 0x4u;

                while (colCnt > 0u)
                {
                    /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
                    sum += *pIn1++ * (*pIn2);
                    pIn2 += numColsB;

                    /* Decrement the loop counter */
                    colCnt--;
                }

                /* Store the result in the destination buffer */
                *px++ = sum;

                /* Update the pointer pIn2 to point to the  starting address of the next column */
                j++;
                pIn2 = pSrcB->pData + j;

                /* Decrement the column loop counter */
                col--;

            } while (col > 0u);

            /* Update the pointer pInA to point to the  starting address of the next row */
            i = i + numColsB;
            pInA = pInA + numColsA;

            /* Decrement the row loop counter */
            row--;

        } while (row > 0u);
        /* Set status as RFL_MATRIX_SUCCESS */
        status = RFL_MATRIX_SUCCESS;
    }

    /* Return to application */
    return (status);
}

rfl_matrix_status rflMatrixTrans(const rfl_matrix_instance *pSrc, rfl_matrix_instance *pDst)
{
    float *pIn = pSrc->pData;          /* input data matrix pointer */
    float *pOut = pDst->pData;         /* output data matrix pointer */
    float *px;                         /* Temporary output data matrix pointer */
    uint16_t nRows = pSrc->numRows;    /* number of rows */
    uint16_t nColumns = pSrc->numCols; /* number of columns */

    /* Run the below code for Cortex-M4 and Cortex-M3 */

    uint16_t blkCnt, i = 0u, row = nRows; /* loop counters */
    rfl_matrix_status status;             /* status of matrix transpose  */

    {
        /* Matrix transpose by exchanging the rows with columns */
        /* row loop     */
        do
        {
            /* Loop Unrolling */
            blkCnt = nColumns >> 2;

            /* The pointer px is set to starting address of the column being processed */
            px = pOut + i;

            /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
             ** a second loop below computes the remaining 1 to 3 samples. */
            while (blkCnt > 0u) /* column loop */
            {
                /* Read and store the input element in the destination */
                *px = *pIn++;

                /* Update the pointer px to point to the next row of the transposed matrix */
                px += nRows;

                /* Read and store the input element in the destination */
                *px = *pIn++;

                /* Update the pointer px to point to the next row of the transposed matrix */
                px += nRows;

                /* Read and store the input element in the destination */
                *px = *pIn++;

                /* Update the pointer px to point to the next row of the transposed matrix */
                px += nRows;

                /* Read and store the input element in the destination */
                *px = *pIn++;

                /* Update the pointer px to point to the next row of the transposed matrix */
                px += nRows;

                /* Decrement the column loop counter */
                blkCnt--;
            }

            /* Perform matrix transpose for last 3 samples here. */
            blkCnt = nColumns % 0x4u;

            while (blkCnt > 0u)
            {
                /* Read and store the input element in the destination */
                *px = *pIn++;

                /* Update the pointer px to point to the next row of the transposed matrix */
                px += nRows;

                /* Decrement the column loop counter */
                blkCnt--;
            }

            i++;

            /* Decrement the row loop counter */
            row--;

        } while (row > 0u); /* row loop end  */

        /* Set status as RFL_MATRIX_SUCCESS */
        status = RFL_MATRIX_SUCCESS;
    }

    /* Return to application */
    return (status);
}

rfl_matrix_status rflMatrixInverse(const rfl_matrix_instance *pSrc, rfl_matrix_instance *pDst)
{
    float *pIn = pSrc->pData;                                /* input data matrix pointer */
    float *pOut = pDst->pData;                               /* output data matrix pointer */
    float *pInT1, *pInT2;                                    /* Temporary input data matrix pointer */
    float *pOutT1, *pOutT2;                                  /* Temporary output data matrix pointer */
    float *pPivotRowIn, *pPRT_in, *pPivotRowDst, *pPRT_pDst; /* Temporary input and output data matrix pointer */
    uint32_t numRows = pSrc->numRows;                        /* Number of rows in the matrix  */
    uint32_t numCols = pSrc->numCols;                        /* Number of Cols in the matrix  */

    float maxC; /* maximum value in the column */

    /* Run the below code for Cortex-M4 and Cortex-M3 */

    float Xchg, in = 0.0f, in1;                      /* Temporary input values  */
    uint32_t i, rowCnt, flag = 0u, j, loopCnt, k, l; /* loop counters */
    rfl_matrix_status status;                        /* status of matrix inverse */

    {

        /*--------------------------------------------------------------------------------------------------------------
         * Matrix Inverse can be solved using elementary row operations.
         *
         *	Gauss-Jordan Method:
         *
         *	   1. First combine the identity matrix and the input matrix separated by a bar to form an
         *        augmented matrix as follows:
         *				        _ 	      	       _         _	       _
         *					   |  a11  a12 | 1   0  |       |  X11 X12  |
         *					   |           |        |   =   |           |
         *					   |_ a21  a22 | 0   1 _|       |_ X21 X21 _|
         *
         *		2. In our implementation, pDst Matrix is used as identity matrix.
         *
         *		3. Begin with the first row. Let i = 1.
         *
         *	    4. Check to see if the pivot for column i is the greatest of the column.
         *		   The pivot is the element of the main diagonal that is on the current row.
         *		   For instance, if working with row i, then the pivot element is aii.
         *		   If the pivot is not the most significant of the columns, exchange that row with a row
         *		   below it that does contain the most significant value in column i. If the most
         *         significant value of the column is zero, then an inverse to that matrix does not exist.
         *		   The most significant value of the column is the absolute maximum.
         *
         *	    5. Divide every element of row i by the pivot.
         *
         *	    6. For every row below and  row i, replace that row with the sum of that row and
         *		   a multiple of row i so that each new element in column i below row i is zero.
         *
         *	    7. Move to the next row and column and repeat steps 2 through 5 until you have zeros
         *		   for every element below and above the main diagonal.
         *
         *		8. Now an identical matrix is formed to the left of the bar(input matrix, pSrc).
         *		   Therefore, the matrix to the right of the bar is our solution(pDst matrix, pDst).
         *----------------------------------------------------------------------------------------------------------------*/

        /* Working pointer for destination matrix */
        pOutT1 = pOut;

        /* Loop over the number of rows */
        rowCnt = numRows;

        /* Making the destination matrix as identity matrix */
        while (rowCnt > 0u)
        {
            /* Writing all zeroes in lower triangle of the destination matrix */
            j = numRows - rowCnt;
            while (j > 0u)
            {
                *pOutT1++ = 0.0f;
                j--;
            }

            /* Writing all ones in the diagonal of the destination matrix */
            *pOutT1++ = 1.0f;

            /* Writing all zeroes in upper triangle of the destination matrix */
            j = rowCnt - 1u;
            while (j > 0u)
            {
                *pOutT1++ = 0.0f;
                j--;
            }

            /* Decrement the loop counter */
            rowCnt--;
        }

        /* Loop over the number of columns of the input matrix.
           All the elements in each column are processed by the row operations */
        loopCnt = numCols;

        /* Index modifier to navigate through the columns */
        l = 0u;

        while (loopCnt > 0u)
        {
            /* Check if the pivot element is zero..
             * If it is zero then interchange the row with non zero row below.
             * If there is no non zero element to replace in the rows below,
             * then the matrix is Singular. */

            /* Working pointer for the input matrix that points
             * to the pivot element of the particular row  */
            pInT1 = pIn + (l * numCols);

            /* Working pointer for the destination matrix that points
             * to the pivot element of the particular row  */
            pOutT1 = pOut + (l * numCols);

            /* Temporary variable to hold the pivot value */
            in = *pInT1;

            /* Grab the most significant value from column l */
            maxC = 0;
            for (i = l; i < numRows; i++)
            {
                maxC = *pInT1 > 0 ? (*pInT1 > maxC ? *pInT1 : maxC) : (-*pInT1 > maxC ? -*pInT1 : maxC);
                pInT1 += numCols;
            }

            /* Update the status if the matrix is singular */
            if (maxC == 0.0f)
            {
                return RFL_MATRIX_SINGULAR;
            }

            /* Restore pInT1  */
            pInT1 = pIn;

            /* Destination pointer modifier */
            k = 1u;

            /* Check if the pivot element is the most significant of the column */
            if ((in > 0.0f ? in : -in) != maxC)
            {
                /* Loop over the number rows present below */
                i = numRows - (l + 1u);

                while (i > 0u)
                {
                    /* Update the input and destination pointers */
                    pInT2 = pInT1 + (numCols * l);
                    pOutT2 = pOutT1 + (numCols * k);

                    /* Look for the most significant element to
                     * replace in the rows below */
                    if ((*pInT2 > 0.0f ? *pInT2 : -*pInT2) == maxC)
                    {
                        /* Loop over number of columns
                         * to the right of the pilot element */
                        j = numCols - l;

                        while (j > 0u)
                        {
                            /* Exchange the row elements of the input matrix */
                            Xchg = *pInT2;
                            *pInT2++ = *pInT1;
                            *pInT1++ = Xchg;

                            /* Decrement the loop counter */
                            j--;
                        }

                        /* Loop over number of columns of the destination matrix */
                        j = numCols;

                        while (j > 0u)
                        {
                            /* Exchange the row elements of the destination matrix */
                            Xchg = *pOutT2;
                            *pOutT2++ = *pOutT1;
                            *pOutT1++ = Xchg;

                            /* Decrement the loop counter */
                            j--;
                        }

                        /* Flag to indicate whether exchange is done or not */
                        flag = 1u;

                        /* Break after exchange is done */
                        break;
                    }

                    /* Update the destination pointer modifier */
                    k++;

                    /* Decrement the loop counter */
                    i--;
                }
            }

            /* Update the status if the matrix is singular */
            if ((flag != 1u) && (in == 0.0f))
            {
                return RFL_MATRIX_SINGULAR;
            }

            /* Points to the pivot row of input and destination matrices */
            pPivotRowIn = pIn + (l * numCols);
            pPivotRowDst = pOut + (l * numCols);

            /* Temporary pointers to the pivot row pointers */
            pInT1 = pPivotRowIn;
            pInT2 = pPivotRowDst;

            /* Pivot element of the row */
            in = *pPivotRowIn;

            /* Loop over number of columns
             * to the right of the pilot element */
            j = (numCols - l);

            while (j > 0u)
            {
                /* Divide each element of the row of the input matrix
                 * by the pivot element */
                in1 = *pInT1;
                *pInT1++ = in1 / in;

                /* Decrement the loop counter */
                j--;
            }

            /* Loop over number of columns of the destination matrix */
            j = numCols;

            while (j > 0u)
            {
                /* Divide each element of the row of the destination matrix
                 * by the pivot element */
                in1 = *pInT2;
                *pInT2++ = in1 / in;

                /* Decrement the loop counter */
                j--;
            }

            /* Replace the rows with the sum of that row and a multiple of row i
             * so that each new element in column i above row i is zero.*/

            /* Temporary pointers for input and destination matrices */
            pInT1 = pIn;
            pInT2 = pOut;

            /* index used to check for pivot element */
            i = 0u;

            /* Loop over number of rows */
            /*  to be replaced by the sum of that row and a multiple of row i */
            k = numRows;

            while (k > 0u)
            {
                /* Check for the pivot element */
                if (i == l)
                {
                    /* If the processing element is the pivot element,
                       only the columns to the right are to be processed */
                    pInT1 += numCols - l;

                    pInT2 += numCols;
                }
                else
                {
                    /* Element of the reference row */
                    in = *pInT1;

                    /* Working pointers for input and destination pivot rows */
                    pPRT_in = pPivotRowIn;
                    pPRT_pDst = pPivotRowDst;

                    /* Loop over the number of columns to the right of the pivot element,
                       to replace the elements in the input matrix */
                    j = (numCols - l);

                    while (j > 0u)
                    {
                        /* Replace the element by the sum of that row
                           and a multiple of the reference row  */
                        in1 = *pInT1;
                        *pInT1++ = in1 - (in * *pPRT_in++);

                        /* Decrement the loop counter */
                        j--;
                    }

                    /* Loop over the number of columns to
                       replace the elements in the destination matrix */
                    j = numCols;

                    while (j > 0u)
                    {
                        /* Replace the element by the sum of that row
                           and a multiple of the reference row  */
                        in1 = *pInT2;
                        *pInT2++ = in1 - (in * *pPRT_pDst++);

                        /* Decrement the loop counter */
                        j--;
                    }
                }

                /* Increment the temporary input pointer */
                pInT1 = pInT1 + l;

                /* Decrement the loop counter */
                k--;

                /* Increment the pivot index */
                i++;
            }

            /* Increment the input pointer */
            pIn++;

            /* Decrement the loop counter */
            loopCnt--;

            /* Increment the index modifier */
            l++;
        }

        /* Set status as RFL_MATRIX_SUCCESS */
        status = RFL_MATRIX_SUCCESS;

        if ((flag != 1u) && (in == 0.0f))
        {
            pIn = pSrc->pData;
            for (i = 0; i < numRows * numCols; i++)
            {
                if (pIn[i] != 0.0f)
                    break;
            }

            if (i == numRows * numCols)
                status = RFL_MATRIX_SINGULAR;
        }
    }
    /* Return to application */
    return (status);
}
