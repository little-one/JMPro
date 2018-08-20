/**********************************************************************
 * Software Copyright Licensing Disclaimer
 *
 * This software module was originally developed by contributors to the
 * course of the development of ISO/IEC 14496-10 for reference purposes
 * and its performance may not have been optimized.  This software
 * module is an implementation of one or more tools as specified by
 * ISO/IEC 14496-10.  ISO/IEC gives users free license to this software
 * module or modifications thereof. Those intending to use this software
 * module in products are advised that its use may infringe existing
 * patents.  ISO/IEC have no liability for use of this software module
 * or modifications thereof.  The original contributors retain full
 * rights to modify and use the code for their own purposes, and to
 * assign or donate the code to third-parties.
 *
 * This copyright notice must be included in all copies or derivative
 * works.  Copyright (c) ISO/IEC 2004.
 **********************************************************************/

/*!
 *************************************************************************************
 * \file mv-search.c
 *
 * \brief
 *    Motion Vector Search, unified for B and P Pictures
 *
 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 *      - Inge Lille-Lang�y               <inge.lille-langoy@telenor.com>
 *      - Rickard Sjoberg                 <rickard.sjoberg@era.ericsson.se>
 *      - Stephan Wenger                  <stewe@cs.tu-berlin.de>
 *      - Jani Lainema                    <jani.lainema@nokia.com>
 *      - Detlev Marpe                    <marpe@hhi.de>
 *      - Thomas Wedi                     <wedi@tnt.uni-hannover.de>
 *      - Heiko Schwarz                   <hschwarz@hhi.de>
 *
 *************************************************************************************
*/

#include "contributors.h"

#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <limits.h>

#include "global.h"
#include "image.h"
#include "mv-search.h"
#include "refbuf.h"
#include "memalloc.h"
#include "mb_access.h"
#include "fast_me.h"

#include <time.h>
#include <sys/timeb.h>

// These procedure pointers are used by motion_search() and one_eigthpel()
static pel_t  (*PelY_14)     (pel_t**, int, int, int, int);
static pel_t *(*PelYline_11) (pel_t *, int, int, int, int);

// Statistics, temporary
int     max_mvd;
int*    spiral_search_x;
int*    spiral_search_y;//��������
int*    mvbits;
int*    refbits;
int*    byte_abs;
int**** motion_cost;


void SetMotionVectorPredictor (int  pmv[2],
                               int  ***refPic,
                               int  ****tmp_mv,
                               int  ref_frame,
                               int  list,
                               int  block_x,
                               int  block_y,
                               int  blockshape_x, //ģ��
                               int  blockshape_y);

#ifdef _FAST_FULL_ME_

/*****
 *****  static variables for fast integer motion estimation
 *****
 */
static int  **search_setup_done;  //!< flag if all block SAD's have been calculated yet
static int  **search_center_x;    //!< absolute ���� search center for fast full motion search
static int  **search_center_y;    //!< absolute search center for fast full motion search
static int  **pos_00;             //!< position of (0,0) vector
//����һ��������list0/list1
static int  *****BlockSAD;        //!< SAD for all blocksize, ref. frames and motion vectors
//[list0/list1][num_references][7��ģʽ:1-16*16 2-16*8 3-8*16 4-8*8 5-8*4 6-4*8 7-4*4]
//[16��4*4��]
static int  **max_search_range;

extern ColocatedParams *Co_located;

/*!
 ***********************************************************************
 * \brief
 *    function creating arrays for fast integer motion estimation
 ***********************************************************************
 */
void
InitializeFastFullIntegerSearch ()
{
  int  i, j, k, list;
  int  search_range = input->search_range;
  int  max_pos      = (2*search_range+1) * (2*search_range+1);

  if ((BlockSAD = (int*****)malloc (2 * sizeof(int****))) == NULL)
  	/*malloc��ϵͳ�������ָ��size���ֽڵ��ڴ�ռ䡣���������� int* ���͡�*/
    no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");

  for (list=0; list<2;list++)
  {
    if ((BlockSAD[list] = (int****)malloc ((img->max_num_references+1) * sizeof(int***))) == NULL)//�����һ�����±�0����
      no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");
    for (i = 0; i <= img->max_num_references; i++)
    {
      if ((BlockSAD[list][i] = (int***)malloc (8 * sizeof(int**))) == NULL)//7��ģʽ���±�0����
        no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");
      for (j = 1; j < 8; j++)
      {
        if ((BlockSAD[list][i][j] = (int**)malloc (16 * sizeof(int*))) == NULL)//16��4x4С��
          no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");
        for (k = 0; k < 16; k++)
        {
          if ((BlockSAD[list][i][j][k] = (int*)malloc (max_pos * sizeof(int))) == NULL)//���е�����λ��
            no_mem_exit ("InitializeFastFullIntegerSearch: BlockSAD");
        }
      }
    }
  }

  if ((search_setup_done = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_setup_done");
  if ((search_center_x = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_center_x");
  if ((search_center_y = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_center_y");
  if ((pos_00 = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: pos_00");
  if ((max_search_range = (int**)malloc (2*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: max_search_range");

  for (list=0; list<2; list++)
  {
  if ((search_setup_done[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_setup_done");
  if ((search_center_x[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_center_x");
  if ((search_center_y[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: search_center_y");
  if ((pos_00[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: pos_00");
  if ((max_search_range[list] = (int*)malloc ((img->max_num_references+1)*sizeof(int)))==NULL)
    no_mem_exit ("InitializeFastFullIntegerSearch: max_search_range");
  }

  // assign max search ranges for reference frames
  //b ����full_search ��˵��
  /* full_search == 2 ������������Χ����
    full_search == 1 ������old�ο�֡,����������Ĳο�֡
    full_search == 0 ������old�ο�֡��С�ں��Ŀ�*/
  if (input->full_search == 2)
  {
    for (list=0;list<2;list++)
      for (i=0; i<=img->max_num_references; i++)  
        max_search_range[list][i] = search_range;
  }
  else
  {
    for (list=0;list<2;list++)
    {
      //b ֻ������Ĳο�֡��������Χ��search_range
      //b �������search_range��һ��
      max_search_range[list][0] = max_search_range[list][img->max_num_references] = search_range;
      for (i=1; i< img->max_num_references; i++)  max_search_range[list][i] = search_range / 2;
    }
  }

}



/*!
 ***********************************************************************
 * \brief
 *    function for deleting the arrays for fast integer motion estimation
 ***********************************************************************
 */
void
ClearFastFullIntegerSearch ()
{//�ͷ�����Ķ�̬�ڴ�
  int  i, j, k, list;

  for (list=0; list<2; list++)
  {
    for (i = 0; i <= img->max_num_references; i++)
    {
      for (j = 1; j < 8; j++)
      {
        for (k = 0; k < 16; k++)
        {
          free (BlockSAD[list][i][j][k]);
		  //free(void *p)�����������ͷų���̬������ڴ档������Ǹ�ָ�����͡�
        }
        free (BlockSAD[list][i][j]);
      }
      free (BlockSAD[list][i]);
    }
    free (BlockSAD[list]);
  }
  free (BlockSAD);

  for (list=0; list<2; list++)
  {
    free (search_setup_done[list]);
    free (search_center_x[list]);
    free (search_center_y[list]);
    free (pos_00[list]);
    free (max_search_range[list]);
  }
  free (search_setup_done);
  free (search_center_x);
  free (search_center_y);
  free (pos_00);
  free (max_search_range);

}


/*!
 ***********************************************************************
 * \brief
 *    function resetting flags for fast integer motion estimation
 *    (have to be called in start_macroblock())
 ***********************************************************************
 */
void
ResetFastFullIntegerSearch ()
{
  int i,list;

  for (list=0; list<2; list++)
    for (i = 0; i <= img->max_num_references; i++)
      search_setup_done [list][i] = 0;
}

/*!
 ***********************************************************************
 * \brief
 *    calculation of SAD for larger blocks on the basis of 4x4 blocks
 ***********************************************************************
 */
void
SetupLargerBlocks (int list, int refindex, int max_pos)
{//�ú�鷶Χ��4*4С���SAD���������ֿ�ģʽ���ֿ��SAD
#define ADD_UP_BLOCKS()   _o=*_bo; _i=*_bi; _j=*_bj; for(pos=0;pos<max_pos;pos++) _o[pos] = _i[pos] + _j[pos];
#define INCREMENT(inc)    _bo+=inc; _bi+=inc; _bj+=inc;

  int    pos, **_bo, **_bi, **_bj;
  register int *_o,   *_i,   *_j;

  //--- blocktype 6 --- 4x8
  _bo = BlockSAD[list][refindex][6];//�������4*8�ֿ�ģʽ��SAD
  _bi = BlockSAD[list][refindex][7];//������ĸ�4*4С���SAD
  _bj = _bi + 4;//��4ָ����һ��4x4С��
  /*ÿһ��ADD_UP_BLOCKS����һ��4x8С���SAD
    INCREMENT(1),ָ�����һ��4x4С��
    INCREMENT(5)��ָ��������4x4С��
  */
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(5);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS(); INCREMENT(1);
  ADD_UP_BLOCKS();

  //--- blocktype 5 ---8x4
  _bo = BlockSAD[list][refindex][5];//�������8*4�ֿ�ģʽ��SAD
  _bi = BlockSAD[list][refindex][7];
  _bj = _bi + 1;
  /*�ο�����Ĵ��룬���������⣬�����������*/
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS();

  //--- blocktype 4 ---8x8
  _bo = BlockSAD[list][refindex][4];
  _bi = BlockSAD[list][refindex][6];
  _bj = _bi + 1;
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS(); INCREMENT(6);
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS();

  //--- blocktype 3 --- 8x16 
  _bo = BlockSAD[list][refindex][3];
  _bi = BlockSAD[list][refindex][4];
  _bj = _bi + 8;
  ADD_UP_BLOCKS(); INCREMENT(2);
  ADD_UP_BLOCKS();

  //--- blocktype 2 --- 16x8
  _bo = BlockSAD[list][refindex][2];
  _bi = BlockSAD[list][refindex][4];
  _bj = _bi + 2;
  ADD_UP_BLOCKS(); INCREMENT(8);
  ADD_UP_BLOCKS();

  //--- blocktype 1 ---16x16
  _bo = BlockSAD[list][refindex][1];
  _bi = BlockSAD[list][refindex][3];
  _bj = _bi + 2;
  ADD_UP_BLOCKS();
}


/*!
 ***********************************************************************
 * \brief  ����Ԥ�����MV�������������ģ�����������Χ�ڵ����к����Ϸ�ʽ��SADֵ
 *    Setup the fast search for an macroblock
 *    �������: ���ȼ��� 16 �� 4x4 ����������Χ
 *	    ������λ�õ� SAD��Ȼ���������� 6 �ַָ�ģʽ���Ե� SAD��
 *	    ����4x8��8x4��SAD��������4x4���SAD������
 ***********************************************************************
 */
void SetupFastFullPelSearch (int ref, int list)  // <--  reference frame parameter, list0 or 1
{
  int     pmv[2];    //�洢Ԥ���˶�������λ��1/4����
  //orig_blocks���ô�һ����������16x16
  pel_t   orig_blocks[256], *orgptr=orig_blocks, *refptr; 
  int     offset_x, offset_y, x, y, range_partly_outside, ref_x, ref_y, pos, abs_x, abs_y, bindex, blky;
  int     LineSadBlk0, LineSadBlk1, LineSadBlk2, LineSadBlk3;  //�����洢һ�����ص�SAD
  
  int     max_width, max_height;
  int     img_width, img_height;
  
  StorablePicture *ref_picture;//�ο�֡ 
  pel_t   *ref_pic; //�ο�����

   //[7]Ϊ���ַֿ�ģʽ���˶�����SADֵ,ʱ���μ�7�ַֿ�ģʽ
  int**   block_sad     = BlockSAD[list][ref][7]; //ȫ�ֱ���������

  /*��õ�ǰ���ref�ο�֡�������뾶
   �����в�ͬ�����뾶,ǰ���Ѿ�˵�����ɲο�max_search_range�ĸ�ֵ
   �ں���InitializeFastFullIntegerSearch��
  */
  int     search_range  = max_search_range[list][ref];
  
  //���������Χ
  int     max_pos       = (2*search_range+1) * (2*search_range+1);

  //�ο�listX��ֵ�ĵط�,�ں���init_lists��init_mbaff_lists
  int     list_offset   = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

  /*�Ƿ������Ȩ�ص�Ԥ��-weighted prediction(WP)��
    weighted_pred_flag��weighted_bipred_idc�ֱ���config�����
    WeightedPrediction��WeightedBiprediction 
    WP����������fading sequence�ģ����ǽ��������ĳ�����Ч���ǳ���
    WPֻ��H.264��mainline��extented���У�
    ���Բο���ƪ����:
    <<WEIGHTED PREDICTION IN THE H.264/MPEG AVC VIDEO CODING STANDARD>>by:Jill M. Boyce 
    WeightedPrediction��WeightedBipredictionĬ��ֵ��Ϊ0�����Բ�ʹ��WP����
 */
  int     apply_weights = ( (active_pps->weighted_pred_flag && (img->type == P_SLICE || img->type == SP_SLICE)) ||
                            (active_pps->weighted_bipred_idc && (img->type == B_SLICE)));
  
  //��òο�֡
  ref_picture     = listX[list+list_offset][ref];
  
  if (apply_weights)
    ref_pic       = ref_picture->imgY_11_w;
  else
    ref_pic       = ref_picture->imgY_11;

  max_width     = ref_picture->size_x - 17;
  max_height    = ref_picture->size_y - 17;
  
  img_width     = ref_picture->size_x;
  img_height    = ref_picture->size_y;

  //===== get search center: predictor of 16x16 block =====
   //Ԥ��MV��1/4����Ϊ��λ
  SetMotionVectorPredictor (pmv, enc_picture->ref_idx, enc_picture->mv, ref, list, 0, 0, 16, 16);
  
  //search_center��pmv�Ĺ�ϵ
  search_center_x[list][ref] = pmv[0] / 4;
  search_center_y[list][ref] = pmv[1] / 4;//�����������

  if (!input->rdopt)
  {
    //--- correct center so that (0,0) vector is inside ---
    //����Խ��
    search_center_x[list][ref] = max(-search_range, min(search_range, search_center_x[list][ref]));
    search_center_y[list][ref] = max(-search_range, min(search_range, search_center_y[list][ref]));
  }

  //Ԥ��mv��ԭʼ����õ�search_center�ľ�������(������ͼ�����Ͻ�(0,0)λ��) 
  search_center_x[list][ref] += img->opix_x;
  search_center_y[list][ref] += img->opix_y;

  offset_x = search_center_x[list][ref];
  offset_y = search_center_y[list][ref];

  //===== copy original block for fast access =====
  //����ԭʼ��飬��ת��Ϊһά����ʽ
  for   (y = img->opix_y; y < img->opix_y+16; y++)
    for (x = img->opix_x; x < img->opix_x+16; x++)
      *orgptr++ = imgY_org [y][x];//һά�ȽϿ�


  //===== check if whole search range is inside image =====
  //����Ƿ�Խ�磬
  //����:��1���������ʱ��߾ͻ�Խ�� 
  if (offset_x >= search_range && offset_x <= max_width  - search_range &&
      offset_y >= search_range && offset_y <= max_height - search_range   )
  {
    range_partly_outside = 0; PelYline_11 = FastLine16Y_11;//δԽ��
  }
  else
  {
    range_partly_outside = 1;//Խ��
  } 

  //===== determine position of (0,0)-vector =====
  if (!input->rdopt)
  {
    ref_x = img->opix_x - offset_x;
    ref_y = img->opix_y - offset_y;//�������

    for (pos = 0; pos < max_pos; pos++)
    {
      if (ref_x == spiral_search_x[pos] &&//��������
          ref_y == spiral_search_y[pos])
      {
        pos_00[list][ref] = pos;//ȷ����ǰMV�����������е�λ��
        break;
      }
    }
  }

  //===== loop over search range (spiral search): get blockwise SAD =====
  for (pos = 0; pos < max_pos; pos++)
  {
    //abs_y,abs_xΪ�ο�����������
    abs_y = offset_y + spiral_search_y[pos];
    abs_x = offset_x + spiral_search_x[pos];

    if (range_partly_outside)
    {
      //�����Ƿ�Խ�����ú���ָ��
      if (abs_y >= 0 && abs_y <= max_height &&
          abs_x >= 0 && abs_x <= max_width    )
      {
        PelYline_11 = FastLine16Y_11;//δԽ��
      }
      else
      {
        PelYline_11 = UMVLine16Y_11; //����padding�жϣ��������ǲ��³���
      }
    }
    
	/* ����һ�������16 �� 
	   4x4 ����������Χ������λ�õ� SAD
	   ѭ��һ����һ��4x4С��
	*/
    orgptr = orig_blocks;
    bindex = 0;
    for (blky = 0; blky < 4; blky++)
    {
      //ÿһ��4x4�飬��һ��4x4��ΪBlk0,����Blk1...
      LineSadBlk0 = LineSadBlk1 = LineSadBlk2 = LineSadBlk3 = 0; //����
      for (y = 0; y < 4; y++)//ѭ��4�κ�������4��block��SAD
      {
        // ȡ�ο�������  �����1��16������ֵ��SAD���ټ�����һ��16������ֵ��SAD
        refptr = PelYline_11 (ref_pic, abs_y++, abs_x, img_height, img_width);//abs_y�Լ�
        //byte_abs������875��
        LineSadBlk0 += byte_abs [*refptr++ - *orgptr++];//����һ�����SAD
        LineSadBlk0 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk0 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk0 += byte_abs [*refptr++ - *orgptr++];//ѭ�������ĴΣ�����Block0��SAD
        LineSadBlk1 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk1 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk1 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk1 += byte_abs [*refptr++ - *orgptr++];//ѭ�������ĴΣ�����Block1��SAD
        LineSadBlk2 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk2 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk2 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk2 += byte_abs [*refptr++ - *orgptr++];//ѭ�������ĴΣ�����Block2��SAD
        LineSadBlk3 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk3 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk3 += byte_abs [*refptr++ - *orgptr++];
        LineSadBlk3 += byte_abs [*refptr++ - *orgptr++];//ѭ�������ĴΣ�����Block3��SAD
      }
      block_sad[bindex++][pos] = LineSadBlk0;//[4*4�Ӻ��λ������][��������λ������]
      block_sad[bindex++][pos] = LineSadBlk1;
      block_sad[bindex++][pos] = LineSadBlk2;
      block_sad[bindex++][pos] = LineSadBlk3;
    }//ѭ������ʱ�����16��С���SAD
  }

  //===== combine SAD's for larger block types =====
  //��������ļ������ټ�������� 6 �ַָ�ģʽ�� SAD
  SetupLargerBlocks (list, ref, max_pos);

  //===== set flag marking that search setup have been done =====
  //�ñ�־Ϊ1��ͬһ���ο�֡�Ͳ�Ҫ�ټ�����
  search_setup_done[list][ref] = 1;  
}
#endif // _FAST_FULL_ME_

/*!
 ************************************************************************
 * \brief
 *    Set motion vector predictor
 ************************************************************************
 */
 //��Ҫ�����ӿ���Χ��4X4�ڿ�A,B,C,D��״����
 //����MVԤ��ģʽѡ��,��������pmv[2]��
void SetMotionVectorPredictor (int  pmv[2],//����Ԥ���MV
                               int  ***refPic,      //��ǰ����֡ÿ��4X4�ӿ�Ĳο�֡����
                               int  ****tmp_mv,//�Ѿ������˶����Ƶ��ӿ��mv
                               int  ref_frame,    //�ο�֡������
                               int  list,//�ο�֡�б�
                               int  block_x,//4*4���ں���ڵ��������
                               int  block_y,
                               int  blockshape_x,//Ԥ���Ŀ�
                               int  blockshape_y)
{
  int mb_x                 = 4*block_x; //���MB���������(��λ:����) 
  int mb_y                 = 4*block_y;
  
  int mb_nr                = img->current_mb_nr;  //��ǰ����ַ
  int mv_a, mv_b, mv_c, pred_vec=0;
  int mvPredType, rFrameL, rFrameU, rFrameUR;
  int hv;
  //mvPredType --Ԥ��ģʽ
  PixelPos block_a, block_b, block_c, block_d;  //A,B,C,D�ӿ�����꣬�ߴ�,��4X4Ϊ��λ
  
  int SAD_a=0, SAD_b=0, SAD_c=0, SAD_d=0;
  int temp_pred_SAD[2];

  if (input->FMEnable) pred_SAD_space=0; 

  /*
  (-1,0)������߽��ں��--block_a,
  (0,-1)�����ϱ߽��ں��--block_b,
  (blockshape_x,-1)�������Ͻǽ��ں��--block_c,
  (-1,-1)�������Ͻǽ��ں��--block_d
  �ο�H.264��԰������: MVԤ��������
 */
  //�õ�4x4С��block_a�ľ�������ͺ��������
  getLuma4x4Neighbour(mb_nr, block_x, block_y,           -1,  0, &block_a);
  //�õ�4x4С��block_b�ľ�������ͺ��������
  getLuma4x4Neighbour(mb_nr, block_x, block_y,            0, -1, &block_b);
  //�õ�4x4С��block_c�ľ�������ͺ��������
  getLuma4x4Neighbour(mb_nr, block_x, block_y, blockshape_x, -1, &block_c);
  //�õ�4x4С��block_d�ľ�������ͺ��������
  getLuma4x4Neighbour(mb_nr, block_x, block_y,           -1, -1, &block_d);

  /*������δ����������ȡ��С��16x16�ķֿ������block_c�Ŀ���������
   block_cΪʲôҪר���ó����ж��أ�����ͼ����һĿ��Ȼ��:
   �򵥷�����block_a��block_b��block_dΪʲôû��������Űɣ�
   ��Ϊ�����Ƕ��ķֿ飬�ڽ���ME֮ǰ������block_a��block_b��block_d����Ȼ�Ѿ�
   ����ME�ˣ���block_c��һ����������������: ��ǰMB��Ϊ����16x8�������Ǹ�16x8��block_c������?
   ��Ȼû�У���Ϊ����block_c���ұߺ����ڲ�����û��ME�أ��������������δ���
  */
  if (mb_y > 0)
  {
    if (mb_x < 8)  // first column of 8x8 blocks
    {
      if (mb_y==8)
      {
        if (blockshape_x == 16)      block_c.available  = 0;
        else                         block_c.available &= 1;
      }
      else
      {
        if (mb_x+blockshape_x != 8)  block_c.available &= 1;
        else                         block_c.available  = 0;
      }
    }
    else
    {
      if (mb_x+blockshape_x != 16)   block_c.available &= 1;
      else                           block_c.available  = 0;
    }
  }

  //block_c�����þ���block_d���棬block_d��Ȼ��Զ����
  //���ǣ���׼������������? Ҳ��ֻ�Ǵ���ʵ�ֵ�Ȩ��֮�ư�
  if (!block_c.available)
  {
    block_c=block_d;
  }

  mvPredType = MVPRED_MEDIAN;

  if (!img->MbaffFrameFlag)//����MBAFF
  { /*refPic�ǲο�֡��4x4С���������
    �����ڿ鲻����ʱ������-1  ������Χ���ο�֡������
  */
    rFrameL    = block_a.available    ? refPic[list][block_a.pos_x][block_a.pos_y] : -1;
    rFrameU    = block_b.available    ? refPic[list][block_b.pos_x][block_b.pos_y] : -1;
    rFrameUR   = block_c.available    ? refPic[list][block_c.pos_x][block_c.pos_y] : -1;
  }
  else  //�����MBAFF
  {
    if (img->mb_data[img->current_mb_nr].mb_field)//����ǰ���ǳ����(�����е�һ��)
    {
      rFrameL    = block_a.available    ? 
        img->mb_data[block_a.mb_addr].mb_field ?      //������ڿ�Ҳ��field MB��
        refPic[list][block_a.pos_x][block_a.pos_y]:        //��:���زο�֡������
        refPic[list][block_a.pos_x][block_a.pos_y] * 2:  //��:���������Ĳο�֡������
        -1;
      rFrameU    = block_b.available    ? 
        img->mb_data[block_b.mb_addr].mb_field ? 
        refPic[list][block_b.pos_x][block_b.pos_y]:
        refPic[list][block_b.pos_x][block_b.pos_y] * 2: 
        -1;
      rFrameUR    = block_c.available    ? 
        img->mb_data[block_c.mb_addr].mb_field ? 
        refPic[list][block_c.pos_x][block_c.pos_y]:
        refPic[list][block_c.pos_x][block_c.pos_y] * 2: 
        -1;
    }
    else  //����ǰ����֡���
    {
      rFrameL    = block_a.available    ? 
        img->mb_data[block_a.mb_addr].mb_field ?       //������ڿ���field MB��
        refPic[list][block_a.pos_x][block_a.pos_y] >>1:  //��:���زο�֡�����ŵ�һ��
        refPic[list][block_a.pos_x][block_a.pos_y] :         //��:���زο�֡������
        -1;
      rFrameU    = block_b.available    ? 
        img->mb_data[block_b.mb_addr].mb_field ? 
        refPic[list][block_b.pos_x][block_b.pos_y] >>1:
        refPic[list][block_b.pos_x][block_b.pos_y] : 
        -1;
      rFrameUR    = block_c.available    ? 
        img->mb_data[block_c.mb_addr].mb_field ? 
        refPic[list][block_c.pos_x][block_c.pos_y] >>1:
        refPic[list][block_c.pos_x][block_c.pos_y] : 
        -1;
    }
  }
  /*�ܽ�: ��ǰ������ڿ�ͬ��֡���򳡺�飬��ֻ�践�زο�֡�����ż��ɣ�
   ������һ��Ϊ֡���һ��Ϊ����飬��֡���һ���Ĳο�֡�������ǳ�����2��
 */

  /* Prediction if only one of the neighbors uses the reference frame
   * we are checking
   * ��ֻ��һ�����ڿ���ã�pred_mv������
   */
  if(rFrameL == ref_frame && rFrameU != ref_frame && rFrameUR != ref_frame)       
      mvPredType = MVPRED_L;
  else if(rFrameL != ref_frame && rFrameU == ref_frame && rFrameUR != ref_frame)  
     mvPredType = MVPRED_U;
  else if(rFrameL != ref_frame && rFrameU != ref_frame && rFrameUR == ref_frame)  
      mvPredType = MVPRED_UR;
  // Directional predictions 
  /*�����ǰ���Ϊ 8*16 �ָ�����,
   ������� 8*16 �ָ��� A �뵱ǰ�ָ�Ϊͬһ�ο�֡��
   ����� A �� MV Ϊ�÷ָ������ MV Ԥ��ֵ��
   �����ұ� 8*16 �ָ��� C �뵱ǰ�ָ�Ϊͬһ�ο�֡��
   ����� C �� MV Ϊ�÷ָ������ MV Ԥ��ֵ��
 */
  if(blockshape_x == 8 && blockshape_y == 16)
  {
    if(mb_x == 0)
    {
      if(rFrameL == ref_frame)
        mvPredType = MVPRED_L;   //A
    }
    else
    {
      if( rFrameUR == ref_frame)
        mvPredType = MVPRED_UR;  //C
    }
  }
  /*�����ǰ���Ϊ 16*8 �ָ�����,
   �����ϱ� 16*8 �ָ��� B �뵱ǰ�ָ�Ϊͬһ�ο�֡��
   ����� B �� MV Ϊ�÷ָ������ MV Ԥ��ֵ��
   �����±� 16*8 �ָ��� A �뵱ǰ�ָ�Ϊͬһ�ο�֡��
   ����� A �� MV Ϊ�÷ָ������ MV Ԥ��ֵ��
 */  
  else if(blockshape_x == 16 && blockshape_y == 8)
  {
    if(mb_y == 0)
    {
      if(rFrameU == ref_frame)
        mvPredType = MVPRED_U;  //B
    }
    else
    {
      if(rFrameL == ref_frame)
        mvPredType = MVPRED_L;  //A
    }
  }

  //��ȡblock_a,block_b,block_c��mv��tmp_mv����enc_picture->mv��
  //��enc_picture->mv�����ں���PartitionMotionSearch�еõ��ĸ����ֿ��MV
  for (hv=0; hv < 2; hv++)
  {
    if (!img->MbaffFrameFlag || hv==0) //hv=0ʱ����֡�ͳ�����Ϊ����MV��һ����
    {
      mv_a = block_a.available  ? tmp_mv[list][block_a.pos_x][block_a.pos_y][hv] : 0;
      mv_b = block_b.available  ? tmp_mv[list][block_b.pos_x][block_b.pos_y][hv] : 0;
      mv_c = block_c.available  ? tmp_mv[list][block_c.pos_x][block_c.pos_y][hv] : 0;
    }
    else
    {      
      if (img->mb_data[img->current_mb_nr].mb_field)//�����
      {
        mv_a = block_a.available  ? img->mb_data[block_a.mb_addr].mb_field?
          tmp_mv[list][block_a.pos_x][block_a.pos_y][hv]:
          tmp_mv[list][block_a.pos_x][block_a.pos_y][hv] / 2:  //����֡����MV�ǳ�����2��
          0;
        mv_b = block_b.available  ? img->mb_data[block_b.mb_addr].mb_field?
          tmp_mv[list][block_b.pos_x][block_b.pos_y][hv]:
          tmp_mv[list][block_b.pos_x][block_b.pos_y][hv] / 2: 
          0;
        mv_c = block_c.available  ? img->mb_data[block_c.mb_addr].mb_field?
          tmp_mv[list][block_c.pos_x][block_c.pos_y][hv]:
          tmp_mv[list][block_c.pos_x][block_c.pos_y][hv] / 2: 
          0;
      }
      else
      {//֡���
        mv_a = block_a.available  ? img->mb_data[block_a.mb_addr].mb_field?
          tmp_mv[list][block_a.pos_x][block_a.pos_y][hv] * 2://����֡����MV�ǳ�����2��
          tmp_mv[list][block_a.pos_x][block_a.pos_y][hv]: 
          0;
        mv_b = block_b.available  ? img->mb_data[block_b.mb_addr].mb_field?
          tmp_mv[list][block_b.pos_x][block_b.pos_y][hv] * 2:
          tmp_mv[list][block_b.pos_x][block_b.pos_y][hv]: 
          0;
        mv_c = block_c.available  ? img->mb_data[block_c.mb_addr].mb_field?
          tmp_mv[list][block_c.pos_x][block_c.pos_y][hv] * 2:
          tmp_mv[list][block_c.pos_x][block_c.pos_y][hv]: 
          0;
      }
    }

  if(input->FMEnable)//FastMEʹ��
  {//all_bwmincost �洢����Ԥ�����ݣ�  all_mincost�洢MV��SAD��Ϣ
    SAD_a = block_a.available ? ((list==1) ? all_bwmincost[block_a.pos_x][block_a.pos_y][0][FME_blocktype][0]:all_mincost[block_a.pos_x][block_a.pos_y][ref_frame][FME_blocktype][0]):0;
    SAD_b = block_b.available ? ((list==1) ? all_bwmincost[block_b.pos_x][block_b.pos_y][0][FME_blocktype][0]:all_mincost[block_b.pos_x][block_b.pos_y][ref_frame][FME_blocktype][0]):0;
    SAD_d = block_d.available ? ((list==1) ? all_bwmincost[block_d.pos_x][block_d.pos_y][0][FME_blocktype][0]:all_mincost[block_d.pos_x][block_d.pos_y][ref_frame][FME_blocktype][0]):0;
    SAD_c = block_c.available ? ((list==1) ? all_bwmincost[block_c.pos_x][block_c.pos_y][0][FME_blocktype][0]:all_mincost[block_c.pos_x][block_c.pos_y][ref_frame][FME_blocktype][0]):SAD_d;
  }

    switch (mvPredType)  //����mvPredType����pmv
    {
    case MVPRED_MEDIAN:
      if(!(block_b.available || block_c.available))//��B��C�������ã�pred_vec����mv_a
      {
        pred_vec = mv_a;
        if(input->FMEnable) temp_pred_SAD[hv] = SAD_a;
      }
      else    //����ȡ������ֵ��ע����ֵ��ƽ��ֵ������
      {
        pred_vec = mv_a+mv_b+mv_c-min(mv_a,min(mv_b,mv_c))-max(mv_a,max(mv_b,mv_c));
      }
      if(input->FMEnable)
      {
         if (pred_vec == mv_a && SAD_a != 0) temp_pred_SAD[hv] = SAD_a;
         else if (pred_vec == mv_b && SAD_b!=0) temp_pred_SAD[hv] = SAD_b;
              else temp_pred_SAD[hv] = SAD_c;
      }
      break;
    case MVPRED_L:
      pred_vec = mv_a;
      if(input->FMEnable) temp_pred_SAD[hv] = SAD_a;
      break;
    case MVPRED_U:
      pred_vec = mv_b;
      if(input->FMEnable) temp_pred_SAD[hv] = SAD_b;
      break;
    case MVPRED_UR:
      pred_vec = mv_c;
      if(input->FMEnable) temp_pred_SAD[hv] = SAD_c;
      break;
    default:
      break;
    }

    pmv[hv] = pred_vec;

  }
//ȡС��SAD
  if(input->FMEnable) pred_SAD_space = temp_pred_SAD[0]>temp_pred_SAD[1]?temp_pred_SAD[1]:temp_pred_SAD[0];
}

/*!
 ************************************************************************
 * \brief
 *    Initialize the motion search
 spiral_search_x��spiral_search_y(�������������ʼ��)��byte_abs��refbits��mvbits��ʼ��
 ��ô���ӵĳ�ʼ��ʲô����???
 ************************************************************************
 */
void
Init_Motion_Search_Module ()
{
  int bits, i, imin, imax, k, l;
//floor ��ȡ�� ceil��ȡ��  
  int search_range               = input->search_range;
  int number_of_reference_frames = img->max_num_references;
  int max_search_points          = (2*search_range+1)*(2*search_range+1);//����������
  int max_ref_bits               = 1 + 2 * (int)floor(log(max(16,number_of_reference_frames+1)) / log(2) + 1e-10);//ǰ��ͺ���
  int max_ref                    = (1<<((max_ref_bits>>1)+1))-1;//�õ����ο�֡��
  int number_of_subpel_positions = 4 * (2*search_range+3);//��������
  int max_mv_bits                = 3 + 2 * (int)ceil (log(number_of_subpel_positions+1) / log(2) + 1e-10);//x,y����������Ҫ��bit��
  max_mvd                        = (1<<( max_mv_bits >>1)   )-1;


  //=====   CREATE ARRAYS   =====
  //-----------------------------
  //calloc���ڴ�Ķ�̬�洢���з���n������Ϊsize�������ռ䣬��������һ��ָ�������ʼ��ַ��ָ��
  //��malloc������   malloc����ʼ��������������������������� calloc��ʼ��Ϊ0
  if ((spiral_search_x = (int*)calloc(max_search_points, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: spiral_search_x");
  if ((spiral_search_y = (int*)calloc(max_search_points, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: spiral_search_y");
  if ((mvbits = (int*)calloc(2*max_mvd+1, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: mvbits");
  if ((refbits = (int*)calloc(max_ref, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: refbits");
  if ((byte_abs = (int*)calloc(512, sizeof(int))) == NULL)
    no_mem_exit("Init_Motion_Search_Module: byte_abs");

  get_mem4Dint (&motion_cost, 8, 2, img->max_num_references+1, 4);

  //���þ����ʼƫ����
  mvbits   += max_mvd;//��ڵ�ַƫ��max_mvd����max_mvd ��max_mvd
  byte_abs += 256;


  //=====   INIT ARRAYS   =====
  //---------------------------
  //--- init array: motion vector bits ---
  mvbits[0] = 1;
  for (bits=3; bits<=max_mv_bits; bits+=2)
  {
    imax = 1    << (bits >> 1);
    imin = imax >> 1;

    for (i = imin; i < imax; i++)   mvbits[-i] = mvbits[i] = bits;
  }
  //--- init array: reference frame bits ---
  // ������ο�֡�������� te(v) ��ʽ���루�μ���׼��9.1 С�ڣ�����Ҫʹ�õı����������磬������
  // ���Ϊ 4������ǰ��ο�֡Ϊǰ��ο��б��еĵ� 5 ֡ʱ������ 4 ����Ҫ�ı�����Ϊ 5 ������
  refbits[0] = 1;
  for (bits=3; bits<=max_ref_bits; bits+=2)
  {
    imax = (1   << ((bits >> 1) + 1)) - 1;
    imin = imax >> 1;

    for (i = imin; i < imax; i++)   refbits[i] = bits;
  }
  //--- init array: absolute value ---
  byte_abs[0] = 0;
  for (i=1; i<256; i++)   byte_abs[i] = byte_abs[-i] = i;
  //--- init array: search pattern ---
  spiral_search_x[0] = spiral_search_y[0] = 0;
  for (k=1, l=1; l<=max(1,search_range); l++)
  {
    for (i=-l+1; i< l; i++)
    {
      spiral_search_x[k] =  i;  spiral_search_y[k++] = -l;
      spiral_search_x[k] =  i;  spiral_search_y[k++] =  l;
    }
    for (i=-l;   i<=l; i++)
    {
      spiral_search_x[k] = -l;  spiral_search_y[k++] =  i;
      spiral_search_x[k] =  l;  spiral_search_y[k++] =  i;
    }
  }

#ifdef _FAST_FULL_ME_
  if(!input->FMEnable)
    InitializeFastFullIntegerSearch ();
#endif
}


/*!
 ************************************************************************
 * \brief
 *    Free memory used by motion search
 ************************************************************************
 */
void
Clear_Motion_Search_Module ()
{
  //--- correct array offset ---
  mvbits   -= max_mvd;
  byte_abs -= 256;

  //--- delete arrays ---
  free (spiral_search_x);
  free (spiral_search_y);
  free (mvbits);
  free (refbits);
  free (byte_abs);
  free_mem4Dint (motion_cost, 8, 2);

#ifdef _FAST_FULL_ME_
  if(!input->FMEnable)
    ClearFastFullIntegerSearch ();
#endif
}



/*!
 ***********************************************************************
 * \brief������������˳�����ȫ����
 *    Full pixel block motion search
 *    Ŀ���ǵõ�(mv_x,mv_y)��min_mcost��(mv_x,mv_y)ָʾ�����￪ʼ��������������search center
 *    ��������������������������Ƚ�
 ***********************************************************************
 */
int                                               //  ==> minimum motion cost after search
FullPelBlockMotionSearch (pel_t**   orig_pic,     // <--  original pixel values for the AxB block
                          int       ref,          // <--  reference frame (0... or -1 (backward))
                          int       list,
                          int       pic_pix_x,    // <--  absolute x-coordinate of regarded AxB blockAxB���ԭ����ͼ���еľ�������
                          int       pic_pix_y,    // <--  absolute y-coordinate of regarded AxB block
                          int       blocktype,    // <--  block type (1-16x16 ... 7-4x4)
                          int       pred_mv_x,    // <--  motion vector predictor (x) in sub-pel units
                          int       pred_mv_y,    // <--  motion vector predictor (y) in sub-pel units
                          int*      mv_x,         // <--> in: search center (x) / out: motion vector (x) - in pel units
                          int*      mv_y,         // <--> in: search center (y) / out: motion vector (y) - in pel units
                          int       search_range, // <--  1-d search range in pel units
                          int       min_mcost,    // <--  minimum motion cost (cost for center or huge value)
                          double    lambda)       // <--  lagrangian parameter for determining motion cost
{
  int   pos, cand_x, cand_y, y, x4, mcost;
  
  pel_t *orig_line, *ref_line;
  pel_t *(*get_ref_line)(int, pel_t*, int, int, int, int);//
//�ο�֡ƫ���� ֡������Ӧ�Һ���ַΪż��=4 ֡������Ӧ����ַΪ����=2 ��֡������Ӧ=0
  int   list_offset   = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;
  pel_t *ref_pic			= listX[list+list_offset][ref]->imgY_11;
  int   img_width     = listX[list+list_offset][ref]->size_x;
  int   img_height    = listX[list+list_offset][ref]->size_y;

  int   best_pos      = 0;                                        // position with minimum motion cost
  //���������Ҫ������λ�ø���
  int   max_pos       = (2*search_range+1)*(2*search_range+1);    // number of search positions
  int   lambda_factor = LAMBDA_FACTOR (lambda);                   // factor for determining lagragian motion cost
  int   blocksize_y   = input->blc_size[blocktype][1];            // vertical block size
  int   blocksize_x   = input->blc_size[blocktype][0];            // horizontal block size
  int   blocksize_x4  = blocksize_x >> 2;                         // horizontal block size in 4-pel units
  int   pred_x        = (pic_pix_x << 2) + pred_mv_x;       // predicted position x (in sub-pel units)1/4������Ϊ��λ��Ԥ��MV
  int   pred_y        = (pic_pix_y << 2) + pred_mv_y;       // predicted position y (in sub-pel units)
  int   center_x      = pic_pix_x + *mv_x;                        // center position x (in pel units)
  int   center_y      = pic_pix_y + *mv_y;                        // center position y (in pel units)
  int   check_for_00  = (blocktype==1 && !input->rdopt && img->type!=B_SLICE && ref==0);

  //===== set function for getting reference picture lines =====
  //ͨ���ж�������Χ�᲻����磬���û�ȡ�ο�����ֵ�ĺ���
  if ((center_x > search_range) && (center_x < img->width -1-search_range-blocksize_x) &&
      (center_y > search_range) && (center_y < img->height-1-search_range-blocksize_y)   )
  {
     get_ref_line = FastLineX;//δ����
  }
  else
  {
     get_ref_line = UMVLineX;//����
  }


  //===== loop over all search positions =====
  //max_pos������λ�õĸ���,���������
  for (pos=0; pos<max_pos; pos++)
  {
    //--- set candidate position (absolute position in pel units) ---
    /*(center_x,center_y)����Ԥ��MV���Ƴ������������ģ�������Ϊ���ĵķ�Χ�ڣ�
    �԰���������˳�����еĺ�ѡ�����������
    ÿ����ѡ�㶼��һ�����ܲο�������Ͻ���ʼ��
  */
    cand_x = center_x + spiral_search_x[pos];//��������
    cand_y = center_y + spiral_search_y[pos];

    //--- initialize motion cost (cost for motion vector) and check ---
    //����MVD�Ĵ��ۣ�������ķ�֮һ����(cand--candidate��ѡ��)
    mcost = MV_COST (lambda_factor, 2, cand_x, cand_y, pred_x, pred_y);
    if (check_for_00 && cand_x==pic_pix_x && cand_y==pic_pix_y)
    {//�����������ĵ�ΪԤ��MVָ��ĵ�
      mcost -= WEIGHTED_COST (lambda_factor, 16);
    }
    //���ֻ��MV�Ĵ��۾��Ѿ��������е���С���۾�����
    if (mcost >= min_mcost)   continue;

    //--- add residual cost to motion cost ---
    //blocksize_y blocksize_x4 �Ƿֿ��С16x16 16x8 8x16......
    for (y=0; y<blocksize_y; y++) 
    {
      //(cand_x,cand_y+y)��һ�е���ʼ���꣬y++ ����ÿһ��
      ref_line  = get_ref_line (blocksize_x, ref_pic, cand_y+y, cand_x, img_height, img_width);
      orig_line = orig_pic [y];
      //���㵱ǰ֡�Ͳο�֡�����زв�
      for (x4=0; x4<blocksize_x4; x4++) //��4��Ϊһ�����
      {
        mcost += byte_abs[ *orig_line++ - *ref_line++ ];
        mcost += byte_abs[ *orig_line++ - *ref_line++ ];
        mcost += byte_abs[ *orig_line++ - *ref_line++ ];
        mcost += byte_abs[ *orig_line++ - *ref_line++ ];
      }

      if (mcost >= min_mcost)  //����Ѿ�����С���۴󣬾�û��Ҫ�������������
      {
        break;
      }
    }
	
    //--- check if motion cost is less than minimum cost ---
    //��¼����С���ۺ����ƥ��λ��
    if (mcost < min_mcost)
    {
      best_pos  = pos;
      min_mcost = mcost;
    }
  }


  //===== set best motion vector and return minimum motion cost =====
  if (best_pos)
  {
    *mv_x += spiral_search_x[best_pos];  //��Ϊ�������������м�¼���Ǹ�λ�õĵ�
    *mv_y += spiral_search_y[best_pos];  //��(center_x,center_y)�Ĳ�
  }
  return min_mcost;  //������С����
}


#ifdef _FAST_FULL_ME_
/*!
 ***********************************************************************
 * \brief��������������
 *    Fast Full pixel block motion search
 *    Ŀ���ǵõ�(mv_x,mv_y)��min_mcost��(mv_x,mv_y)ָʾ�����￪ʼ��������������search center
 *    ��������������������������Ƚ�
 ***********************************************************************
 */
int                                                   //  ==> minimum motion cost after search
FastFullPelBlockMotionSearch (pel_t**   orig_pic,     // <--  not used
                              int       ref,          // <--  reference frame (0... or -1 (backward))
                              int       list,
                              int       pic_pix_x,    // <--  absolute x-coordinate of regarded AxB block
                              int       pic_pix_y,    // <--  absolute y-coordinate of regarded AxB block
                              int       blocktype,    // <--  block type (1-16x16 ... 7-4x4)
                              int       pred_mv_x,    // <--  motion vector predictor (x) in sub-pel units
                              int       pred_mv_y,    // <--  motion vector predictor (y) in sub-pel units
                              int*      mv_x,         //  --> motion vector (x) - in pel units
                              int*      mv_y,         //  --> motion vector (y) - in pel units
                              int       search_range, // <--  1-d search range in pel units
                              int       min_mcost,    // <--  minimum motion cost (cost for center or huge value)
                              double    lambda)       // <--  lagrangian parameter for determining motion cost
{
  int   pos, offset_x, offset_y, cand_x, cand_y, mcost;

  int   max_pos       = (2*search_range+1)*(2*search_range+1);              // number of search positions
  int   lambda_factor = LAMBDA_FACTOR (lambda);                             // factor for determining lagragian motion cost
  int   best_pos      = 0;                                                  // position with minimum motion cost
  int   block_index;                                                        // block index for indexing SAD array
  int*  block_sad;                                                          // pointer to SAD array

  block_index   = (pic_pix_y-img->opix_y)+((pic_pix_x-img->opix_x)>>2); // block index for indexing SAD array
  block_sad     = BlockSAD[list][ref][blocktype][block_index];         // pointer to SAD array

  //===== set up fast full integer search if needed / set search center =====
  if (!search_setup_done[list][ref])//��һ���ο�ֻ֡��һ��
  {
	//����������Χ����λ�����зֿ�ģʽ��SAD(������)
    SetupFastFullPelSearch (ref, list);
  }

  offset_x = search_center_x[list][ref] - img->opix_x; //�����������ԭ����ƫ��
  offset_y = search_center_y[list][ref] - img->opix_y;

  //===== cost for (0,0)-vector: it is done before, because MVCost can be negative =====
  if (!input->rdopt)
  {
    //�Ѹղż����SAD ��mv������ӵõ��ܴ���
    mcost = block_sad[pos_00[list][ref]] + MV_COST (lambda_factor, 2, 0, 0, pred_mv_x, pred_mv_y);
    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos_00[list][ref];//ÿ֡�������ĵ�λ��
    }
  }

  //===== loop over all search positions =====
  for (pos=0; pos<max_pos; pos++, block_sad++)
  {
    //--- check residual cost ---
    if (*block_sad < min_mcost)
    {
      //--- get motion vector cost ---
      //���������λ�ã�����������˳��spiral_search_xy
      cand_x = offset_x + spiral_search_x[pos];
      cand_y = offset_y + spiral_search_y[pos];
      mcost  = *block_sad;
      mcost += MV_COST (lambda_factor, 2, cand_x, cand_y, pred_mv_x, pred_mv_y);	//���� MV ����

      //--- check motion cost ---
      if (mcost < min_mcost)
      {
        min_mcost = mcost;
        best_pos  = pos;
      }
    }
  }

  //===== set best motion vector and return minimum motion cost =====
  *mv_x = offset_x + spiral_search_x[best_pos];//���ݴ�����С����������MV
  *mv_y = offset_y + spiral_search_y[best_pos];
  return min_mcost;
}
#endif


/*!
 ***********************************************************************
 * \brief
 *    Calculate SA(T)D
 ***********************************************************************
 */
int SATD (int* diff, int use_hadamard)
{
  int k, satd = 0, m[16], dd, *d=diff;
  
  if (use_hadamard)	// ���������Hadamard�任�����ȶԲв�����Hadamard�任��Ȼ�󽫱任���16���в�ֵȡ����ֵ�����Ϊ����
  {
    /*===== hadamard transform =====*/
    m[ 0] = d[ 0] + d[12];
    m[ 4] = d[ 4] + d[ 8];
    m[ 8] = d[ 4] - d[ 8];
    m[12] = d[ 0] - d[12];
    m[ 1] = d[ 1] + d[13];
    m[ 5] = d[ 5] + d[ 9];
    m[ 9] = d[ 5] - d[ 9];
    m[13] = d[ 1] - d[13];
    m[ 2] = d[ 2] + d[14];
    m[ 6] = d[ 6] + d[10];
    m[10] = d[ 6] - d[10];
    m[14] = d[ 2] - d[14];
    m[ 3] = d[ 3] + d[15];
    m[ 7] = d[ 7] + d[11];
    m[11] = d[ 7] - d[11];
    m[15] = d[ 3] - d[15];
    
    d[ 0] = m[ 0] + m[ 4];
    d[ 8] = m[ 0] - m[ 4];
    d[ 4] = m[ 8] + m[12];
    d[12] = m[12] - m[ 8];
    d[ 1] = m[ 1] + m[ 5];
    d[ 9] = m[ 1] - m[ 5];
    d[ 5] = m[ 9] + m[13];
    d[13] = m[13] - m[ 9];
    d[ 2] = m[ 2] + m[ 6];
    d[10] = m[ 2] - m[ 6];
    d[ 6] = m[10] + m[14];
    d[14] = m[14] - m[10];
    d[ 3] = m[ 3] + m[ 7];
    d[11] = m[ 3] - m[ 7];
    d[ 7] = m[11] + m[15];
    d[15] = m[15] - m[11];
    
    m[ 0] = d[ 0] + d[ 3];
    m[ 1] = d[ 1] + d[ 2];
    m[ 2] = d[ 1] - d[ 2];
    m[ 3] = d[ 0] - d[ 3];
    m[ 4] = d[ 4] + d[ 7];
    m[ 5] = d[ 5] + d[ 6];
    m[ 6] = d[ 5] - d[ 6];
    m[ 7] = d[ 4] - d[ 7];
    m[ 8] = d[ 8] + d[11];
    m[ 9] = d[ 9] + d[10];
    m[10] = d[ 9] - d[10];
    m[11] = d[ 8] - d[11];
    m[12] = d[12] + d[15];
    m[13] = d[13] + d[14];
    m[14] = d[13] - d[14];
    m[15] = d[12] - d[15];
    
    d[ 0] = m[ 0] + m[ 1];
    d[ 1] = m[ 0] - m[ 1];
    d[ 2] = m[ 2] + m[ 3];
    d[ 3] = m[ 3] - m[ 2];
    d[ 4] = m[ 4] + m[ 5];
    d[ 5] = m[ 4] - m[ 5];
    d[ 6] = m[ 6] + m[ 7];
    d[ 7] = m[ 7] - m[ 6];
    d[ 8] = m[ 8] + m[ 9];
    d[ 9] = m[ 8] - m[ 9];
    d[10] = m[10] + m[11];
    d[11] = m[11] - m[10];
    d[12] = m[12] + m[13];
    d[13] = m[12] - m[13];
    d[14] = m[14] + m[15];
    d[15] = m[15] - m[14];
    
    /*===== sum up =====*/
    for (dd=diff[k=0]; k<16; dd=diff[++k])
    {
      satd += (dd < 0 ? -dd : dd);
    }
    satd >>= 1;
  }
  else	// ���δ����Hadamard�任����ֱ�ӽ�16���в�ֵȡ����ֵ�����Ϊ����
  {
    /*===== sum up =====*/
    for (k = 0; k < 16; k++)
    {
      satd += byte_abs [diff [k]];
    }
  }
  
  return satd;
}



/*!
 ***********************************************************************
 * \brief��ȷ����������ص�Ϊ���ģ����а����ع��ƣ�������Ѱ����ص�Ϊ���ģ�����
 1/4���ع���
 *    Sub pixel block motion search
 ***********************************************************************
 */
int                                               //  ==> minimum motion cost after search
SubPelBlockMotionSearch (pel_t**   orig_pic,      // <--  original pixel values for the AxB block
                         int       ref,           // <--  reference frame (0... or -1 (backward))
                         int       list,          // <--  reference picture list 
                         int       pic_pix_x,     // <--  absolute x-coordinate of regarded AxB block
                         int       pic_pix_y,     // <--  absolute y-coordinate of regarded AxB block
                         int       blocktype,     // <--  block type (1-16x16 ... 7-4x4)
                         int       pred_mv_x,     // <--  motion vector predictor (x) in sub-pel units
                         int       pred_mv_y,     // <--  motion vector predictor (y) in sub-pel units
                         int*      mv_x,          // <--> in: search center (x) / out: motion vector (x) - in pel units
                         int*      mv_y,          // <--> in: search center (y) / out: motion vector (y) - in pel units
                         int       search_pos2,   // <--  �������������λ�� (default: 9)
                         int       search_pos4,   // <--  search positions for quarter-pel search  (default: 9)
                         int       min_mcost,     // <--  minimum motion cost (cost for center or huge value)
                         double    lambda         // <--  lagrangian parameter for determining motion cost
                         )
{
  int   diff[16], *d;
  int   pos, best_pos, mcost, abort_search;
  int   y0, x0, ry0, rx0, ry;
  int   cand_mv_x, cand_mv_y;
  int   max_pos_x4, max_pos_y4;
  pel_t *orig_line;
  pel_t **ref_pic;      
  StorablePicture *ref_picture;
  int   lambda_factor   = LAMBDA_FACTOR (lambda);
  int   mv_shift        = 0;
  int   check_position0 = (blocktype==1 && *mv_x==0 && *mv_y==0 && input->hadamard && !input->rdopt && img->type!=B_SLICE && ref==0);
  int   blocksize_x     = input->blc_size[blocktype][0];
  int   blocksize_y     = input->blc_size[blocktype][1];
  int   pic4_pix_x      = (pic_pix_x << 2);//�������1/4����Ϊ��λ������
  int   pic4_pix_y      = (pic_pix_y << 2);
  int   min_pos2        = (input->hadamard ? 0 : 1);
  int   max_pos2        = (input->hadamard ? max(1,search_pos2) : search_pos2);
  int   list_offset     = ((img->MbaffFrameFlag)&&(img->mb_data[img->current_mb_nr].mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

  int  apply_weights = ( (active_pps->weighted_pred_flag && (img->type == P_SLICE || img->type == SP_SLICE)) ||
                         (active_pps->weighted_bipred_idc && (img->type == B_SLICE)));  //�Ƿ�ʹ��Ȩ�ؼ������

  int   img_width, img_height;
  
  ref_picture     = listX[list+list_offset][ref];

  if (apply_weights)
  {
    ref_pic = listX[list+list_offset][ref]->imgY_ups_w;
  }
  else
    ref_pic = listX[list+list_offset][ref]->imgY_ups; //����1/4���ز�ֵ��ͼ�����imgY_ups

  img_width  = ref_picture->size_x;
  img_height = ref_picture->size_y;//δ����ֵ��ͼ��Ŀ���

  max_pos_x4      = ((ref_picture->size_x - blocksize_x+1)<<2);//���1/4��������λ��
  max_pos_y4      = ((ref_picture->size_y - blocksize_y+1)<<2);
  
  /*********************************
   *****                       *****
   *****  HALF-PEL REFINEMENT  *****
   *****                       *****
   *********************************/
  //===== convert search center to quarter-pel units ===== �ȱ��1/4���ص��˶�����
  *mv_x <<= 2;
  *mv_y <<= 2;
  //===== set function for getting pixel values =====
  if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 2) &&
      (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 2)   )//�жϰ������Ƿ����
  {
    PelY_14 = FastPelY_14;//û����
  }
  else
  {
    PelY_14 = UMVPelY_14;
  }
  //===== loop over search positions =====
  for (best_pos = 0, pos = min_pos2; pos < max_pos2; pos++)	//������������ֻ���������9����(max_pos2==9)
  {
    cand_mv_x = *mv_x + (spiral_search_x[pos] << 1);    // quarter-pel units
    cand_mv_y = *mv_y + (spiral_search_y[pos] << 1);    // quarter-pel units

    //----- set motion vector cost -----
    mcost = MV_COST (lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);
    if (check_position0 && pos==0)
    {
      mcost -= WEIGHTED_COST (lambda_factor, 16);
    }

    if (mcost >= min_mcost) continue;

    //----- add up SATD -----
    for (y0=0, abort_search=0; y0<blocksize_y && !abort_search; y0+=4)
    {
      ry0 = ((pic_pix_y+y0)<<2) + cand_mv_y;//��������

      for (x0=0; x0<blocksize_x; x0+=4)//4*4�����SATD
      {
        rx0 = ((pic_pix_x+x0)<<2) + cand_mv_x;
        d   = diff;//�����ڴ�ռ�
//�ҵ�1/4���ص��������ģ����ݲ�ֵ�������ֵ����������ֵͬλ�ó����
        orig_line = orig_pic [y0  ];    ry=ry0;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+1];    ry=ry0+4;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+2];    ry=ry0+8;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+3];    ry=ry0+12;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d        = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        if ((mcost += SATD (diff, input->hadamard)) > min_mcost)
        {
          abort_search = 1;
          break;
        }//����С���۴���ֹ����
      }
    }

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos;
    }
  }
  if (best_pos)
  {
    *mv_x += (spiral_search_x [best_pos] << 1);
    *mv_y += (spiral_search_y [best_pos] << 1);
  }


  /************************************
   *****                          *****
   *****  QUARTER-PEL REFINEMENT  *****
   *****                          *****
   ************************************/
  //===== set function for getting pixel values =====
  if ((pic4_pix_x + *mv_x > 1) && (pic4_pix_x + *mv_x < max_pos_x4 - 1) &&
      (pic4_pix_y + *mv_y > 1) && (pic4_pix_y + *mv_y < max_pos_y4 - 1)   )
  {
    PelY_14 = FastPelY_14;
  }
  else
  {
    PelY_14 = UMVPelY_14;
  }
  //===== loop over search positions =====
  for (best_pos = 0, pos = 1; pos < search_pos4; pos++)//1/4����������ֻ���������9����(max_pos2==9)
  {
    cand_mv_x = *mv_x + spiral_search_x[pos];    // quarter-pel units
    cand_mv_y = *mv_y + spiral_search_y[pos];    // quarter-pel units

    //----- set motion vector cost -----
    mcost = MV_COST (lambda_factor, mv_shift, cand_mv_x, cand_mv_y, pred_mv_x, pred_mv_y);

    if (mcost >= min_mcost) continue;

    //----- add up SATD -----
    for (y0=0, abort_search=0; y0<blocksize_y && !abort_search; y0+=4)
    {
      ry0 = ((pic_pix_y+y0)<<2) + cand_mv_y;

      for (x0=0; x0<blocksize_x; x0+=4)
      {
        rx0 = ((pic_pix_x+x0)<<2) + cand_mv_x;
        d   = diff;

        orig_line = orig_pic [y0  ];    ry=ry0;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+1];    ry=ry0+4;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+2];    ry=ry0+8;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d++      = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        orig_line = orig_pic [y0+3];    ry=ry0+12;
        *d++      = orig_line[x0  ]  -  PelY_14 (ref_pic, ry, rx0   , img_height, img_width);
        *d++      = orig_line[x0+1]  -  PelY_14 (ref_pic, ry, rx0+ 4, img_height, img_width);
        *d++      = orig_line[x0+2]  -  PelY_14 (ref_pic, ry, rx0+ 8, img_height, img_width);
        *d        = orig_line[x0+3]  -  PelY_14 (ref_pic, ry, rx0+12, img_height, img_width);

        if ((mcost += SATD (diff, input->hadamard)) > min_mcost)
        {
          abort_search = 1;
          break;
        }
      }
    }

    if (mcost < min_mcost)
    {
      min_mcost = mcost;
      best_pos  = pos;
    }
  }
  if (best_pos)
  {
    *mv_x += spiral_search_x [best_pos];
    *mv_y += spiral_search_y [best_pos];
  }

  //===== return minimum motion cost =====
  return min_mcost;
}



/*!
 ***********************************************************************
 * \brief
 *    Block motion search  ����һ���ֿ���˶������������˶�����
 ***********************************************************************
 */
int                                         //!< minimum motion cost after search
BlockMotionSearch (int       ref,           //!< reference idx
                   int       list,          //!< reference pciture list
                   int       mb_x,          //!< x-coordinate inside macroblock ������Ϊ��λMB������
                   int       mb_y,          //!< y-coordinate inside macroblock ������Ϊ��λ
                   int       blocktype,     //!< block type (1-16x16 ... 7-4x4)
                   int       search_range,  //!< 1-d search range for integer-position search
                   double    lambda         //!< lagrangian parameter for determining motion cost
                   )
{
  static pel_t   orig_val [256];//�洢һ��MB������
  //�������������ʹ2ά��������ٶ��ϻ��һЩ����Ϊ�������±����
  static pel_t  *orig_pic  [16] = {orig_val,     orig_val+ 16, orig_val+ 32, orig_val+ 48,
                                   orig_val+ 64, orig_val+ 80, orig_val+ 96, orig_val+112,
                                   orig_val+128, orig_val+144, orig_val+160, orig_val+176,
                                   orig_val+192, orig_val+208, orig_val+224, orig_val+240};

  int       pred_mv_x, pred_mv_y, mv_x, mv_y, i, j;

  int       max_value = (1<<20);//40
  int       min_mcost = max_value; //��������С����,�ȴ�����ֵ

  int       block_x   = (mb_x>>2); //�ٱ����4x4��Ϊ��λ
  int       block_y   = (mb_y>>2); //ͬ��
  
  int       bsx       = input->blc_size[blocktype][0]; //block size x
  int       bsy       = input->blc_size[blocktype][1]; //block size y

  int       pic_pix_x = img->opix_x + mb_x;   //img->opix_xy ���ԭ�����������
  int       pic_pix_y = img->opix_y + mb_y;

  int*      pred_mv;

  int***    mv_array  = enc_picture->mv[list];

  int****** all_mv    = img->all_mv;

#ifdef WIN32
  struct _timeb tstruct1;
  struct _timeb tstruct2;
#else
  struct timeb tstruct1;
  struct timeb tstruct2;
#endif
  
  int me_tmp_time;

  int  N_Bframe=0, n_Bframe=0;
  if(input->FMEnable)//�����˶�����
  {
    N_Bframe = input->successive_Bframe;
    n_Bframe =(N_Bframe) ? ((Bframe_ctr%N_Bframe)+1) : 0 ;
  }

   pred_mv = img->pred_mv[block_x][block_y][list][ref][blocktype];

  //==================================
  //=====   GET ORIGINAL BLOCK   =====
  //==================================
  for (j = 0; j < bsy; j++)
  {
    for (i = 0; i < bsx; i++)
    {
      orig_pic[j][i] = imgY_org[pic_pix_y+j][pic_pix_x+i];	//�õ�������ԭʼͼ�����������ֵ
    }
  }

  //���ʹ�ÿ����˶����ƣ�Ĭ���ǲ�ʹ�õ�(baseline,main,extendedһ��)
  //Ҫ�˽���δ��������Ҫ�Ķ�������ģ���׼����û�еģ�����FME�Ĵ�����ʱ����
  if(input->FMEnable)
  {
    
    if(blocktype>6)
    {//all_mv[block_x][block_y][list][ref][blocktype][MV_x/MV_y]
    //all_bwmincost[block_x][block_y][ref][blocktype][0:SAD 1:MV_x 2:MV_y]
      pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][5][0];
      pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][5][1];
      pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][5][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][5][0]);
      pred_SAD_uplayer   /= 2; 
      
    }
    else if(blocktype>4)
    {
      pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][4][0];
      pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][4][1];
      pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][4][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][4][0]);
      pred_SAD_uplayer   /= 2; 
      
    }
    else if(blocktype == 4)
    {
      pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][2][0];
      pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][2][1];
      pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][2][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][2][0]);
      pred_SAD_uplayer   /= 2; 
    }
    else if(blocktype > 1)
    {
      pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][1][0];
      pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][1][1];
      pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][1][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][1][0]);
      pred_SAD_uplayer   /= 2; 
    }
    
    if ((img->type==B_SLICE)&& (img->nal_reference_idc>0))
    {
      if(blocktype>6)
      {
        pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][5][0];
        pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][5][1];
        pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][5][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][5][0]);
        pred_SAD_uplayer   /= 2; 
      }
      else if(blocktype>4)
      {
        pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][4][0];
        pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][4][1];
        pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][4][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][4][0]);
        pred_SAD_uplayer   /= 2; 
      }
      else if(blocktype == 4)
      {
        pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][2][0];
        pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][2][1];
        pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][2][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][2][0]);
        pred_SAD_uplayer   /= 2; 
      }
      else if(blocktype > 1)
      {
        pred_MV_uplayer[0] = all_mv[block_x][block_y][list][ref][1][0];
        pred_MV_uplayer[1] = all_mv[block_x][block_y][list][ref][1][1];
        pred_SAD_uplayer    = (list==1) ? (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][1][0]) : (all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][1][0]);
        pred_SAD_uplayer   /= 2; 
      }
    }
    
    pred_SAD_uplayer = flag_intra_SAD ? 0 : pred_SAD_uplayer;// for irregular motion
    
    //Coordinate prediction
    if (img->number > ref+1)
    {
      pred_SAD_time = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][0];
      pred_MV_time[0] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][1];
      pred_MV_time[1] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][2];
    }
    
    if(list==1 && (Bframe_ctr%N_Bframe) > 1) 
    {
      pred_SAD_time = all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][0];
      pred_MV_time[0] = (int)(all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][blocktype][1] * ((n_Bframe==1) ? (N_Bframe) : (N_Bframe-n_Bframe+1.0)/(N_Bframe-n_Bframe+2.0)) );//should add a factor
      pred_MV_time[1] = (int)(all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][blocktype][2] *((n_Bframe==1) ? (N_Bframe) : (N_Bframe-n_Bframe+1.0)/(N_Bframe-n_Bframe+2.0)) );//should add a factor
    }
    
    if (input->PicInterlace == FIELD_CODING) 
    {
      if (img->type == P_SLICE && ref > 1)
      {
        pred_SAD_ref = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-2)][blocktype][0];
        pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
        pred_MV_ref[0] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-2)][blocktype][1];
        pred_MV_ref[0] = (int)(pred_MV_ref[0]*((ref>>1)+1)/(float)((ref>>1)));
        pred_MV_ref[1] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-2)][blocktype][2];
        pred_MV_ref[1] = (int)(pred_MV_ref[1]*((ref>>1)+1)/(float)((ref>>1)));
      }
      if (img->type == B_SLICE && list==0 && (ref==0 || ref==1) )
      {
        pred_SAD_ref = all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][blocktype][0];
        pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
        pred_MV_ref[0] =(int) (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][1]*(-n_Bframe)/(N_Bframe-n_Bframe+1.0f)); //should add a factor
        pred_MV_ref[1] =(int) ( all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][2]*(-n_Bframe)/(N_Bframe-n_Bframe+1.0f)); 
      }
    }
    else //frame case
    {
      if (ref > 0)
      {//field_mode top_field
        pred_SAD_ref = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-1)][blocktype][0];
        pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
        pred_MV_ref[0] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-1)][blocktype][1];
        pred_MV_ref[0] = (int)(pred_MV_ref[0]*(ref+1)/(float)(ref));
        pred_MV_ref[1] = all_mincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][(ref-1)][blocktype][2];
        pred_MV_ref[1] = (int)(pred_MV_ref[1]*(ref+1)/(float)(ref));
      }
      if (img->type == B_SLICE && (list==0 && ref==0)) //B frame forward prediction, first ref
      {
        pred_SAD_ref = all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][0][blocktype][0];
        pred_SAD_ref = flag_intra_SAD ? 0 : pred_SAD_ref;//add this for irregular motion
        pred_MV_ref[0] =(int) (all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][1]*(-n_Bframe)/(N_Bframe-n_Bframe+1.0f)); //should add a factor
        pred_MV_ref[1] =(int) ( all_bwmincost[(img->pix_x>>2)+block_x][(img->pix_y>>2)+block_y][ref][blocktype][2]*(-n_Bframe)/(N_Bframe-n_Bframe+1.0f)); 
      }
    }
 }
 
  //===========================================
  //=====   GET MOTION VECTOR PREDICTOR   =====
  //===========================================

  if (input->FMEnable) 
    FME_blocktype=blocktype;

   //����Ԥ��MV������pred_mv�У����ķ�֮һ����Ϊ��λ
  SetMotionVectorPredictor (pred_mv, enc_picture->ref_idx, enc_picture->mv, ref, list, block_x, block_y, bsx, bsy);
  
  pred_mv_x = pred_mv[0];
  pred_mv_y = pred_mv[1]; //������˶����ƾ��ǻ���Ԥ��mv��
#ifdef WIN32
  _ftime( &tstruct1 );    // start time ms
#else
  ftime(&tstruct1);
#endif

  //==================================
  //=====   INTEGER-PEL SEARCH ===== �����˶�����
  //==================================

  if(input->FMEnable)
  {
    mv_x = pred_mv_x / 4;   
    mv_y = pred_mv_y / 4;
    if (!input->rdopt)
    {
      //--- adjust search center so that the (0,0)-vector is inside ---
      mv_x = max (-search_range, min (search_range, mv_x));
      mv_y = max (-search_range, min (search_range, mv_y));
	  //ʹmv_x,mv_y��(-search_range,search_range)֮��
    }
    
    min_mcost = FastIntegerPelBlockMotionSearch(orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                                                          pred_mv_x, pred_mv_y, &mv_x, &mv_y, 
                                                                          search_range,min_mcost, lambda);
    //FAST MOTION ESTIMATION. ZHIBO CHEN 2003.3
    for (i=0; i < (bsx>>2); i++)
    {
      for (j=0; j < (bsy>>2); j++)
      {
        if(list == 0) 
          all_mincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][0] = min_mcost;
        else
          all_bwmincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][0] = min_mcost; 
      }
    }
  }
  else
  {
#ifndef _FAST_FULL_ME_

    //--- set search center ---
    mv_x = pred_mv_x / 4;    //���������������Ϊ��λ��MV
    mv_y = pred_mv_y / 4;
    if (!input->rdopt)
    {
      //--- adjust search center so that the (0,0)-vector is inside ---
      /*ʹmv_x,mv_y��(-search_range,search_range)֮�䣬
      �����������ļ��е�RestrictSearchRangeѡ�����ͨ����������ʹ��ͬ�ο�֡
      �Լ���ͬ��С�Ŀ��������Χ��ͬ�����ԣ�Ԥ��MV�п��ܴ���search_range
   */
      mv_x = max (-search_range, min (search_range, mv_x));
      mv_y = max (-search_range, min (search_range, mv_y));
    }
    
    //--- perform motion search ---
    min_mcost = FullPelBlockMotionSearch     (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                              pred_mv_x, pred_mv_y, &mv_x, &mv_y, search_range,
                                              min_mcost, lambda);

#else

    // comments:   - orig_pic is not used  -> be careful
    //             - search center is automatically determined
    min_mcost = FastFullPelBlockMotionSearch (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                              pred_mv_x, pred_mv_y, &mv_x, &mv_y, search_range,
                                              min_mcost, lambda);

#endif
  }

#ifdef WIN32
      _ftime(&tstruct2);   // end time ms
#else
      ftime(&tstruct2);    // end time ms
#endif
      
      me_tmp_time=(tstruct2.time*1000+tstruct2.millitm) - (tstruct1.time*1000+tstruct1.millitm); 
      me_tot_time += me_tmp_time;
      me_time += me_tmp_time;

  //==============================
  //=====   SUB-PEL SEARCH   =====
  //==============================
  if (input->hadamard)
  {
    min_mcost = max_value;
  }

  if(input->FMEnable)
  {
    if(blocktype >3)
    {
      min_mcost =  FastSubPelBlockMotionSearch (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                                pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9,
                                                min_mcost, lambda, /*useABT*/0);
    }
    else
    {
      min_mcost =  SubPelBlockMotionSearch (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                            pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9,
                                            min_mcost, lambda);
    }


    for (i=0; i < (bsx>>2); i++)
    {
      for (j=0; j < (bsy>>2); j++)
      {
        if(list == 0)
        {
          all_mincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][1] = mv_x;
          all_mincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][2] = mv_y;
        }
        else
        {
          all_bwmincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][1] = mv_x;
          all_bwmincost[(img->pix_x>>2)+block_x+i][(img->pix_y>>2)+block_y+j][ref][blocktype][2] = mv_y;
          
        }
      }
    }
  }
  else  //���з������������������������֮��MV�ֱ������1/4����Ϊ��λ
  {
    min_mcost =  SubPelBlockMotionSearch (orig_pic, ref, list, pic_pix_x, pic_pix_y, blocktype,
                                          pred_mv_x, pred_mv_y, &mv_x, &mv_y, 9, 9,
                                          min_mcost, lambda);
  }


  if (!input->rdopt)
  {
    // Get the skip mode cost�õ���Ծ���Ĵ���
    if (blocktype == 1 && (img->type == P_SLICE||img->type == SP_SLICE))
    {
      int cost;

      FindSkipModeMotionVector ();

      cost  = GetSkipCostMB (lambda);
      cost -= (int)floor(8*lambda+0.4999);

      if (cost < min_mcost)
      {
        min_mcost = cost;
        mv_x      = img->all_mv [0][0][0][0][0][0];
        mv_y      = img->all_mv [0][0][0][0][0][1];
      }
    }
  }

  //===============================================
  //=====   SET MV'S AND RETURN MOTION COST   =====
  //===============================================

  if(input->FMEnable)
  {
    int h4x4blkno = (img->pix_x>>2)+block_x;  
    int v4x4blkno = (img->pix_y>>2)+block_y;
    for (i=0; i < (bsx>>2); i++)
    {
      for (j=0; j < (bsy>>2); j++)
      {
        all_mv[block_x+i][block_y+j][list][ref][blocktype][0] = mv_x;
        all_mv[block_x+i][block_y+j][list][ref][blocktype][1] = mv_y;
        mv_array[h4x4blkno+i][v4x4blkno+j][0] = mv_x;
        mv_array[h4x4blkno+i][v4x4blkno+j][1] = mv_y;
      }
    }
  }
  else
  {
    for (i=0; i < (bsx>>2); i++)
    {
      for (j=0; j < (bsy>>2); j++)
      {
        all_mv[block_x+i][block_y+j][list][ref][blocktype][0] = mv_x;
        all_mv[block_x+i][block_y+j][list][ref][blocktype][1] = mv_y;
      }
    }
  }

  return min_mcost; //chj ������С����
}


/*!
 ***********************************************************************
 * \brief
 *    Motion Cost for Bidirectional modes
 ***********************************************************************
 */
int BIDPartitionCost (int   blocktype,
                      int   block8x8,
                      int   fw_ref,
                      int   bw_ref,
                      int   lambda_factor)
{
  static int  bx0[5][4] = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,2,0,0}, {0,2,0,2}};
  static int  by0[5][4] = {{0,0,0,0}, {0,0,0,0}, {0,2,0,0}, {0,0,0,0}, {0,0,2,2}};

  int   diff[16];
  int   pic_pix_x, pic_pix_y, block_x, block_y;
  int   v, h, mcost, i, j, k;
  int   mvd_bits  = 0;
  int   parttype  = (blocktype<4?blocktype:4);
  int   step_h0   = (input->blc_size[ parttype][0]>>2);
  int   step_v0   = (input->blc_size[ parttype][1]>>2);
  int   step_h    = (input->blc_size[blocktype][0]>>2);
  int   step_v    = (input->blc_size[blocktype][1]>>2);
  int   bxx, byy;                               // indexing curr_blk

  int   ******all_mv = img->all_mv;
  int   ******  p_mv = img->pred_mv;

  //----- cost for motion vector bits -----
  for (v=by0[parttype][block8x8]; v<by0[parttype][block8x8]+step_v0; v+=step_v)
  for (h=bx0[parttype][block8x8]; h<bx0[parttype][block8x8]+step_h0; h+=step_h)
  {
    mvd_bits += mvbits[ all_mv [h][v][LIST_0][fw_ref][blocktype][0] - p_mv[h][v][LIST_0][fw_ref][blocktype][0] ];
    mvd_bits += mvbits[ all_mv [h][v][LIST_0][fw_ref][blocktype][1] - p_mv[h][v][LIST_0][fw_ref][blocktype][1] ];

    mvd_bits += mvbits[ all_mv [h][v][LIST_1][bw_ref][blocktype][0] - p_mv[h][v][LIST_1][bw_ref][blocktype][0] ];
    mvd_bits += mvbits[ all_mv [h][v][LIST_1][bw_ref][blocktype][1] - p_mv[h][v][LIST_1][bw_ref][blocktype][1] ];
  }

  mcost = WEIGHTED_COST (lambda_factor, mvd_bits);

  //----- cost of residual signal -----
  for (byy=0, v=by0[parttype][block8x8]; v<by0[parttype][block8x8]+step_v0; byy+=4, v++)
  {
    pic_pix_y = img->opix_y + (block_y = (v<<2));

    for (bxx=0, h=bx0[parttype][block8x8]; h<bx0[parttype][block8x8]+step_h0; bxx+=4, h++)
    {
      pic_pix_x = img->opix_x + (block_x = (h<<2));

      LumaPrediction4x4 (block_x, block_y, 2, blocktype, blocktype, fw_ref, bw_ref);

      for (k=j=0; j<4; j++)
      for (  i=0; i<4; i++, k++)
      {
        diff[k] = imgY_org[pic_pix_y+j][pic_pix_x+i] - img->mpr[i+block_x][j+block_y];
      }
      mcost += SATD (diff, input->hadamard);
    }
  }
  return mcost;
}

/*!
 ************************************************************************
 * \brief
 *    Get cost for skip mode for an macroblock
 ************************************************************************
 */
int GetSkipCostMB (double lambda)
{
  int block_y, block_x, pic_pix_y, pic_pix_x, i, j, k;
  int diff[16];
  int cost = 0;

  for (block_y=0; block_y<16; block_y+=4)
  {
    pic_pix_y = img->opix_y + block_y;

    for (block_x=0; block_x<16; block_x+=4)
    {
      pic_pix_x = img->opix_x + block_x;

      //===== prediction of 4x4 block =====
      LumaPrediction4x4 (block_x, block_y, 0, 0, 0, 0, 0);

      //===== get displaced frame difference ======                
      for (k=j=0; j<4; j++)
        for (i=0; i<4; i++, k++)
        {
          diff[k] = imgY_org[pic_pix_y+j][pic_pix_x+i] - img->mpr[i+block_x][j+block_y];
        }
      cost += SATD (diff, input->hadamard);
    }
  }

  return cost;
}

/*!
 ************************************************************************
 * \brief
 *    Find motion vector for the Skip mode
 ************************************************************************
 */
void FindSkipModeMotionVector ()
{
  int bx, by;
  int ******all_mv = img->all_mv;

  int pmv[2];

  int zeroMotionAbove;
  int zeroMotionLeft;
  PixelPos mb_a, mb_b;
  int      a_mv_y = 0;
  int      a_ref_idx = 0;
  int      b_mv_y = 0;
  int      b_ref_idx = 0;

  Macroblock *currMB = &img->mb_data[img->current_mb_nr];
  
  getLuma4x4Neighbour(img->current_mb_nr,0,0,-1, 0,&mb_a);
  getLuma4x4Neighbour(img->current_mb_nr,0,0, 0,-1,&mb_b);
  
  if (mb_a.available)
  {
    a_mv_y    = enc_picture->mv[LIST_0][mb_a.pos_x][mb_a.pos_y][1];
    a_ref_idx = enc_picture->ref_idx[LIST_0][mb_a.pos_x][mb_a.pos_y];
    //֡������ ����MV��ref_idx
    if (currMB->mb_field && !img->mb_data[mb_a.mb_addr].mb_field)
    {
      a_mv_y    /=2;
      a_ref_idx *=2;
    }
    if (!currMB->mb_field && img->mb_data[mb_a.mb_addr].mb_field)
    {
      a_mv_y    *=2;
      a_ref_idx >>=1;
    }
  }
  
  if (mb_b.available)
  {
    b_mv_y    = enc_picture->mv[LIST_0][mb_b.pos_x][mb_b.pos_y][1];
    b_ref_idx = enc_picture->ref_idx[LIST_0][mb_b.pos_x][mb_b.pos_y];
    
    if (currMB->mb_field && !img->mb_data[mb_b.mb_addr].mb_field)
    {
      b_mv_y    /=2;
      b_ref_idx *=2;
    }
    if (!currMB->mb_field && img->mb_data[mb_b.mb_addr].mb_field)
    {
      b_mv_y    *=2;
      b_ref_idx >>=1;
    }
  }
  //a��������mv_aΪ0 zeroMotionLeft=1
  zeroMotionLeft  = !mb_a.available ? 1 : a_ref_idx==0 && enc_picture->mv[LIST_0][mb_a.pos_x][mb_a.pos_y][0]==0 && a_mv_y==0 ? 1 : 0;
  zeroMotionAbove = !mb_b.available ? 1 : b_ref_idx==0 && enc_picture->mv[LIST_0][mb_b.pos_x][mb_b.pos_y][0]==0 && b_mv_y==0 ? 1 : 0;
  
  if (zeroMotionAbove || zeroMotionLeft)
  {
    for (by = 0;by < 4;by++)
      for (bx = 0;bx < 4;bx++)
      {
        all_mv [bx][by][0][0][0][0] = 0;
        all_mv [bx][by][0][0][0][1] = 0;
      }
  }
  else
  {
    SetMotionVectorPredictor (pmv, enc_picture->ref_idx, enc_picture->mv, 0, LIST_0, 0, 0, 16, 16);
    for (by = 0;by < 4;by++)
      for (bx = 0;bx < 4;bx++)
      {
        all_mv [bx][by][0][0][0][0] = pmv[0];
        all_mv [bx][by][0][0][0][1] = pmv[1];
      }
  }
}

/*!
 ************************************************************************
 * \brief
 *    Get cost for direct mode for an 8x8 block  B֡ 8x8 block��ֱ��Ԥ��ģʽ
 ************************************************************************
 */
int Get_Direct_Cost8x8 (int block, double lambda)
{
  int block_y, block_x, pic_pix_y, pic_pix_x, i, j, k;
  int diff[16];
  int cost  = 0;
  int mb_y  = (block/2)<<3; //chj �Ѻ���ڿ������ɺ���������������
  int mb_x  = (block%2)<<3;

  for (block_y=mb_y; block_y<mb_y+8; block_y+=4)//chj ��8x8��ѭ��
  {
    pic_pix_y = img->opix_y + block_y;//chj �������ͼ��ԭ��ľ�������

    for (block_x=mb_x; block_x<mb_x+8; block_x+=4)
    {
      pic_pix_x = img->opix_x + block_x;//chj ͬ��

      if (direct_pdir[pic_pix_x>>2][pic_pix_y>>2]<0) //chj if�ÿ�directģʽ������
      {
        return (1<<30); //mode not allowed
      }

      //===== prediction of 4x4 block =====
      LumaPrediction4x4 (block_x, block_y, direct_pdir[pic_pix_x>>2][pic_pix_y>>2], 0, 0, 
                         direct_ref_idx[LIST_0][pic_pix_x>>2][pic_pix_y>>2], 
                         direct_ref_idx[LIST_1][pic_pix_x>>2][pic_pix_y>>2]);
      //chj a ��������������ȫ�ֱ���img->mpr������Ҫ��
      //chj mpr�������ǲο��������
      //===== get displaced frame difference ======                
      for (k=j=0; j<4; j++)
        for (i=0; i<4; i++, k++)
        {
          diff[k] = imgY_org[pic_pix_y+j][pic_pix_x+i] - img->mpr[i+block_x][j+block_y];
        }//chj a mpr->current best prediction mode
      cost += SATD (diff, input->hadamard);
	  //chj a ���㷽��:�Ѳв��Ƚ��� Hadamard �任��Ȼ��ȡ����ֵ���
    }
  }

  return cost;
}



/*!
 ************************************************************************
 * \brief
 *    Get cost for direct mode for an macroblock �õ�B���ֱ��ģʽ����
 ************************************************************************
 */
int Get_Direct_CostMB (double lambda)
{
  int i;
  int cost = 0;
  
  for (i=0; i<4; i++)
  {
    cost += Get_Direct_Cost8x8 (i, lambda);
    if (cost >= (1<<30)) return cost;
  }
  return cost;
}


/*!
 ************************************************************************
 * \brief��ÿ���ֿ�����˶��������������
 *    Motion search for a partition
 ************************************************************************
 */
void
PartitionMotionSearch (int    blocktype,
                                                                    int    block8x8,
                                                                    double lambda)
{
  /*bx0[parttype][block8x8],by0[parttype][block8x8]
   4���ֿ飬����block8x8=4��4�����ͷָ�:mode 1 2 3 4(parttype!=0)��
   ע��mode>4��ʱ��parttypeΪ4��mode��blocktype
   parttype��4�������е��Ǻ��ָ�
   bx0[5][4] = {{0,0,0,0}(����), 
                {0,0,0,0},(mode1-16x16��ֻ�õ�һ��Ԫ��,��Ϊ�޷ֿ�) 
                {0,0,0,0},(mode2-16x8��ֻ��ǰ����Ԫ��,��Ϊֻ�������ֿ�) 
                {0,2,0,0},(mode3-8x16��ֻ��ǰ����Ԫ��,��Ϊֻ�������ֿ�)   
                {0,2,0,2}};(mode4-8x8,�ĸ�Ԫ�ض��ã�4���ֿ�)
   by0[5][4] = {{0,0,0,0}(����), 
                {0,0,0,0},(mode1-16x16��ֻ�õ�һ��Ԫ��,��Ϊ�޷ֿ�)   
                {0,2,0,0},(mode2-16x8��ֻ��ǰ����Ԫ��,��Ϊֻ�������ֿ�)   
                {0,0,0,0},(mode3-8x16��ֻ��ǰ����Ԫ��,��Ϊֻ�������ֿ�)    
                {0,0,2,2}};(mode4-8x8,�ĸ�Ԫ�ض��ã�4���ֿ�)
   bx0,by0�Ǹ���ģʽ�·ֿ����ʼ���꣬����ĵ�λ��4x4С�飬ֻ��Ҫ�����ֿ�ͼ����һĿ��Ȼ
  */
  static int  bx0[5][4] = {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,2,0,0}, {0,2,0,2}};
  static int  by0[5][4] = {{0,0,0,0}, {0,0,0,0}, {0,2,0,0}, {0,0,0,0}, {0,0,2,2}};

  int   **ref_array, ***mv_array;
  int   ref, v, h, mcost, search_range, i, j;
  int   pic_block_x, pic_block_y;
  int   bslice    = (img->type==B_SLICE);
  int   parttype  = (blocktype<4?blocktype:4); //chj ���4
  int   step_h0   = (input->blc_size[ parttype][0]>>2);/*****************************/
  int   step_v0   = (input->blc_size[ parttype][1]>>2); /*�����������Ϊ��λ������� */
  int   step_h    = (input->blc_size[blocktype][0]>>2);/*4x4С��Ϊ��λ�����������ֿ�*/
  int   step_v    = (input->blc_size[blocktype][1]>>2);/*****************************/
  int   list;
  int   numlists;
  int   list_offset;
  
  if (img->mb_data[img->current_mb_nr].mb_field) //�Ƿ�Ϊ֡������Ӧ���� 
  {
    if(img->current_mb_nr%2)
      list_offset = 4; // bottom field mb 
    else
      list_offset = 2; // top field mb
  }
  else
  {
    list_offset = 0;  // no mb aff or frame mb
  }

  numlists=bslice?2:1; //�����bslice����2��list

  //===== LOOP OVER REFERENCE FRAMES =====
  for (list=0; list<numlists;list++) 
  {
     /*listXsize��ʲô? listXsize�Ǽ�¼֡�ͳ��Ĳο�ͼ��ĸ�����
     �ο�ͼ�����list�У�list[0��1]�ǵ�ǰ����frame�ο�����
     list[2,3]��Ӧ��ǰ����list[0,1]�Ķ���(top field)�ο�����
     list[4,5]��Ӧ��ǰ����list[0,1]�ĵ׳�(bottom field)�ο�����
  */
    for (ref=0; ref < listXsize[list+list_offset]; ref++)//Ҫ�������еĲο�����list[0,5]
    {
        //----- set search range ---
        /* ����search range:
        �����ļ�����һ��:RestrictSearchRange���������full_search��
        ��������search range��˵������:
        ������ó�2�������д�С�Ŀ�������ο�֡�������ķ�Χ��Ϊsearch range��
        ������ó�1�������д�С�Ŀ���ref numΪ0�Ĳο�֡(newest)�ϵ�������ΧΪsearch range��
        ����ο�֡��Ϊһ�룻
        ������ó�0����������ϲο�֡��������Χ����֮�⣬�����з�16x16����������ټ���        
    */
#ifdef _FULL_SEARCH_RANGE_
        if      (input->full_search == 2) search_range = input->search_range;
        else if (input->full_search == 1) search_range = input->search_range /  (min(ref,1)+1);
        else    search_range = input->search_range / ((min(ref,1)+1) * min(2,blocktype));
#else
        search_range = input->search_range / ((min(ref,1)+1) * min(2,blocktype));
#endif
        
        //----- set arrays -----
        ref_array = enc_picture->ref_idx[list];//ָ���ο������б�
        mv_array  = enc_picture->mv[list];//ָ���洢MV���б�
        
        //----- init motion cost -----
        /*����˼��motion_cost���±�:1.blocktype 2.list 3.ref 4.block8x8
       ÿһ�����Բ�ͬ��blocktype��list��ref��block8x8��motion_costȫ��������
    */
        motion_cost[blocktype][list][ref][block8x8] = 0;
        
        //===== LOOP OVER SUB MACRO BLOCK partitions����   block8x8=4
        for (v=by0[parttype][block8x8]; v<by0[parttype][block8x8]+step_v0; v+=step_v)
        {
          pic_block_y = img->block_y + v;//��4x4��Ϊ��λ�ķֿ����������
          
          for (h=bx0[parttype][block8x8]; h<bx0[parttype][block8x8]+step_h0; h+=step_h)
          {   //chj parttype:�ֿ�����,block8x8�ֿ����
            pic_block_x = img->block_x + h;//��4x4��Ϊ��λ�ķֿ���Ժ�����
            
            //--- motion search for block ---
			//ѭ��ִ��ÿ����飨���Ǻ�飩
			//�ָ��ڸ����ο�֡�е��˶�������
			//���������mcost���в���� + MV���ۣ�
			//����SAD + MVD���������bit����
            mcost = BlockMotionSearch     (ref, list, h<<2, v<<2, blocktype, 
                                           search_range, lambda);
			//motion_cost�洢���ֿ鷽���ڸ��ο�֡�Ĵ���
            motion_cost[blocktype][list][ref][block8x8] += mcost;
            
            //--- set motion vectors and reference frame (for motion vector prediction) ---
            /*��all_mv��ref�浽enc_picture�ṹ���У���������Ԥ��MV*/
            for (j=0; j<step_v; j++)
              for (i=0; i<step_h; i++)
              {
                mv_array  [pic_block_x+i][pic_block_y+j][0] = img->all_mv[h][v][list][ref][blocktype][0];
                mv_array  [pic_block_x+i][pic_block_y+j][1] = img->all_mv[h][v][list][ref][blocktype][1];
                ref_array [pic_block_x+i][pic_block_y+j]    = ref;
              }
          }
        }
    }
  }
}





extern int* last_P_no;
/*********************************************
 *****      B֡ ����ֱ��ģʽ��MV         *****
 *****  Calculate Direct Motion Vectors  *****
 *****   δ�Ķ�                          *****
 *********************************************/
void Get_Direct_Motion_Vectors ()
{

  int  block_x, block_y, pic_block_x, pic_block_y, opic_block_x, opic_block_y;
  int  ******all_mvs = img->all_mv;
  int  mv_scale;
  byte **    moving_block;
  int ****   co_located_mv;
  int ***    co_located_ref_idx;
  int64 ***    co_located_ref_id;
  Macroblock *currMB = &img->mb_data[img->current_mb_nr];

  if ((img->MbaffFrameFlag)&&(currMB->mb_field))
  {
    if(img->current_mb_nr%2)
    {
      moving_block = Co_located->bottom_moving_block;
      co_located_mv = Co_located->bottom_mv;
      co_located_ref_idx = Co_located->bottom_ref_idx;
      co_located_ref_id = Co_located->bottom_ref_pic_id;
    }
    else
    {
      moving_block = Co_located->top_moving_block;
      co_located_mv = Co_located->top_mv;
      co_located_ref_idx = Co_located->top_ref_idx;
      co_located_ref_id = Co_located->top_ref_pic_id;
    }
  }
  else
  {
    moving_block = Co_located->moving_block;
    co_located_mv = Co_located->mv;
    co_located_ref_idx = Co_located->ref_idx;
    co_located_ref_id = Co_located->ref_pic_id;
  }

  if (img->direct_type)  //spatial direct mode copy from decoder
  {
    
    int fw_rFrameL, fw_rFrameU, fw_rFrameUL, fw_rFrameUR;
    int bw_rFrameL, bw_rFrameU, bw_rFrameUL, bw_rFrameUR; 
    int fw_rFrame,bw_rFrame;
    int pmvfw[2]={0,0},pmvbw[2]={0,0};

    PixelPos mb_left, mb_up, mb_upleft, mb_upright;              
    
    getLuma4x4Neighbour(img->current_mb_nr,0,0,-1, 0,&mb_left);
    getLuma4x4Neighbour(img->current_mb_nr,0,0, 0,-1,&mb_up);
    getLuma4x4Neighbour(img->current_mb_nr,0,0,16, -1,&mb_upright);
    getLuma4x4Neighbour(img->current_mb_nr,0,0, -1,-1,&mb_upleft);

    if (!img->MbaffFrameFlag)
    {
      fw_rFrameL = mb_left.available ? enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] : -1;
      fw_rFrameU = mb_up.available ? enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] : -1;
      fw_rFrameUL = mb_upleft.available ? enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] : -1;
      fw_rFrameUR = mb_upright.available ? enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] : fw_rFrameUL;      
      
      bw_rFrameL = mb_left.available ? enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] : -1;
      bw_rFrameU = mb_up.available ? enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] : -1;
      bw_rFrameUL = mb_upleft.available ? enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] : -1;
      bw_rFrameUR = mb_upright.available ? enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] : bw_rFrameUL;      
    }
    else
    {
      if (currMB->mb_field)
      {
        fw_rFrameL = mb_left.available ? 
          img->mb_data[mb_left.mb_addr].mb_field  || enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] < 0? 
          enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] : 
          enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] * 2: -1;

        fw_rFrameU = mb_up.available ? 
          img->mb_data[mb_up.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] < 0? 
          enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] : 
        enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] * 2: -1;

        fw_rFrameUL = mb_upleft.available ? 
          img->mb_data[mb_upleft.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] < 0?         
          enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] : 
        enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] *2: -1;      

        fw_rFrameUR = mb_upright.available ? 
          img->mb_data[mb_upright.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] < 0?
          enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] : 
        enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] * 2: fw_rFrameUL;      
        
        bw_rFrameL = mb_left.available ? 
          img->mb_data[mb_left.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] < 0? 
          enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] : 
        enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] * 2: -1;

        bw_rFrameU = mb_up.available ? 
          img->mb_data[mb_up.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] < 0? 
          enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] : 
        enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] * 2: -1;

        bw_rFrameUL = mb_upleft.available ? 
          img->mb_data[mb_upleft.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] < 0?         
          enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] : 
        enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] *2: -1;      

        bw_rFrameUR = mb_upright.available ? 
          img->mb_data[mb_upright.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] < 0?         
          enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] : 
        enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] * 2: bw_rFrameUL;              
      }
      else
      {
        fw_rFrameL = mb_left.available ? 
          img->mb_data[mb_left.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y]  < 0 ?
          enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y] >> 1 : 
        enc_picture->ref_idx[LIST_0][mb_left.pos_x][mb_left.pos_y]: -1;
        
        fw_rFrameU = mb_up.available ? 
          img->mb_data[mb_up.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] >> 1 :  
        enc_picture->ref_idx[LIST_0][mb_up.pos_x][mb_up.pos_y] : -1;
        
        fw_rFrameUL = mb_upleft.available ? 
          img->mb_data[mb_upleft.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y]>> 1 : 
        enc_picture->ref_idx[LIST_0][mb_upleft.pos_x][mb_upleft.pos_y] : -1;      
        
        fw_rFrameUR = mb_upright.available ? 
          img->mb_data[mb_upright.mb_addr].mb_field || enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] < 0 ? 
          enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] >> 1 :  
        enc_picture->ref_idx[LIST_0][mb_upright.pos_x][mb_upright.pos_y] : fw_rFrameUL;      
        
        bw_rFrameL = mb_left.available ? 
          img->mb_data[mb_left.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] >> 1 :  
        enc_picture->ref_idx[LIST_1][mb_left.pos_x][mb_left.pos_y] : -1;
        
        bw_rFrameU = mb_up.available ? 
          img->mb_data[mb_up.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] >> 1 : 
        enc_picture->ref_idx[LIST_1][mb_up.pos_x][mb_up.pos_y] : -1;
        
        bw_rFrameUL = mb_upleft.available ? 
          img->mb_data[mb_upleft.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] >> 1 : 
        enc_picture->ref_idx[LIST_1][mb_upleft.pos_x][mb_upleft.pos_y] : -1;      
        
        bw_rFrameUR = mb_upright.available ? 
          img->mb_data[mb_upright.mb_addr].mb_field || enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] < 0 ?
          enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] >> 1: 
        enc_picture->ref_idx[LIST_1][mb_upright.pos_x][mb_upright.pos_y] : bw_rFrameUL;      
      }
    }
    
    fw_rFrame = (fw_rFrameL >= 0 && fw_rFrameU >= 0) ? min(fw_rFrameL,fw_rFrameU): max(fw_rFrameL,fw_rFrameU);
    fw_rFrame = (fw_rFrame >= 0 && fw_rFrameUR >= 0) ? min(fw_rFrame,fw_rFrameUR): max(fw_rFrame,fw_rFrameUR);
    
    bw_rFrame = (bw_rFrameL >= 0 && bw_rFrameU >= 0) ? min(bw_rFrameL,bw_rFrameU): max(bw_rFrameL,bw_rFrameU);
    bw_rFrame = (bw_rFrame >= 0 && bw_rFrameUR >= 0) ? min(bw_rFrame,bw_rFrameUR): max(bw_rFrame,bw_rFrameUR);        
    
    if (fw_rFrame >=0)
      SetMotionVectorPredictor (pmvfw, enc_picture->ref_idx, enc_picture->mv, fw_rFrame, LIST_0, 0, 0, 16, 16);
    
    if (bw_rFrame >=0)
      SetMotionVectorPredictor (pmvbw, enc_picture->ref_idx, enc_picture->mv, bw_rFrame, LIST_1, 0, 0, 16, 16);

    for (block_y=0; block_y<4; block_y++)
    {
      pic_block_y  = (img->pix_y>>2) + block_y;
      opic_block_y = (img->opix_y>>2) + block_y;
      
      for (block_x=0; block_x<4; block_x++)
      {
        pic_block_x  = (img->pix_x>>2) + block_x;
        opic_block_x = (img->opix_x>>2) + block_x;

        if (fw_rFrame >=0)
        {
          if (!fw_rFrame  && !moving_block[opic_block_x][opic_block_y])
          {
            all_mvs [block_x][block_y][LIST_0][0][0][0] = 0;
            all_mvs [block_x][block_y][LIST_0][0][0][1] = 0;            
            direct_ref_idx[LIST_0][pic_block_x][pic_block_y]=0;       
          }
          else
          {
            all_mvs [block_x][block_y][LIST_0][fw_rFrame][0][0] = pmvfw[0];
            all_mvs [block_x][block_y][LIST_0][fw_rFrame][0][1] = pmvfw[1];
            direct_ref_idx[LIST_0][pic_block_x][pic_block_y]=fw_rFrame;              
          }
        }
        else
        {
          all_mvs [block_x][block_y][LIST_0][0][0][0] = 0;
          all_mvs [block_x][block_y][LIST_0][0][0][1] = 0;
          direct_ref_idx[LIST_0][pic_block_x][pic_block_y]=-1;          
        }

        if (bw_rFrame >=0)
        {
          if(bw_rFrame==0 && !moving_block[opic_block_x][opic_block_y])
          {                  
            all_mvs [block_x][block_y][LIST_1][0][0][0] = 0;
            all_mvs [block_x][block_y][LIST_1][0][0][1] = 0;
            direct_ref_idx[LIST_1][pic_block_x][pic_block_y]=bw_rFrame;     
          }
          else
          {
            all_mvs [block_x][block_y][LIST_1][bw_rFrame][0][0] = pmvbw[0];
            all_mvs [block_x][block_y][LIST_1][bw_rFrame][0][1] = pmvbw[1];
            direct_ref_idx[LIST_1][pic_block_x][pic_block_y]=bw_rFrame;
          }               
        }
        else
        {      
          direct_ref_idx[LIST_1][pic_block_x][pic_block_y]=-1;

          all_mvs [block_x][block_y][LIST_1][0][0][0] = 0;
          all_mvs [block_x][block_y][LIST_1][0][0][1] = 0;
        }
        
        if (fw_rFrame < 0 && bw_rFrame < 0)
        {
          direct_ref_idx[LIST_0][pic_block_x][pic_block_y] = 
          direct_ref_idx[LIST_1][pic_block_x][pic_block_y] = 0;
        }

        if      (direct_ref_idx[LIST_1][pic_block_x][pic_block_y]==-1) direct_pdir[pic_block_x][pic_block_y] = 0;
        else if (direct_ref_idx[LIST_0][pic_block_x][pic_block_y]==-1) direct_pdir[pic_block_x][pic_block_y] = 1;
        else                                                           direct_pdir[pic_block_x][pic_block_y] = 2;
      }
    }
  }
  else
  {
    //temporal direct mode copy from decoder
    for (block_y=0; block_y<4; block_y++)
    {
      pic_block_y  = (img->pix_y>>2) + block_y;
      opic_block_y = (img->opix_y>>2) + block_y;
      
      for (block_x=0; block_x<4; block_x++)
      {
        int refList; 
        int ref_idx; 

        int list_offset = ((img->MbaffFrameFlag)&&(currMB->mb_field))? img->current_mb_nr%2 ? 4 : 2 : 0;

        pic_block_x  = (img->pix_x>>2) + block_x;
        opic_block_x = (img->opix_x>>2) + block_x;
        
        refList = (co_located_ref_idx[LIST_0][opic_block_x][opic_block_y]== -1 ? LIST_1 : LIST_0);
        ref_idx = co_located_ref_idx[refList][opic_block_x][opic_block_y];
              
        // next P is intra mode
        if (ref_idx==-1)
        {
          all_mvs [block_x][block_y][LIST_0][0][0][0] = 0;
          all_mvs [block_x][block_y][LIST_0][0][0][1] = 0;
          all_mvs [block_x][block_y][LIST_1][0][0][0] = 0;
          all_mvs [block_x][block_y][LIST_1][0][0][1] = 0;
          direct_ref_idx[LIST_0][pic_block_x][pic_block_y] = 0;
          direct_ref_idx[LIST_1][pic_block_x][pic_block_y] = 0;
          direct_pdir[pic_block_x][pic_block_y] = 2;
        }
        // next P is skip or inter mode
        else 
        {
          int mapped_idx=INVALIDINDEX;
          int iref; 

          {
            for (iref=0;iref<min(img->num_ref_idx_l0_active,listXsize[LIST_0+list_offset]);iref++)
            {
              if (enc_picture->ref_pic_num[LIST_0 +list_offset][iref]==co_located_ref_id[refList ][opic_block_x][opic_block_y])
              {
                mapped_idx=iref;
                break;
              }
              else //! invalid index. Default to zero even though this case should not happen
              {                        
                mapped_idx=INVALIDINDEX;
              }
            }
          }

          if (mapped_idx !=INVALIDINDEX)
          {
            mv_scale = img->mvscale[LIST_0+list_offset][mapped_idx];

            if (mv_scale==9999)
            {
              // forward
              all_mvs [block_x][block_y][LIST_0][0][0][0] = co_located_mv[refList][opic_block_x][opic_block_y][0];
              all_mvs [block_x][block_y][LIST_0][0][0][1] = co_located_mv[refList][opic_block_x][opic_block_y][1];
              // backward
              all_mvs [block_x][block_y][LIST_1][       0][0][0] = 0;
              all_mvs [block_x][block_y][LIST_1][       0][0][1] = 0;
            }else
            {
              // forward
              all_mvs [block_x][block_y][LIST_0][mapped_idx][0][0] = (mv_scale * co_located_mv[refList][opic_block_x][opic_block_y][0] + 128) >> 8;
              all_mvs [block_x][block_y][LIST_0][mapped_idx][0][1] = (mv_scale * co_located_mv[refList][opic_block_x][opic_block_y][1] + 128) >> 8;
              // backward
              all_mvs [block_x][block_y][LIST_1][       0][0][0] = ((mv_scale - 256)* co_located_mv[refList][opic_block_x][opic_block_y][0] + 128) >> 8;
              all_mvs [block_x][block_y][LIST_1][       0][0][1] = ((mv_scale - 256)* co_located_mv[refList][opic_block_x][opic_block_y][1] + 128) >> 8;
            }
            direct_ref_idx[LIST_0][pic_block_x][pic_block_y] = mapped_idx;
            direct_ref_idx[LIST_1][pic_block_x][pic_block_y] = 0;
            direct_pdir[pic_block_x][pic_block_y] = 2;
          }
          else
          {
            direct_ref_idx[LIST_0][pic_block_x][pic_block_y] = -1;
            direct_ref_idx[LIST_1][pic_block_x][pic_block_y] = -1;
            direct_pdir[pic_block_x][pic_block_y] = -1;
          }
        }
      }
    }
  }
}

/*!
 ************************************************************************
 * \brief
 *    control the sign of a with b
 ************************************************************************
 */
int sign(int a,int b)
{
  int x;
  x=absm(a);
  if (b >= 0)
    return x;
  else
    return -x;
}
