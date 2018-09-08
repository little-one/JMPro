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
 **************************************************************************************
 * \file
 *    slice.c
 * \brief
 *    generate the slice header, setup the bit buffer for slices,
 *    and generates the slice NALU(s)

 * \author
 *    Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Thomas Stockhammer            <stockhammer@ei.tum.de>
 *      - Detlev Marpe                  <marpe@hhi.de>
 *      - Stephan Wenger                <stewe@cs.tu-berlin.de>
 ***************************************************************************************
 */

#include "contributors.h"

#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "global.h"
#include "header.h"
#include "rtp.h"
#include "nalu.h"
#include "annexb.h"
#include "parset.h"
#include "fmo.h"
#include "vlc.h"
#include "image.h"
#include "cabac.h"
#include "elements.h"
#include "mbuffer.h"

// Local declarations
#ifdef MY_SECRET_ENCODE
extern char* SecretBinaryBitStream;
extern int SecretPosition;
extern int SecretBitNum;
static const int SearchOrderForMark[8][2] = {
	{ 5, 3 }, { 4, 3 }, { 5, 2 }, { 4, 2 }, { 5, 1 }, { 4, 1 }, { 5, 0 }, { 4, 0 }
};
#endif

#ifndef Q_BITS
#define Q_BITS 15
#endif

#ifndef OUTPUT_EMBED_POSITION
#define OUTPUT_EMBED_POSITION
#endif

static const int dequant_coef[6][4][4] = {
	{ { 10, 13, 10, 13 }, { 13, 16, 13, 16 }, { 10, 13, 10, 13 }, { 13, 16, 13, 16 } },
	{ { 11, 14, 11, 14 }, { 14, 18, 14, 18 }, { 11, 14, 11, 14 }, { 14, 18, 14, 18 } },
	{ { 13, 16, 13, 16 }, { 16, 20, 16, 20 }, { 13, 16, 13, 16 }, { 16, 20, 16, 20 } },
	{ { 14, 18, 14, 18 }, { 18, 23, 18, 23 }, { 14, 18, 14, 18 }, { 18, 23, 18, 23 } },
	{ { 16, 20, 16, 20 }, { 20, 25, 20, 25 }, { 16, 20, 16, 20 }, { 20, 25, 20, 25 } },
	{ { 18, 23, 18, 23 }, { 23, 29, 23, 29 }, { 18, 23, 18, 23 }, { 23, 29, 23, 29 } }
};

extern const byte SNGL_SCAN[16][2];
extern const byte FIELD_SCAN[16][2];

//#undef MY_SECRET

static Slice *malloc_slice();
static void  free_slice(Slice *slice);
static void  init_slice(int start_mb_addr);
static void set_ref_pic_num();
extern ColocatedParams *Co_located;
extern StorablePicture **listX[6];

/*!
 ************************************************************************
 * \brief
 *    init_ref_pic_list_reordering initializations should go here
 ************************************************************************
 */
void init_ref_pic_list_reordering()
{
	Slice* currSlice = img->currentSlice;

	currSlice->ref_pic_list_reordering_flag_l0 = 0;
	currSlice->ref_pic_list_reordering_flag_l1 = 0;
}


/*!
 ************************************************************************
 *  \brief
 *     This function generates the slice (and partition) header(s)
 *
 *  \return number of bits used for the slice (and partition) header(s)
 *
 *  \par Side effects:
 *      Adds slice/partition header symbols to the symbol buffer
 *      increments Picture->no_slices, allocates memory for the
 *      slice, sets img->currSlice
 ************************************************************************
 */
int start_slice()
{
	EncodingEnvironmentPtr eep;
	Slice *currSlice = img->currentSlice;
	Bitstream *currStream;
	int header_len = 0;
	int i;
	int NumberOfPartitions = (input->partition_mode == PAR_DP_1 ? 1 : 3);

	//one  partition for IDR img
	if (img->currentPicture->idr_flag)
	{
		NumberOfPartitions = 1;
	}

	init_ref_pic_list_reordering();	// 通过这里的初始化来设定编码时候是否启用参考图像重排序功能

	RTPUpdateTimestamp(img->tr);   // this has no side effects, just leave it for all NALs

	for (i = 0; i < NumberOfPartitions; i++)
	{
		currStream = (currSlice->partArr[i]).bitstream;

		currStream->write_flag = 0;
		if (i == 0)     // First partition
			header_len += SliceHeader(0);	// 参见标准7.3.3、7.4.3节
		else          // Second/Third partition
			header_len += Partition_BC_Header(i);

		//! Initialize CABAC
		if (input->symbol_mode == CABAC)
		{
			eep = &((currSlice->partArr[i]).ee_cabac);
			if (currStream->bits_to_go != 8)
				header_len += currStream->bits_to_go;
			writeVlcByteAlign(currStream);
			arienco_start_encoding(eep, currStream->streamBuffer, &(currStream->byte_pos)/*, &(currStream->last_startcode)*/, img->type);
			cabac_new_slice();
		}
		else
		{
			// Initialize CA-VLC
			CAVLC_init();
		}
	}
	if (input->symbol_mode == CABAC)
	{
		init_contexts();
	}
	return header_len;
}



/*!
 ************************************************************************
 * \brief
 *    This function terminates a slice (but doesn't write it out),
 *    the old terminate_slice (0)
 * \return
 *    0 if OK,                                                         \n
 *    1 in case of error
 *
 ************************************************************************
 */
int terminate_slice()
{
	int bytes_written;
	Bitstream *currStream;
	Slice *currSlice = img->currentSlice;
	EncodingEnvironmentPtr eep;
	int i;
	int byte_pos_before_startcode_emu_prevention;

	if (input->symbol_mode == CABAC)
		write_terminating_bit(1);      // only once, not for all partitions

	for (i = 0; i < currSlice->max_part_nr; i++)
	{
		currStream = (currSlice->partArr[i]).bitstream;
		if (input->symbol_mode == UVLC)
		{
			SODBtoRBSP(currStream);
			byte_pos_before_startcode_emu_prevention = currStream->byte_pos;
			currStream->byte_pos = RBSPtoEBSP(currStream->streamBuffer, 0, currStream->byte_pos, 0);
			*(stat->em_prev_bits) += (currStream->byte_pos - byte_pos_before_startcode_emu_prevention) * 8;
		}
		else     // CABAC
		{
			eep = &((currSlice->partArr[i]).ee_cabac);
			// terminate the arithmetic code
			arienco_done_encoding(eep);
			currStream->bits_to_go = eep->Ebits_to_go;
			currStream->byte_buf = 0;
			bytes_written = currStream->byte_pos;
			byte_pos_before_startcode_emu_prevention = currStream->byte_pos;
			currStream->byte_pos = RBSPtoEBSP(currStream->streamBuffer, 0, currStream->byte_pos, eep->E);
			*(stat->em_prev_bits) += (currStream->byte_pos - byte_pos_before_startcode_emu_prevention) * 8;
		}           // CABAC
	}           // partition loop
	if (input->symbol_mode == CABAC)
	{
		store_contexts();
	}

	return 0;
}

/*!
 ************************************************************************
 * \brief
 *    Encodes one slice
 * \par
 *   returns the number of coded MBs in the SLice
 ************************************************************************
 */
int encode_one_slice(int SliceGroupId, Picture *pic)
{
	Boolean end_of_slice = FALSE;
	Boolean recode_macroblock;
	int len;
	int NumberOfCodedMBs = 0;
	int CurrentMbAddr;
	double FrameRDCost, FieldRDCost;

	img->cod_counter = 0;

	set_ref_pic_num();

	if (img->type == B_SLICE)
		compute_collocated(Co_located, listX);

	//获取条带组中第一个宏块
	CurrentMbAddr = FmoGetFirstMacroblockInSlice(SliceGroupId);
	// printf ("\n\nEncode_one_slice: PictureID %d SliceGroupId %d  SliceID %d  FirstMB %d \n", img->tr, SliceGroupId, img->current_slice_nr, CurrentMbInScanOrder);

	//初始化条带
	init_slice(CurrentMbAddr);
	Bytes_After_Header = img->currentSlice->partArr[0].bitstream->byte_pos;

	if (input->symbol_mode == CABAC)
	{
		SetCtxModelNumber();
	}

	/*
	  // Tian Dong: June 7, 2002 JVT-B042
	  // When the pictures are put into different layers and subseq, not all the reference frames
	  // in multi-frame buffer are valid for prediction. The acutual number of the valid reference
	  // frames, fb->num_short_used, will be given by start_slice(sym).
	  // Save the fb->short_used.
	  if (input->NumFramesInELSubSeq)
	  {
	  short_used = fb->short_used;
	  img_ref = img->nb_references;
	  }
	  */
	//生成条带头语法结构
	len = start_slice();

	// Rate control
	img->NumberofHeaderBits += len;

	// basic unit layer rate control
	if (img->BasicUnit < img->Frame_Total_Number_MB)
		img->NumberofBasicUnitHeaderBits += len;

	//  printf("short size, used, num-used: (%d,%d,%d)\n", fb->short_size, fb->short_used, fb->num_short_used);

	/*
	  // Tian Dong: June 7, 2002 JVT-B042
	  if (input->NumFramesInELSubSeq)
	  {
	  fb->short_used = fb->num_short_used;
	  img->nb_references = fb->short_used + fb->long_used;
	  }
	  */
	// Update statistics
	stat->bit_slice += len;
	stat->bit_use_header[img->type] += len;
	// printf ("\n\n");
	while (end_of_slice == FALSE) // loop over macroblocks
	{
		//sw paff
		if (!img->MbaffFrameFlag)
		{
			recode_macroblock = FALSE;
			rdopt = &rddata_top_frame_mb;   // store data in top frame MB 
			//初始化相关变量
			start_macroblock(CurrentMbAddr, FALSE);

			//if (img->current_mb_nr == 0)
			//{
			//	printf("\n 第%d帧 \n", img->number);
			//	for (int i = 0; i < 16; i++)
			//	{
			//		for (int j = 0; j < 16; j++)
			//		{
			//			printf("%d ", (int)(imgUV_org[0][i][j] - 0));
			//		}
			//		printf("\n");
			//	}
			//}

			//编码一个宏块
			encode_one_macroblock();

			/*if (img->current_mb_nr == 0)
			{
				printf("\n ------------------------第%d帧 \n", img->number);
				for (int i = 0; i < 16; i++)
				{
					for (int j = 0; j < 16; j++)
					{
						printf("%d ", (int)(imgUV_org[0][i][j] - 0));
					}
					printf("\n");
				}
			}*/


#ifdef MY_SECRET_ENCODE
			if (Encode_EmbedCodeFlg)
			{
				if (SecretPosition < SecretBitNum)
				{
					char secretCh = *(SecretBinaryBitStream + SecretPosition);
					SecretPosition++;

					byte EnergySwitch = 0;	//能量块开关，如果能量分布满足密文则置1
					byte MarkSwitch = 0;	//标识位开关，如果有可写入标识，则置1


					/////////////////////////////////////////////////////////////////////
					//						    能量块计算							   //
					/////////////////////////////////////////////////////////////////////
					//计算左上和右下的4x4块的能量总和,以及右上和左下块的能量总和
					int RTEnergySum = 0;
					int LDEnergySum = 0;
					//计算四个4x4块的首个0前的位置并且将能量的绝对值累加
					int fPosition1_0 = 0;
					int fPosition1_1 = 0;
					int fPosition1_2 = 0;
					int fPosition1_3 = 0;
					int fArray1_0[16];
					int fArray1_1[16];
					int fArray1_2[16];
					int fArray1_3[16];

					int fPosition2_0 = 0;
					int fPosition2_1 = 0;
					int fPosition2_2 = 0;
					int fPosition2_3 = 0;
					int fArray2_0[16];
					int fArray2_1[16];
					int fArray2_2[16];
					int fArray2_3[16];

					for (int i = 0; i < 16; i++)
					{
						fArray1_0[i] = img->cofAC[1][0][0][i];
						fArray1_1[i] = img->cofAC[1][1][0][i];
						fArray1_2[i] = img->cofAC[1][2][0][i];
						fArray1_3[i] = img->cofAC[1][3][0][i];
						fArray2_0[i] = img->cofAC[2][0][0][i];
						fArray2_1[i] = img->cofAC[2][1][0][i];
						fArray2_2[i] = img->cofAC[2][2][0][i];
						fArray2_3[i] = img->cofAC[2][3][0][i];
					}
					fPosition1_0 = GetLastNonZeroPosition(fArray1_0, 16);
					fPosition1_1 = GetLastNonZeroPosition(fArray1_1, 16);
					fPosition1_2 = GetLastNonZeroPosition(fArray1_2, 16);
					fPosition1_3 = GetLastNonZeroPosition(fArray1_3, 16);
					fPosition2_0 = GetLastNonZeroPosition(fArray2_0, 16);
					fPosition2_1 = GetLastNonZeroPosition(fArray2_1, 16);
					fPosition2_2 = GetLastNonZeroPosition(fArray2_2, 16);
					fPosition2_3 = GetLastNonZeroPosition(fArray2_3, 16);

					for (int i = 0; i < 16; i++)
					{
						if (i <= fPosition1_0)
							RTEnergySum += abs(img->cofAC[1][0][0][i]);
						if (i <= fPosition1_3)
							RTEnergySum += abs(img->cofAC[1][3][0][i]);
						if (i <= fPosition1_1)
							RTEnergySum += abs(img->cofAC[1][1][0][i]);
						if (i<=fPosition1_2)
							RTEnergySum += abs(img->cofAC[1][2][0][i]);
						if (i <= fPosition2_0)
							LDEnergySum += abs(img->cofAC[2][0][0][i]);
						if (i <= fPosition2_3)
							LDEnergySum += abs(img->cofAC[2][3][0][i]);
						if (i <= fPosition2_1)
							LDEnergySum += abs(img->cofAC[2][1][0][i]);
						if (i <= fPosition2_2)
							LDEnergySum += abs(img->cofAC[2][2][0][i]);

					}

					//比较能量获得该位数据
					char curData = (RTEnergySum > LDEnergySum) ? '1' : '0';
					if (curData == secretCh)
						EnergySwitch = 1;

					/////////////////////////////////////////////////////////////////////
					//						    标识位搜索							   //
					/////////////////////////////////////////////////////////////////////
					int MarkB8, MarkB4;
					int(*tLevel)[16] = NULL;
					int sPosition = 0;
					for (int i = 0; i < 8; i++)
					{
						MarkB8 = SearchOrderForMark[i][0];
						MarkB4 = SearchOrderForMark[i][1];
						tLevel = img->cofAC[MarkB8][MarkB4][0];

						//寻找最后一个非零位存入sPosition
						sPosition = -1;
						for (int j = 0; j < 16; j++)
						{
							if (*(*tLevel + sPosition+1) != 0)
							{
								sPosition++;
							}
							else
								break;
						}
						if (sPosition == -1)	//说明该块为全零，则需要跳过
						{
							continue;
						}
						else     //该块有满足条件的嵌入位，则停止搜索，锁定该嵌入位
						{
							MarkSwitch = 1;		//将标识开关置为1
							break;
						}
					}

					if (MarkSwitch)
					{
						if (EnergySwitch)	//修改标识位
						{
							if ((*(*tLevel + sPosition)) % 2 == 0)
							{
								if ((*(*tLevel + sPosition)) > 0)
									*(*tLevel + sPosition) -= 1;
								else
									*(*tLevel + sPosition) += 1;
							}
#ifdef OUTPUT_EMBED_POSITION
							printf("在第%d帧的第%d个宏块中嵌入密文的第%d位, 标识位坐标: %d, %d \n:", img->number, CurrentMbAddr, SecretPosition, MarkB8, MarkB4);
#endif
						}
						else     //将标识位写入0并回退密文指针
						{
							if ((*(*tLevel + sPosition)) % 2 == 1)
							{
								*(*tLevel + sPosition) += 1;
							}
							else if ((*(*tLevel + sPosition)) % 2 == -1)
							{
								*(*tLevel + sPosition) -= 1;
							}
							SecretPosition--;
						}
					}
					else     //回退密文指针
					{
						SecretPosition--;
					}
					//if (secretCh == curData)	//说明数据与密文一致，可以使用该宏块，检测标识位是否可用
					//{
					//	//首先检测第4个4x4块是否可用
					//	int tLevel[16];
					//	for (int i = 0; i < 16; i++)
					//		tLevel[i] = img->cofAC[1][3][0][i];
					//	int sPosition = GetLastNonZeroPosition(tLevel, 16);
					//	if (sPosition != -1)	//说明该4x4块可用，则将标识位嵌入此块
					//	{
					//		if (tLevel[sPosition] % 2 == 0)
					//		{
					//			if (tLevel[sPosition] > 0)
					//				tLevel[sPosition] -= 1;
					//			else
					//				tLevel[sPosition] += 1;
					//		}
					//		//将修改后的level写回原数组中
					//		img->cofAC[1][3][0][sPosition] = tLevel[sPosition];
					//		//printf("在第%d帧的第%d个宏块中嵌入第%d位,密文潜在了第4个4x4块中\n", img->number, CurrentMbAddr, SecretPosition);
					//	}
					//	else    //第4块不可用，则检测第3块
					//	{
					//		for (int i = 0; i < 16; i++)
					//			tLevel[i] = img->cofAC[1][2][0][i];
					//		sPosition = 0;
					//		sPosition = GetLastNonZeroPosition(tLevel, 16);
					//		if (sPosition != -1)		//说明第3块可用，将标识嵌入此块
					//		{
					//			if (tLevel[sPosition] % 2 == 0)
					//			{
					//				if (tLevel[sPosition] > 0)
					//					tLevel[sPosition] -= 1;
					//				else
					//					tLevel[sPosition] += 1;
					//			}
					//			//将修改后的level写回原数组中
					//			img->cofAC[1][2][0][sPosition] = tLevel[sPosition];
					//			//printf("在第%d帧的第%d个宏块中嵌入第%d位,密文潜在了第3个4x4块中\n", img->number, CurrentMbAddr, SecretPosition);
					//		}
					//		else      //标识位不可用，将密文信息回退
					//		{
					//			SecretPosition--;
					//		}
					//	}
					//}
					//else
					//{
					//	//将标识位置0并回退密文信息
					//	int tLevel[16];
					//	for (int i = 0; i < 16; i++)
					//		tLevel[i] = img->cofAC[1][3][0][i];
					//	int sPosition = GetLastNonZeroPosition(tLevel, 16);
					//	if (sPosition != -1)	//说明该4x4块可用，则将标识位嵌入此块
					//	{
					//		if (tLevel[sPosition] % 2 == 1)
					//		{
					//			tLevel[sPosition] += 1;
					//		}
					//		else if (tLevel[sPosition] % 2 == -1)
					//		{
					//			tLevel[sPosition] -= 1;
					//		}
					//		//将修改后的level写回原数组中
					//		img->cofAC[1][3][0][sPosition] = tLevel[sPosition];
					//	}
					//	else    //第4块不可用，则检测第3块
					//	{
					//		for (int i = 0; i < 16; i++)
					//			tLevel[i] = img->cofAC[1][2][0][i];
					//		sPosition = 0;
					//		sPosition = GetLastNonZeroPosition(tLevel, 16);
					//		if (sPosition != -1)		//说明第3块可用，将标识嵌入此块
					//		{
					//			if (tLevel[sPosition] % 2 == 1)
					//			{
					//				tLevel[sPosition] += 1;
					//			}
					//			else if (tLevel[sPosition] % 2 == -1)
					//			{
					//				tLevel[sPosition] -= 1;
					//			}
					//			//将修改后的level写回原数组中
					//			img->cofAC[1][2][0][sPosition] = tLevel[sPosition];
					//		}
					//	}
					//	SecretPosition--;
					//}
						
				}
			}
			
#endif


			//生成宏块语法结构
			write_one_macroblock(1);
			//更新条带信息
			terminate_macroblock(&end_of_slice, &recode_macroblock);

			// printf ("encode_one_slice: mb %d,  slice %d,   bitbuf bytepos %d EOS %d\n", 
			//       img->current_mb_nr, img->current_slice_nr, 
			//       img->currentSlice->partArr[0].bitstream->byte_pos, end_of_slice);

			if (recode_macroblock == FALSE)       // The final processing of the macroblock has been done
			{
				CurrentMbAddr = FmoGetNextMBNr(CurrentMbAddr);
				if (CurrentMbAddr == -1)   // end of slice
				{
					// printf ("FMO End of Slice Group detected, current MBs %d, force end of slice\n", NumberOfCodedMBs+1);
					end_of_slice = TRUE;
				}
				NumberOfCodedMBs++;       // only here we are sure that the coded MB is actually included in the slice
				proceed2nextMacroblock(CurrentMbAddr);
			}
			else
			{
				//!Go back to the previous MB to recode it
				img->current_mb_nr = FmoGetPreviousMBNr(img->current_mb_nr);
				if (img->current_mb_nr == -1)   // The first MB of the slice group  is too big,
					// which means it's impossible to encode picture using current slice bits restriction
				{
					snprintf(errortext, ET_SIZE, "Error encoding first MB with spcified parameter, bits of current MB may be too big");
					error(errortext, 300);
				}
			}
		}
		else                      // TBD -- Addition of FMO
		{

			//! This following ugly code breaks slices, at least for a slice mode that accumulates a certain
			//! number of bits into one slice.  
			//! The suggested algorithm is as follows:
			//!
			//! SaveState (Bitstream, stats,  etc. etc.);
			//! BitsForThisMBPairInFrameMode = CodeMB (Upper, FRAME_MODE) + CodeMB (Lower, FRAME_MODE);
			//! DistortionForThisMBPairInFrameMode = CalculateDistortion(Upper) + CalculateDistortion (Lower);
			//! RestoreState();
			//! BitsForThisMBPairInFieldMode = CodeMB (Upper, FIELD_MODE) + CodeMB (Lower, FIELD_MODE);
			//! DistortionForThisMBPairInFrameMode = CalculateDistortion(Upper) + CalculateDistortion (Lower);
			//! FrameFieldMode = Decision (...)
			//! RestoreState()
			//! if (FrameFieldMode == FRAME) {
			//!   CodeMB (Upper, FRAME); CodeMB (Lower, FRAME);
			//! } else {
			//!   CodeMB (Upper FIELD); CodeMB (Lower, FIELD);
			//! }
			//!
			//! Open questions/issues:
			//!   1. CABAC/CA-VLC state:  It seems that the CABAC/CA_VLC states are changed during the
			//!      dummy encoding processes (for the R-D based selection), but that they are never
			//!      reset, once the selection is made.  I believe that this breaks the MB-adaptive
			//!      frame/field coding.  The necessary code for the state saves is readily available
			//!      in macroblock.c, start_macroblock() and terminate_macroblock() (this code needs
			//!      to be double checked that it works with CA-VLC as well
			//!   2. would it be an option to allocate Bitstreams with zero data in them (or copy the
			//!      already generated bitstream) for the "test coding"?  

			if (input->MbInterlace == ADAPTIVE_CODING)
			{
				// code MB pair as frame MB 
				recode_macroblock = FALSE;


				img->field_mode = 0;  // MB coded as frame
				img->top_field = 0;   // Set top field to 0

				//Rate control
				img->write_macroblock = 0;
				img->bot_MB = 0;

				start_macroblock(CurrentMbAddr, FALSE);

				rdopt = &rddata_top_frame_mb; // store data in top frame MB 
				encode_one_macroblock();     // code the MB as frame
				FrameRDCost = rdopt->min_rdcost;
				//***   Top MB coded as frame MB ***//

				//Rate control
				img->bot_MB = 1; //for Rate control

				// go to the bottom MB in the MB pair
				img->field_mode = 0;  // MB coded as frame  //GB

				start_macroblock(CurrentMbAddr + 1, FALSE);
				rdopt = &rddata_bot_frame_mb; // store data in top frame MB
				encode_one_macroblock();     // code the MB as frame
				FrameRDCost += rdopt->min_rdcost;

				//***   Bottom MB coded as frame MB ***//
			}

			if ((input->MbInterlace == ADAPTIVE_CODING) || (input->MbInterlace == FIELD_CODING))
			{
				//Rate control
				img->bot_MB = 0;

				// start coding the MB pair as a field MB pair
				img->field_mode = 1;  // MB coded as frame
				img->top_field = 1;   // Set top field to 1
				img->buf_cycle <<= 1;
				input->num_reference_frames <<= 1;
				img->num_ref_idx_l0_active <<= 1;
				img->num_ref_idx_l0_active += 1;
				start_macroblock(CurrentMbAddr, TRUE);

				rdopt = &rddata_top_field_mb; // store data in top frame MB 
				//        TopFieldIsSkipped = 0;        // set the top field MB skipped flag to 0
				encode_one_macroblock();     // code the MB as frame
				FieldRDCost = rdopt->min_rdcost;
				//***   Top MB coded as field MB ***//
				//Rate control
				img->bot_MB = 1;//for Rate control

				img->top_field = 0;   // Set top field to 0
				start_macroblock(CurrentMbAddr + 1, TRUE);
				rdopt = &rddata_bot_field_mb; // store data in top frame MB 
				encode_one_macroblock();     // code the MB as frame
				FieldRDCost += rdopt->min_rdcost;
				//***   Bottom MB coded as field MB ***//
			}

			//Rate control
			img->write_macroblock_frame = 0;  //Rate control


			// decide between frame/field MB pair
			if ((input->MbInterlace == ADAPTIVE_CODING) && (FrameRDCost < FieldRDCost))
			{
				img->field_mode = 0;
				img->buf_cycle >>= 1;
				input->num_reference_frames >>= 1;
				MBPairIsField = 0;
				img->num_ref_idx_l0_active -= 1;
				img->num_ref_idx_l0_active >>= 1;

				//Rate control
				img->write_macroblock_frame = 1;  //for Rate control
			}
			else
			{
				img->field_mode = 1;
				MBPairIsField = 1;
			}

			//Rate control
			img->write_macroblock = 1;//Rate control 

			if (MBPairIsField)
				img->top_field = 1;
			else
				img->top_field = 0;

			//Rate control
			img->bot_MB = 0;// for Rate control

			// go back to the Top MB in the MB pair
			start_macroblock(CurrentMbAddr, img->field_mode);

			rdopt = img->field_mode ? &rddata_top_field_mb : &rddata_top_frame_mb;
			copy_rdopt_data(0);  // copy the MB data for Top MB from the temp buffers
			write_one_macroblock(1);     // write the Top MB data to the bitstream
			NumberOfCodedMBs++;   // only here we are sure that the coded MB is actually included in the slice
			terminate_macroblock(&end_of_slice, &recode_macroblock);     // done coding the Top MB 
			proceed2nextMacroblock(CurrentMbAddr);        // Go to next macroblock

			//Rate control
			img->bot_MB = 1;//for Rate control
			// go to the Bottom MB in the MB pair
			img->top_field = 0;
			start_macroblock(CurrentMbAddr + 1, img->field_mode);

			rdopt = img->field_mode ? &rddata_bot_field_mb : &rddata_bot_frame_mb;
			copy_rdopt_data(1);  // copy the MB data for Bottom MB from the temp buffers

			write_one_macroblock(0);     // write the Bottom MB data to the bitstream
			NumberOfCodedMBs++;   // only here we are sure that the coded MB is actually included in the slice
			terminate_macroblock(&end_of_slice, &recode_macroblock);     // done coding the Top MB 
			proceed2nextMacroblock(CurrentMbAddr);        // Go to next macroblock

			if (MBPairIsField)    // if MB Pair was coded as field the buffer size variables back to frame mode
			{
				img->buf_cycle >>= 1;
				input->num_reference_frames >>= 1;
				img->num_ref_idx_l0_active -= 1;
				img->num_ref_idx_l0_active >>= 1;
			}
			img->field_mode = img->top_field = 0; // reset to frame mode


			// go to next MB pair, not next MB
			CurrentMbAddr = FmoGetNextMBNr(CurrentMbAddr);
			CurrentMbAddr = FmoGetNextMBNr(CurrentMbAddr);

			if (CurrentMbAddr == FmoGetLastCodedMBOfSliceGroup(FmoMB2SliceGroup(CurrentMbAddr)))
				end_of_slice = TRUE;        // just in case it does n't get set in terminate_macroblock  
		}
	}
	
	terminate_slice();
	return NumberOfCodedMBs;
}

#ifdef MY_SECRET_ENCODE
void ZigzagScanConvert(int* OneDimensionalArray, int** TwoDimensionalArray, int width, int direction)
{
	//通过公式求出和为 x-1 的位置共有 width - | width - x | ，即横纵坐标和为0的位置有1个，横纵坐标和为1的位置有2个
	//0是一维转二维，1是二维转一维
	int currTensor = 0;		//记录一维数组当前访问位置
	int currX = 0;
	int currY = 0;
	int RorDFlg = 0;		//向右或者向下的标识位，若为-1则向右，若为1则向下
	int totalNum = 0;
	int XDirection = 0;
	int YDirection = 0;
	for (int i = 0; i < 2 * width - 1; i++)		//将和为0一直遍历到和为2*width
	{
		totalNum = width - abs(width - i - 1);
		if (i % 2 == 0)		//调整方向，如果横纵坐标和为偶数前进方向是右上，横减纵加
		{
			XDirection = -1;
			YDirection = 1;
		}
		else     //为奇数前进方向是左下，横加纵减
		{
			XDirection = 1;
			YDirection = -1;
		}
		for (int j = 0; j < totalNum; j++)
		{
			if (direction == 1)
				//*(OneDimensionalArray + currTensor) = TwoDimensionalArray[currX][currY];
				*(OneDimensionalArray + currTensor) = *(*(TwoDimensionalArray + currX) + currY);
			else if (direction == 0)
				*(*(TwoDimensionalArray + currX) + currY) = *(OneDimensionalArray + currTensor);
			currTensor++;
			if (j != totalNum - 1)		//此方向未走完，则调整X Y坐标继续扫描
			{
				currX += XDirection;
				currY += YDirection;
			}
			else      //此方向已经扫描完，需要更换方向
			{
				//首先判断应该向下还是向右
				int mid = (int)(width * 2 - 1) / 2;
				if (i % 2 == 0)
					RorDFlg = -1;	//和为偶数向右
				else
					RorDFlg = 1;    //和为奇数向下
				if (i >= mid)
					RorDFlg *= -1;	//如果和比中值大，则翻转方向
				if (RorDFlg == -1)
				{
					currY++;
				}
				else
					currX++;
			}
		}
	}
}

int findNZeroPosition(int* ScanArray, int ArraySize, int currPosition)
{
	if (currPosition == ArraySize - 1)	//说明已经达到数组尾但是任未找到非零元
		return ArraySize;
	else if (ScanArray[currPosition + 1] != 0)
		return currPosition + 1;
	else
		return findNZeroPosition(ScanArray, ArraySize, currPosition + 1);
}
int** MallocTwoDimensionArray(int size_x, int size_y)
{
	int** tA = (int**)malloc(sizeof(int*)*size_x);
	for (int i = 0; i < size_x; i++)
	{
		*(tA + i) = malloc(sizeof(int)*size_y);
		for (int j = 0; j < size_y; j++)
		{
			*(*(tA + i) + j) = 0;
		}
	}
	return tA;
}
void FreeTwoDimensionArray(int** Array, int xSize)
{
	for (int i = 0; i < xSize; i++)
	{
		free(*(Array + i));
	}
	free(Array);
}
void ReverseFromLevelRun(int* zScan, int* Level, int* Run, int size)
{
	int zNum = 0;
	for (int i = 0, ScanPosition = 0; i < size; i++)
	{
		zNum = Run[i];
		for (int j = 0; j < zNum; j++)
		{
			if (ScanPosition + j < 16)
			{
				zScan[ScanPosition + j] = 0;
			}
			else
			{
				printf("ScanPosition越界");
			}
		}
		ScanPosition += zNum;
		if (ScanPosition < size)
		{
			zScan[ScanPosition] = Level[i];
			ScanPosition++;
		}
		else
		{
			printf("ScanPosition越界");
		}
		if (ScanPosition == size)
			break;
	}
}
void MyIndexConvert(int* fD, int* sD, int direction)
{
	//direction为1时将坐标模式转换为 block8 block4 level/run scan_pos模式
	//direction为0时将坐标模式转换为 行 列 level/run scan_pos模式
	if (direction)
	{
		int b8 = ((*fD) / 2) * 2 + ((*sD) / 2);
		int b4 = ((*fD) % 2) * 2 + ((*sD) % 2);
		*fD = b8;
		*sD = b4;
	}
	else
	{
		int rowNum = ((*fD) / 2) * 2 + ((*sD) / 2);
		int colNum = ((*fD) % 2) * 2 + ((*sD) % 2);
		*fD = rowNum;
		*sD = colNum;
	}
}
int GetLastNonZeroPosition(int* tarray, int size)
{
	int buff = -1;
	for (int i = 0; i < size; i++)
	{
		if (tarray[i] != 0)
			buff = i;
		else
			break;
	}
	return buff;
}
#endif
/*!
 ************************************************************************
 * \brief
 *    Initializes the parameters for a new slice and
 *     allocates the memory for the coded slice in the Picture structure
 *  \par Side effects:
 *      Adds slice/partition header symbols to the symbol buffer
 *      increments Picture->no_slices, allocates memory for the
 *      slice, sets img->currSlice
 ************************************************************************
 */
static void init_slice(int start_mb_addr)
{
	int i;
	Picture *currPic = img->currentPicture;
	Slice *curr_slice;
	DataPartition *dataPart;
	Bitstream *currStream;

	img->current_mb_nr = start_mb_addr;

	// Allocate new Slice in the current Picture, and set img->currentSlice
	assert(currPic != NULL);
	currPic->no_slices++;
	if (currPic->no_slices >= MAXSLICEPERPICTURE)
		error("Too many slices per picture, increase MAXLSICESPERPICTURE in global.h.", -1);
	currPic->slices[currPic->no_slices - 1] = malloc_slice();
	curr_slice = currPic->slices[currPic->no_slices - 1];
	img->currentSlice = curr_slice;

	curr_slice->picture_id = img->tr % 256;
	curr_slice->qp = img->qp;
	curr_slice->start_mb_nr = start_mb_addr;
	curr_slice->slice_too_big = dummy_slice_too_big;

	for (i = 0; i < curr_slice->max_part_nr; i++)
	{
		dataPart = &(curr_slice->partArr[i]);
		if (input->symbol_mode == UVLC)
			dataPart->writeSyntaxElement = writeSyntaxElement_UVLC;
		else
			dataPart->writeSyntaxElement = writeSyntaxElement_CABAC;

		currStream = dataPart->bitstream;
		currStream->bits_to_go = 8;
		currStream->byte_pos = 0;
		currStream->byte_buf = 0;
	}
}


/*!
 ************************************************************************
 * \brief
 *    Allocates a slice structure along with its dependentdata structures
 * \return
 *    Pointer to a Slice
 ************************************************************************
 */
static Slice *malloc_slice()
{
	int i;
	DataPartition *dataPart;
	Slice *slice;
	const int buffer_size = (img->width * img->height * 4); // AH 190202: There can be data expansion with 
	// low QP values. So, we make sure that buffer 
	// does not everflow. 4 is probably safe multiplier.

	if ((slice = (Slice *)calloc(1, sizeof(Slice))) == NULL) no_mem_exit("malloc_slice: slice structure");

	if (input->symbol_mode == CABAC)
	{
		// create all context models
		slice->mot_ctx = create_contexts_MotionInfo();
		slice->tex_ctx = create_contexts_TextureInfo();
	}

	slice->max_part_nr = input->partition_mode == 0 ? 1 : 3;

	//for IDR img there should be only one partition
	if (img->currentPicture->idr_flag)
		slice->max_part_nr = 1;

	assignSE2partition[0] = assignSE2partition_NoDP;
	//ZL 
	//for IDR img all the syntax element shoulde be mapped to one partition        
	if (!img->currentPicture->idr_flag&&input->partition_mode == 1)
		assignSE2partition[1] = assignSE2partition_DP;
	else
		assignSE2partition[1] = assignSE2partition_NoDP;



	slice->num_mb = 0;          // no coded MBs so far

	if ((slice->partArr = (DataPartition *)calloc(slice->max_part_nr, sizeof(DataPartition))) == NULL) no_mem_exit("malloc_slice: partArr");
	for (i = 0; i < slice->max_part_nr; i++) // loop over all data partitions
	{
		dataPart = &(slice->partArr[i]);
		if ((dataPart->bitstream = (Bitstream *)calloc(1, sizeof(Bitstream))) == NULL) no_mem_exit("malloc_slice: Bitstream");
		if ((dataPart->bitstream->streamBuffer = (byte *)calloc(buffer_size, sizeof(byte))) == NULL) no_mem_exit("malloc_slice: StreamBuffer");
		// Initialize storage of bitstream parameters
	}
	return slice;
}


/*!
 ************************************************************************
 * \brief
 *    Memory frees of all Slice structures and of its dependent
 *    data structures
 * \par Input:
 *    Image Parameters struct struct img_par *img
 ************************************************************************
 */
void free_slice_list(Picture *currPic)
{
	int i;

	for (i = 0; i < currPic->no_slices; i++)
	{
		free_slice(currPic->slices[i]);
		currPic->slices[i] = NULL;
	}
}


/*!
 ************************************************************************
 * \brief
 *    Memory frees of the Slice structure and of its dependent
 *    data structures
 * \param slice:
 *    Slice to be freed
 ************************************************************************
 */
static void free_slice(Slice *slice)
{
	int i;
	DataPartition *dataPart;

	if (slice != NULL)
	{
		for (i = 0; i < slice->max_part_nr; i++) // loop over all data partitions
		{
			dataPart = &(slice->partArr[i]);
			if (dataPart != NULL)
			{
				if (dataPart->bitstream->streamBuffer != NULL)
					free(dataPart->bitstream->streamBuffer);
				if (dataPart->bitstream != NULL)
					free(dataPart->bitstream);
			}
		}
		if (slice->partArr != NULL)
			free(slice->partArr);
		if (input->symbol_mode == CABAC)
		{
			delete_contexts_MotionInfo(slice->mot_ctx);
			delete_contexts_TextureInfo(slice->tex_ctx);
		}
		//free(img->currentSlice);
		free(slice);
	}
}


/*!
 ************************************************************************
 * \brief
 *    This function set the value of a bit in a bitstream to 1
 ************************************************************************
 */
void modify_redundant_pic_cnt(unsigned char *buffer)
{
	unsigned char tmp = 1 << (rpc_bits_to_go - 1);
	buffer[rpc_bytes_to_go] |= tmp;
}

void set_ref_pic_num()
{
	int i, j;

	//! need to add field ref_pic_num that handles field pair.

	for (i = 0; i < listXsize[LIST_0]; i++)
	{
		enc_picture->ref_pic_num[LIST_0][i] = listX[LIST_0][i]->poc * 2 + ((listX[LIST_0][i]->structure == BOTTOM_FIELD) ? 1 : 0);
		enc_picture->frm_ref_pic_num[LIST_0][i] = listX[LIST_0][i]->frame_poc * 2;
		enc_picture->top_ref_pic_num[LIST_0][i] = listX[LIST_0][i]->top_poc * 2;
		enc_picture->bottom_ref_pic_num[LIST_0][i] = listX[LIST_0][i]->bottom_poc * 2 + 1;
	}

	for (i = 0; i < listXsize[LIST_1]; i++)
	{
		enc_picture->ref_pic_num[LIST_1][i] = listX[LIST_1][i]->poc * 2 + ((listX[LIST_1][i]->structure == BOTTOM_FIELD) ? 1 : 0);
		enc_picture->frm_ref_pic_num[LIST_1][i] = listX[LIST_1][i]->frame_poc * 2;
		enc_picture->top_ref_pic_num[LIST_1][i] = listX[LIST_1][i]->top_poc * 2;
		enc_picture->bottom_ref_pic_num[LIST_1][i] = listX[LIST_1][i]->bottom_poc * 2 + 1;
	}

	if (img->structure == FRAME)
		for (j = 2; j < 6; j++)
			for (i = 0; i < listXsize[j]; i++)
			{
				enc_picture->ref_pic_num[j][i] = listX[j][i]->poc * 2 + ((listX[j][i]->structure == BOTTOM_FIELD) ? 1 : 0);
				enc_picture->frm_ref_pic_num[j][i] = listX[j][i]->frame_poc * 2;
				enc_picture->top_ref_pic_num[j][i] = listX[j][i]->top_poc * 2;
				enc_picture->bottom_ref_pic_num[j][i] = listX[j][i]->bottom_poc * 2 + 1;
			}

}
