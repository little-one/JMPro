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
 * \file mb_access.c
 *
 * \brief
 *    Functions for macroblock neighborhoods
 *�Ѷ�
 *  \author
 *      Main contributors (see contributors.h for copyright, address and affiliation details)
 *      - Karsten S�hring          <suehring@hhi.de>
 *************************************************************************************
 */

#include "global.h"

/*!
 ************************************************************************
 * \brief
 *    returns 1 if the macroblock at the given address is available
 *    ������mbAddr�Ƿ����
 ************************************************************************
 */
int mb_is_available(int mbAddr, int currMbAddr)
{
  if ((mbAddr < 0) || (mbAddr > ((int)img->PicSizeInMbs - 1)))
    return 0;

  // the following line checks both: slice number and if the mb has been decoded
  //�жϵ�ǰ֡�Ƿ�����deblocking
  if (!img->DeblockCall)
  { //�ж�mbAddr��currMbAddr�Ƿ�����һ��slice
    if (img->mb_data[mbAddr].slice_nr != img->mb_data[currMbAddr].slice_nr)
      return 0;
  }  
  return 1;
}


/*!
 ************************************************************************
 * \brief
 *    Checks the availability of neighboring macroblocks of
 *    the current macroblock for prediction and context determination;
 *    �ú�����start_macroblock��deblockmb�е��ã���ÿ��encode_one_macroblock����deblockִ��֮ǰ��
 *    ��Ҫ�����Χ���Ŀ�����(A,B,C,D)
 ************************************************************************
 */
 /*
 *     MBaddrD    MBaddrB    MBaddrC
 *
 *
 *     MBaddrA    MBaddrCur
 *
*/
void CheckAvailabilityOfNeighbors()
{
  const int mb_nr = img->current_mb_nr;
  Macroblock *currMB = &img->mb_data[mb_nr];

  // mark all neighbors as unavailable
  currMB->mb_available_up   = NULL;
  currMB->mb_available_left = NULL;

  if (img->MbaffFrameFlag)
  {
    currMB->mbAddrA = 2 * (mb_nr/2 - 1);//��ߺ��Ե�top���
    currMB->mbAddrB = 2 * (mb_nr/2 - img->PicWidthInMbs);//�ϱߺ��Ե�top���
    currMB->mbAddrC = 2 * (mb_nr/2 - img->PicWidthInMbs + 1); //���Ϻ��Ե�top���
    currMB->mbAddrD = 2 * (mb_nr/2 - img->PicWidthInMbs - 1);//���Ϻ��Ե�top���
    //mbAddrA�Ƿ�����ҵ�ǰ�鲻�������һ��
    currMB->mbAvailA = mb_is_available(currMB->mbAddrA, mb_nr) && (((mb_nr/2) % img->PicWidthInMbs)!=0);
    //mbAddrB�Ƿ����
    currMB->mbAvailB = mb_is_available(currMB->mbAddrB, mb_nr);
    //mbAddrC�Ƿ�����ҵ�ǰ�鲻�����ұ�һ��
    currMB->mbAvailC = mb_is_available(currMB->mbAddrC, mb_nr) && (((mb_nr/2 +1) % img->PicWidthInMbs)!=0);
    //mbAddrD�Ƿ�����ҵ�ǰ�鲻�������һ��
    currMB->mbAvailD = mb_is_available(currMB->mbAddrD, mb_nr) && (((mb_nr/2) % img->PicWidthInMbs)!=0);
  }
  else
  {
    currMB->mbAddrA = mb_nr - 1; //��ߺ��
    currMB->mbAddrB = mb_nr - img->PicWidthInMbs; //�ϱߺ��
    currMB->mbAddrC = mb_nr - img->PicWidthInMbs + 1; //���Ϻ��
    currMB->mbAddrD = mb_nr - img->PicWidthInMbs - 1; //���Ϻ��
    //mbAddrA�Ƿ�����ҵ�ǰ�鲻�������һ��
    currMB->mbAvailA = mb_is_available(currMB->mbAddrA, mb_nr) && ((mb_nr % img->PicWidthInMbs)!=0);
    //mbAddrB�Ƿ����
    currMB->mbAvailB = mb_is_available(currMB->mbAddrB, mb_nr);
    //mbAddrC�Ƿ�����ҵ�ǰ�鲻�����ұ�һ��
    currMB->mbAvailC = mb_is_available(currMB->mbAddrC, mb_nr) && (((mb_nr+1) % img->PicWidthInMbs)!=0);
    //mbAddrD�Ƿ�����ҵ�ǰ�鲻�������һ��
    currMB->mbAvailD = mb_is_available(currMB->mbAddrD, mb_nr) && ((mb_nr % img->PicWidthInMbs)!=0);
  }

  if (currMB->mbAvailA) currMB->mb_available_left = &(img->mb_data[currMB->mbAddrA]);
  if (currMB->mbAvailB) currMB->mb_available_up   = &(img->mb_data[currMB->mbAddrB]);
}


/*!
 ************************************************************************
 * \brief
 *    returns the x and y macroblock coordinates for a given MbAddress
 *    �õ���ַΪmb_addr�ĺ��ĺ������
 ************************************************************************
 */
void get_mb_block_pos (int mb_addr, int *x, int*y)
{
  //PicWidthInMbs ���Ժ��Ϊ��λ��ͼ����
  if (img->MbaffFrameFlag) 
  {
    *x = ((mb_addr/2) % img->PicWidthInMbs); //�õ���MBΪ��λ��λ��MBAFF
    *y = ( ((mb_addr/2) / img->PicWidthInMbs)  * 2 + (mb_addr%2));
  }
  else 
  {
    *x = (mb_addr % img->PicWidthInMbs);//�õ���MBΪ��λ��λ��non-MBAFF
    *y = (mb_addr / img->PicWidthInMbs);
  }
}

/*!
 ************************************************************************
 * \brief
 *    returns the x and y sample coordinates for a given MbAddress
 *    �õ���ַΪmb_addr�ĺ��ľ��Ե�ַ(��������)
 ************************************************************************
 */
void get_mb_pos (int mb_addr, int *x, int*y)
{
  get_mb_block_pos(mb_addr, x, y);
  
  (*x) *= MB_BLOCK_SIZE;//�õ�����Ϊmb_addr�ĺ��ľ��Ե�ַ
  (*y) *= MB_BLOCK_SIZE;
}

/*!
 ************************************************************************
 * \brief
 * �õ�Non-MBAFFʱ�������ص�ľ��������������������ڵ��������
 *    get neighbouring positions for non-aff coding
 * \param curr_mb_nr
 *   current macroblock number (decoding order)
 * \param xN
 *    input x position
 * \param yN
 *    input y position
 * \param luma
 *    1 if luma coding, 0 for chroma
 * \param pix
 *    returns position informations
 ************************************************************************
 */
void getNonAffNeighbour(unsigned int curr_mb_nr, int xN, int yN, int luma, PixelPos *pix)
{
  Macroblock *currMb = &img->mb_data[curr_mb_nr];
  int maxWH;

  if (luma)
    maxWH = 16;
  else
    maxWH = 8;

  //mbAddrX��mbAvailX������CheckAvailabilityOfNeighbors����õ���
  //��������(xN,yN)�����ص�Ŀ�����
  if ((xN<0)&&(yN<0))
  {
    pix->mb_addr   = currMb->mbAddrD;
    pix->available = currMb->mbAvailD;
  }
  else
  if ((xN<0)&&((yN>=0)&&(yN<maxWH)))
  {
    pix->mb_addr  = currMb->mbAddrA;
    pix->available = currMb->mbAvailA;
  }
  else
  if (((xN>=0)&&(xN<maxWH))&&(yN<0))
  {
    pix->mb_addr  = currMb->mbAddrB;
    pix->available = currMb->mbAvailB;
  }
  else
  if (((xN>=0)&&(xN<maxWH))&&((yN>=0)&&(yN<maxWH)))
  {
    pix->mb_addr  = curr_mb_nr;
    pix->available = 1;
  }
  else
  if ((xN>=maxWH)&&(yN<0))
  {
    pix->mb_addr  = currMb->mbAddrC;
    pix->available = currMb->mbAvailC;
  }
  else 
  {
    pix->available = 0;
  }
  //������ڿ���û��ߵ�ǰͼ��������deblocking
  if (pix->available || img->DeblockCall)
  {
    pix->x = (xN + maxWH) % maxWH;   //�õ��������ں���ڵ��������
    pix->y = (yN + maxWH) % maxWH; 
    get_mb_pos(pix->mb_addr, &(pix->pos_x), &(pix->pos_y)); //�õ����ڿ��������ĺ������
    if (luma)
    {
      pix->pos_x += pix->x; //�õ������������ؾ�������
      pix->pos_y += pix->y;
    }
    else
    {
      pix->pos_x = (pix->pos_x/2) + pix->x;  //�õ�ɫ���������ؾ�������
      pix->pos_y = (pix->pos_y/2) + pix->y;
    }
  }
}

/*!
 ************************************************************************
 * \brief
 * �õ�MBAFFʱ�������ص�ľ��������������������ڵ��������
 * �Ķ�ǰ��֪:
 * 1���������������Ŀ�������һ�µ� 
 *    get neighbouring positions for aff coding
 * \param curr_mb_nr
 *   current macroblock number (decoding order)
 * \param xN
 *    input x position
 * \param yN
 *    input y position
 * \param luma
 *    1 if luma coding, 0 for chroma
 * \param pix
 *    returns position informations
 ************************************************************************
 */
void getAffNeighbour(unsigned int curr_mb_nr, int xN, int yN, int luma, PixelPos *pix)
{
  Macroblock *currMb = &img->mb_data[curr_mb_nr];
  int maxWH;
  int yM = -1;   //MbAffʱ�仯����y�����ϣ�x����ûʲô����

  if (luma)
    maxWH = 16;   //���Ⱥ��߳�
  else
    maxWH = 8;     //ɫ�Ⱥ��߳�

  // initialize to "not available"
  pix->available = 0;

  if(yN > (maxWH - 1))  //��ǰ����·�
  {
    return;
  }
  if ((xN > (maxWH -1)) && ((yN >= 0)&&(yN < (maxWH )))) //��ǰ����ұ�
  {
    return;
  }

  if (xN < 0)  //���
  {
    if (yN < 0)  //���ϣ�yN<0��ζ��yN����-1
    {
      if(!currMb->mb_field)  //��ǰ�����֡��飬��ǰ������֡����
      {
        // frame
        if (curr_mb_nr%2 == 0)  //��ǰ�����top���
        {
          // top
          pix->mb_addr  = currMb->mbAddrD  + 1;  //ȡmbAddrD�ĵ׺����Ϊ���ں��
          pix->available = currMb->mbAvailD;
           yM      = yN;  //����mbAddrD��֡/�����ԣ�������yN
        }
        else    //��ǰ�����bottom���
        {
          // bottom
          pix->mb_addr  = currMb->mbAddrA;   //ȡmbAddrA�Ķ������Ϊ���ں��
          pix->available = currMb->mbAvailA; 
          if (currMb->mbAvailA)   //���mbAddrA������
          {
            if(!img->mb_data[currMb->mbAddrA].mb_field) //�Ҳ�Ϊ�����
            {
               yM = yN;   //��������������ȡyN����
            }
            else   //�����ٿ�Ϊ�����(��ǰ��Ϊ֡���)
            {
              (pix->mb_addr)++;   //ȡmbAddrA�ĵ׳������Ϊ���ں��!
               yM = (yN + maxWH) >> 1; //(-1+16)/2=7,������mbAddrA�ϰ벿������ĳ������
            }
          }
        }
      }
      else   //��ǰ����ǳ���飬��ǰ�����ǳ�����
      {
        // field
        if(curr_mb_nr % 2 == 0)  //��ǰ����Ƕ������
        {
          // top
          pix->mb_addr  = currMb->mbAddrD; //ȡmbAddrD��top�����Ϊ���ں��
          pix->available = currMb->mbAvailD;
          if (currMb->mbAvailD)     //���mbAddrD������
          {
            if(!img->mb_data[currMb->mbAddrD].mb_field) //mbAddrDΪ֡����
            {
              (pix->mb_addr)++;  //ȡ��֡�����Ϊ���ں��
               yM = 2 * yN;             //��������������yMΪ��֡��鵹���ڶ���
            }
            else   //mbAddrDΪ������
            {
               yM = yN;   //��ȡ���������Ϊ���ں�飬yMȡ����������һ��
            }   //�����ǵ�֡��鵹���ڶ��л��Ƕ���������һ�ж��Ǻ��Ե����ڶ���
          }
        }
        else   //��ǰ����ǵ׳����
        {
          // bottom
          pix->mb_addr  = currMb->mbAddrD+1;  //ȡbottom�����Ϊ���ں��,����֡/��
          pix->available = currMb->mbAvailD;
           yM      = yN; //����֡/����yM��ȡ�������һ��
        }
      }
    }
    else
    { // xN < 0 && yN >= 0
      if ((yN >= 0) && (yN <maxWH))  //���
      {
        if (!currMb->mb_field)   //��ǰ����֡���
        {
          // frame
          if(curr_mb_nr % 2 == 0)  //top���
          {
            // top
            pix->mb_addr  = currMb->mbAddrA; //ȡmbAddrA��top�����Ϊ���ں��
            pix->available = currMb->mbAvailA;
            if (currMb->mbAvailA)  //����
            {
              if(!img->mb_data[currMb->mbAddrA].mb_field) //mbAddrA��֡���(��)
              {
                 yM = yN;
              }
              else    //mbAddrA�ǳ����(��)
              {
                if (yN %2 == 0)  //yN���ڶ���
                {
                   yM = yN>> 1;
                }
                else
                {
                  (pix->mb_addr)++;  //yN���ڵ׳�,����������yMȡ�׳����
                   yM = yN>> 1;
                }
              }
            }
          }
          else   //bottom���
          {
            // bottom
            pix->mb_addr  = currMb->mbAddrA;
            pix->available = currMb->mbAvailA;
            if (currMb->mbAvailA)
            {
              if(!img->mb_data[currMb->mbAddrA].mb_field) //mbAddrA��֡���(��)
              {
                (pix->mb_addr)++;  //ȡ��֡���
                 yM = yN;
              }
              else     //mbAddrA�ǳ����(��)
              {
                if (yN %2 == 0)  //yN���ڶ���
                {
                   yM = (yN + maxWH) >> 1;
                }
                else
                {
                  (pix->mb_addr)++;          //yN���ڵ׳�,����������yMȡ�׳����
                   yM = (yN + maxWH) >> 1;
                }
              }
            }
          }
        }
        else   //��ǰ���ǳ����
        {
          // field
          if (curr_mb_nr % 2 == 0)  //��ǰ���Ƕ������
          {
            // top
            pix->mb_addr  = currMb->mbAddrA;   //�ȸ�ֵΪ��֡(��)���
            pix->available = currMb->mbAvailA;
            if (currMb->mbAvailA)
            {
              if(!img->mb_data[currMb->mbAddrA].mb_field)   //mbAddrA��֡���(��)
              {
                if (yN < (maxWH / 2))  //yN���ڶ��������ϰ벿��
                {
                   yM = yN << 1;  //����������yM����mbAddrA�Ķ�֡���
                }
                else            //yN���ڶ��������°벿��
                {
                  (pix->mb_addr)++;            //����������yM����mbAddrA�ĵ�֡���
                   yM = (yN << 1 ) - maxWH;  //����yM�������������λ��
                }
              }
              else     //mbAddrA�ǳ����(��)
              {
                 yM = yN;  //�����Զ�������򵥵����
              }
            }
          }
          else    //��ǰ���ǵ׳����
          {
            // bottom
            pix->mb_addr  = currMb->mbAddrA;  
            pix->available = currMb->mbAvailA;
            if (currMb->mbAvailA)
            {
              if(!img->mb_data[currMb->mbAddrA].mb_field)  //mbAddrA��֡���(��)
              {
                if (yN < (maxWH / 2)) //yN���ڵ׳������ϰ벿��
                {
                   yM = (yN << 1) + 1;   //���������ƣ���Ϊ�ǵ׳����Լ�1
                }
                else   //�������������
                {
                  (pix->mb_addr)++;
                   yM = (yN << 1 ) + 1 - maxWH; 
                }
              }
              else                //mbAddrA�ǳ����(��)
              {
                 (pix->mb_addr)++;   //�����������������Ϊ�׳�
                 yM = yN;      //�׳��Ե׳���������
              }
            }
          }
        }
      }
    }
  }
  else
  {
    // xN >= 0
    if ((xN >= 0)&&(xN <maxWH))  
    {
      if (yN<0)   //�Ϸ�
      {
        if (!currMb->mb_field)  //��ǰ����֡���
        {
          //frame
          if (curr_mb_nr % 2 == 0)   //��ǰ���Ƕ�֡���
          {
            //top
            pix->mb_addr  = currMb->mbAddrB;
            // for the deblocker if the current MB is a frame and the one above is a field
            // then the neighbor is the top MB of the pair
            if (currMb->mbAvailB)
            {
              if (!(img->DeblockCall == 1 && (img->mb_data[currMb->mbAddrB]).mb_field))
                pix->mb_addr  += 1; //�������(������Deblocking����mbAddrB�ǳ����)���ڿ�Ϊ�׺��
            }

            pix->available = currMb->mbAvailB;
             yM      = yN;
          }
          else   //��ǰ���ǵ�֡���
          {
            // bottom
            pix->mb_addr  = curr_mb_nr - 1;  //���ڿ��Ƕ�֡���
            pix->available = 1; //��Ȼ���ã���Ϊ�뵱ǰ������һ�����Եģ�������һ��
             yM      = yN;
          }
        }
        else    //��ǰ���ǳ����
        {
          // field
          if (curr_mb_nr % 2 == 0)  //����
          {
            // top
            pix->mb_addr  = currMb->mbAddrB;
            pix->available = currMb->mbAvailB;
            if (currMb->mbAvailB)
            {
              if(!img->mb_data[currMb->mbAddrB].mb_field) //mbAddrB��֡���(��)
              {
                (pix->mb_addr)++;   //ȡmbAddrB���Եĵ�֡���
                 yM = 2* yN;   //##  �൱�ڰ�mbAddrB��ת�ɳ������ȡ������Ӧλ��
              }
              else
              {
                 yM = yN;  //�������ע##�������ָʾ��λ����ʵ��һ����!!
              }
            }
          }
          else      //�׳�
          {
            // bottom
            pix->mb_addr  = currMb->mbAddrB + 1;   //����mbAddrB��֡or����ȡ�׺��
            pix->available = currMb->mbAvailB;
             yM      = yN; //�൱�ڰ�mbAddrB��ת�ɳ������ȡ�׳���Ӧλ��
          }
        }
      }
      else
      {
        // yN >=0
        // for the deblocker if this is the extra edge then do this special stuff
        //DeblockCall==2���ĸ�����:
        // 1����ˮƽ�߽磬2����ͼ��߽磬3��MbQ�ǳ������룬4���ٿ���֡������
        if (yN == 0 && img->DeblockCall == 2)
        {
          pix->mb_addr  = currMb->mbAddrB + 1;  //ȡmbAddrB�ĵ�֡���
          pix->available = 1;
           yM      = yN - 1;
        }

        else if ((yN >= 0) && (yN <maxWH))
        {
          pix->mb_addr  = curr_mb_nr;
          pix->available = 1;
           yM      = yN;
        }
      }
    }
    else
    {
      // xN >= maxWH
      if(yN < 0)
      {
        if (!currMb->mb_field)
        {
          // frame
          if (curr_mb_nr % 2 == 0)
          {
            // top
            pix->mb_addr  = currMb->mbAddrC + 1;
            pix->available = currMb->mbAvailC;
             yM      = yN;
          }
          else
          {
            // bottom
            pix->available = 0;
          }
        }
        else
        {
          // field
          if (curr_mb_nr % 2 == 0)
          {
            // top
            pix->mb_addr  = currMb->mbAddrC;
            pix->available = currMb->mbAvailC;
            if (currMb->mbAvailC)
            {
              if(!img->mb_data[currMb->mbAddrC].mb_field)
              {
                (pix->mb_addr)++;
                 yM = 2* yN;
              }
              else
              {
                 yM = yN;
              }
            }
          }
          else
          {
            // bottom
            pix->mb_addr  = currMb->mbAddrC + 1;
            pix->available = currMb->mbAvailC;
             yM      = yN;
          }
        }
      }
    }
  }
  if (pix->available || img->DeblockCall)  //�ù��̸�NonAff�е�һ��,��NonAff����
  {
    pix->x = (xN + maxWH) % maxWH;
    pix->y = (yM + maxWH) % maxWH;
    get_mb_pos(pix->mb_addr, &(pix->pos_x), &(pix->pos_y));
    if (luma)
    {
      pix->pos_x += pix->x;
      pix->pos_y += pix->y;
    }
    else
    {
      pix->pos_x = (pix->pos_x/2) + pix->x;
      pix->pos_y = (pix->pos_y/2) + pix->y;
    }
  }
}


/*!
 ************************************************************************
 * \brief
 *    get neighbouring positions. MB AFF is automatically used from img structure
 *    �õ��������ص�ľ��������������������ڵ�������꣨AFF��NonAFF���������
 * \param curr_mb_nr
 *   current macroblock number (decoding order)
 * \param xN
 *    input x position
 * \param yN
 *    input y position
 * \param luma
 *    1 if luma coding, 0 for chroma
 * \param pix
 *    returns position informations
 ************************************************************************
 */
void getNeighbour(int curr_mb_nr, int xN, int yN, int luma, PixelPos *pix)
{
  if (curr_mb_nr<0) ;
    //error ("getNeighbour: invalid macroblock number", 100);

  if (img->MbaffFrameFlag)
    getAffNeighbour(curr_mb_nr, xN, yN, luma, pix);  //�������������һ���ģ�������AFF
  else
    getNonAffNeighbour(curr_mb_nr, xN, yN, luma, pix); //�õ��������ؾ�������ͺ�����������pix
}


/*!
 ************************************************************************
 * \brief
 *    get neighbouring  get neighbouring 4x4 luma block
 * \param curr_mb_nr
 *   current macroblock number (decoding order)
 * \param block_x
 *    input x block position
 * \param block_y
 *    input y block position
 * \param rel_x
 *    relative x position of neighbor
 * \param rel_y
 *    relative y position of neighbor
 * \param pix
 *    returns position informations
 ************************************************************************
 */
void getLuma4x4Neighbour (int curr_mb_nr, int block_x, 
                          int block_y, int rel_x, int rel_y, PixelPos *pix)
{
  int x = 4* block_x + rel_x;  //block_x, block_y��MB�����4x4������
  int y = 4* block_y + rel_y;  //������õ��������ص���Ե�ǰ���ԭ�������

  getNeighbour(curr_mb_nr, x, y, 1, pix);//����������ص��������Ŀ�����

  if (pix->available)
  {
    pix->x /= 4; //pix->x pix->y���������غ��������
    pix->y /= 4; //������õ���������������4x4С��ĺ�����������
    pix->pos_x /= 4; //pix->pos_x pix->pos_y���������صľ�������
    pix->pos_y /= 4; //���ԣ�������õ���������������4x4С��ľ�������
  }
}

/*!
 ************************************************************************
 * \brief
 *    get neighbouring 4x4 chroma block
 * \param curr_mb_nr
 *   current macroblock number (decoding order)
 * \param block_x
 *    input x block position
 * \param block_y
 *    input y block position
 * \param rel_x
 *    relative x position of neighbor
 * \param rel_y
 *    relative y position of neighbor
 * \param pix
 *    returns position informations
 ************************************************************************
 */
void getChroma4x4Neighbour (int curr_mb_nr, int block_x, int block_y, int rel_x, int rel_y, PixelPos *pix)
{
  int x = 4* block_x + rel_x;
  int y = 4* block_y + rel_y;

  getNeighbour(curr_mb_nr, x, y, 0, pix);

  if (pix->available)
  {
    pix->x /= 4;
    pix->y /= 4;
    pix->pos_x /= 4;
    pix->pos_y /= 4;
  }
}
