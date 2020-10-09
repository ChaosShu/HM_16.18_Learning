/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TComPrediction.cpp
    \brief    prediction class
*/

#include <memory.h>
#include "TComPrediction.h"
#include "TComPic.h"
#include "TComTU.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

const UChar TComPrediction::m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS] =
{
  { // Luma
    10, //4x4
    7, //8x8
    1, //16x16
    0, //32x32
    10, //64x64
  },
  { // Chroma
    10, //4xn
    7, //8xn
    1, //16xn
    0, //32xn
    10, //64xn
  }

};

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

TComPrediction::TComPrediction()
: m_pLumaRecBuffer(0)
, m_iLumaRecStride(0)
{
  for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    for(UInt buf=0; buf<2; buf++)
    {
      m_piYuvExt[ch][buf] = NULL;
    }
  }
}

TComPrediction::~TComPrediction()
{
  destroy();
}

Void TComPrediction::destroy()
{
  for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    for(UInt buf=0; buf<NUM_PRED_BUF; buf++)
    {
      delete [] m_piYuvExt[ch][buf];
      m_piYuvExt[ch][buf] = NULL;
    }
  }

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_acYuvPred[i].destroy();
  }

  m_cYuvPredTemp.destroy();

  if( m_pLumaRecBuffer )
  {
    delete [] m_pLumaRecBuffer;
    m_pLumaRecBuffer = 0;
  }
  m_iLumaRecStride = 0;

  for (UInt i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++)
  {
    for (UInt j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++)
    {
      m_filteredBlock[i][j].destroy();
    }
    m_filteredBlockTmp[i].destroy();
  }
}

Void TComPrediction::initTempBuff(ChromaFormat chromaFormatIDC)
{
  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if( m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] != NULL && m_cYuvPredTemp.getChromaFormat()!=chromaFormatIDC)
  {
    destroy();
  }

  if( m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] == NULL ) // check if first is null (in which case, nothing initialised yet)
  {
    Int extWidth  = MAX_CU_SIZE + 16;
    Int extHeight = MAX_CU_SIZE + 1;

    for (UInt i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++)
    {
      m_filteredBlockTmp[i].create(extWidth, extHeight + 7, chromaFormatIDC);
      for (UInt j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++)
      {
        m_filteredBlock[i][j].create(extWidth, extHeight, chromaFormatIDC);
      }
    }

    m_iYuvExtSize = (MAX_CU_SIZE*2+1) * (MAX_CU_SIZE*2+1);
    for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
    {
      for(UInt buf=0; buf<NUM_PRED_BUF; buf++)
      {
        m_piYuvExt[ch][buf] = new Pel[ m_iYuvExtSize ];
      }
    }

    // new structure
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_acYuvPred[i] .create( MAX_CU_SIZE, MAX_CU_SIZE, chromaFormatIDC );
    }

    m_cYuvPredTemp.create( MAX_CU_SIZE, MAX_CU_SIZE, chromaFormatIDC );
  }


  if (m_iLumaRecStride != (MAX_CU_SIZE>>1) + 1)
  {
    m_iLumaRecStride =  (MAX_CU_SIZE>>1) + 1;
    if (!m_pLumaRecBuffer)
    {
      m_pLumaRecBuffer = new Pel[ m_iLumaRecStride * m_iLumaRecStride ];
    }
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
//NOTE: Bit-Limit - 25-bit source
Pel TComPrediction::predIntraGetPredValDC( const Pel* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight)
{
  assert(iWidth > 0 && iHeight > 0);
  Int iInd, iSum = 0;
  Pel pDcVal;

  for (iInd = 0;iInd < iWidth;iInd++)
  {
    iSum += pSrc[iInd-iSrcStride];
  }
  for (iInd = 0;iInd < iHeight;iInd++)
  {
    iSum += pSrc[iInd*iSrcStride-1];
  }

  pDcVal = (iSum + iWidth) / (iWidth + iHeight);

  return pDcVal;
}

// Function for deriving the angular Intra predictions

/** Function for deriving the simplified angular intra predictions.
 * \param bitDepth           bit depth
 * \param pSrc               pointer to reconstructed sample array
 * \param srcStride          the stride of the reconstructed sample array
 * \param pTrueDst           reference to pointer for the prediction sample array
 * \param dstStrideTrue      the stride of the prediction sample array
 * \param uiWidth            the width of the block
 * \param uiHeight           the height of the block
 * \param channelType        type of pel array (luma/chroma)
 * \param format             chroma format
 * \param dirMode            the intra prediction mode index
 * \param blkAboveAvailable  boolean indication if the block above is available
 * \param blkLeftAvailable   boolean indication if the block to the left is available
 * \param bEnableEdgeFilters indication whether to enable edge filters
 *
 * This function derives the prediction samples for the angular mode based on the prediction direction indicated by
 * the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
 * the reference row above the block in the case of vertical prediction or displacement of the rightmost column
 * of the block and reference column left from the block in the case of the horizontal prediction. The displacement
 * is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
 * the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
 * from the extended main reference.
 */
//NOTE: Bit-Limit - 25-bit source
Void TComPrediction::xPredIntraAng(       Int bitDepth,
                                    const Pel* pSrc,     Int srcStride,
                                          Pel* pTrueDst, Int dstStrideTrue,
                                          UInt uiWidth, UInt uiHeight, ChannelType channelType,
                                          UInt dirMode, const Bool bEnableEdgeFilters
                                  )
{ /*在预测实现过程中，参考像素是存储在一维数组refMain中的*/
  Int width=Int(uiWidth);
  Int height=Int(uiHeight);

  // Map the mode index to main prediction direction and angle
  assert( dirMode != PLANAR_IDX ); //no planar
  const Bool modeDC        = dirMode==DC_IDX;

  // Do the DC prediction
  if (modeDC)
  {
    const Pel dcval = predIntraGetPredValDC(pSrc, srcStride, width, height);

    for (Int y=height;y>0;y--, pTrueDst+=dstStrideTrue)
    {
      for (Int x=0; x<width;) // width is always a multiple of 4.
      {
        pTrueDst[x++] = dcval;//当前快的预测值等于一个均值
      }
    }
  }

  /****************************************						角度预测					***********************************************/
  else // Do angular predictions
  {
    const Bool       bIsModeVer         = (dirMode >= 18);//是否为垂直类模式（大于18为垂直模式）
    const Int        intraPredAngleMode = (bIsModeVer) ? (Int)dirMode - VER_IDX :  -((Int)dirMode - HOR_IDX);//计算相对于垂直（水平）模式的偏移
    //  1.( x<10 )_'+'  2.( 10<=x<18 )_'-'  3.( 18<=x<26 )_'-'  3.( x>=26 )_'+'
    const Int        absAngMode         = abs(intraPredAngleMode);//绝对偏移值
    const Int        signAng            = intraPredAngleMode < 0 ? -1 : 1;//偏移方向
    const Bool       edgeFilter         = bEnableEdgeFilters && isLuma(channelType) && (width <= MAXIMUM_INTRA_FILTERED_WIDTH) && (height <= MAXIMUM_INTRA_FILTERED_HEIGHT);

    // Set bitshifts and scale the angle parameter to block size
    static const Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};//HEVC.P122

    //设置invAngTable 的目的是为了消除 帧内角度预测模式在计算预测值时麻烦的除法运算
    static const Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle  参考书P122计算公式 但是为什么要乘256呢
    Int invAngle                    = invAngTable[absAngMode];
    Int absAng                      = angTable[absAngMode];
    Int intraPredAngle              = signAng * absAng;             //（OffSet值）。。。与intraPredAngleMode没区别吧？

    Pel* refMain;       //指向参考采样中，不需要投影的部分
    Pel* refSide;       //指向参考采样中，需要投影的部分

    Pel  refAbove[2*MAX_CU_SIZE+1];//数组，大小为最大可能的size
    Pel  refLeft[2*MAX_CU_SIZE+1]; //数组，大小为最大可能的size

    /***************Initialize the Main and Left reference array.***************/
    if (intraPredAngle < 0)     //对应原始的11-25模式(第二象限↖)
    {
      const Int refMainOffsetPreScale = (bIsModeVer ? height : width ) - 1;
      const Int refMainOffset         = height - 1;//索引从0开始
      for (Int x=0;x<width+1;x++)  //逐列扫，获取各列上方像素
      {

        //pSrc可能指向的是当前PU位置的第一个像素。通过负索引可以获取之前的重建像素。
        refAbove[x+refMainOffset] = pSrc[x-srcStride-1];//从src里取像素，从height-1 到 width+height-1（行排列）像素
                                                        //因为11-25模式需要参考左侧和上方像素，所以有wid+hei-1个参考像素
                                                        //srcStride是 两倍TU宽，通过一个srcStride可以跳到上一行，如何实现不太清楚
      }
      for (Int y=0;y<height+1;y++)  //逐行扫，获取各行左侧像素
      {
        refLeft[y+refMainOffset] = pSrc[(y-1)*srcStride-1];//读取左侧某个像素  其中（-srcStride-1） 应该是左上侧元素（单个）
      }

      //这两行针对11-25模式的垂直（水平）类，进行不同的Main_Side参考像素分配。Main_不需要投影的像素；Side_需要投影的像素
      refMain = (bIsModeVer ? refAbove : refLeft)  + refMainOffset;//前面俩循环里，refAbove(Left)的起始索引为refMainOffset的原因应该就是配合这里
      refSide = (bIsModeVer ? refLeft  : refAbove) + refMainOffset;

      // Extend the Main reference to the left.
      Int invAngleSum    = 128;       // rounding for (shift by 8)
      for (Int k=-1; k>(refMainOffsetPreScale+1)*intraPredAngle>>5; k--)
      {
        invAngleSum += invAngle;             //  等价于k * invAngle, 即求出在Side Ref上的截距
        refMain[k] = refSide[invAngleSum>>8];//  把Side Ref 投影到Main Ref中  __之前乘256，现在又除256
      }
    }
    else    //对应原始的0-9模式27-34 ( 第三象限↙    第一象限↗)直接用 上方（左侧）像素参考
    {
      for (Int x=0;x<2*width+1;x++)
      {
        refAbove[x] = pSrc[x-srcStride-1];
      }
      for (Int y=0;y<2*height+1;y++)
      {
        refLeft[y] = pSrc[(y-1)*srcStride-1];
      }
      refMain = bIsModeVer ? refAbove : refLeft ;
      refSide = bIsModeVer ? refLeft  : refAbove;
    }

    // swap width/height if we are doing a horizontal mode:   //为计算方便：swap 
    Pel tempArray[MAX_CU_SIZE*MAX_CU_SIZE];
    const Int dstStride = bIsModeVer ? dstStrideTrue : MAX_CU_SIZE;
    Pel *pDst = bIsModeVer ? pTrueDst : tempArray;
    if (!bIsModeVer)
    {
      std::swap(width, height);
    }

    if (intraPredAngle == 0)  // pure vertical or pure horizontal
    {
      for (Int y=0;y<height;y++)//这里的width和height是TU的size吧
      {
        for (Int x=0;x<width;x++)
        {
          pDst[y*dstStride+x] = refMain[x+1];   //第0个元素是左上角     pDst与refMain 在x上对齐
          //pDst索引一直+1，利用dstStride跳行（列），x：垂直————宽；水平————高
        }
      }

      if (edgeFilter)
      {
        for (Int y=0;y<height;y++)
        {
          pDst[y*dstStride] = Clip3 (0, ((1 << bitDepth) - 1), pDst[y*dstStride] + (( refSide[y+1] - refSide[0] ) >> 1) );
        }
      }
    }
    else
    {
      Pel *pDsty=pDst;

      //                                                     deltaPos == y * offSet   dst滑动一个Stride
      for (Int y=0, deltaPos=intraPredAngle; y<height; y++, deltaPos+=intraPredAngle, pDsty+=dstStride)
      {//每一行
        const Int deltaInt   = deltaPos >> 5;   //  Ref中的位置（整像素偏移）
        const Int deltaFract = deltaPos & (32 - 1); //3个对角模式刚好为0，其余都是 deltaPos（1/32像素偏移，用于产生滤波器系数）

        if (deltaFract)
        {
          // Do linear filtering
          const Pel *pRM=refMain+deltaInt+1;    //+1 是因为？？(索引可能相差1，Ref中的[0]是当前预测块左上方像素，Pix[0][0]是当前预测块内左上角像素)
          Int lastRefMainPel=*pRM++;//last指向当前 rRM 的位置
          for (Int x=0;x<width;pRM++,x++)
          {
            Int thisRefMainPel=*pRM;//this指向当前 rPM  +1的位置
            pDsty[x+0] = (Pel) ( ((32-deltaFract)*lastRefMainPel + deltaFract*thisRefMainPel +16) >> 5 );//2抽头   +0.5四舍五入吗
            lastRefMainPel=thisRefMainPel;
          }
        }
        else
        {
          // Just copy the integer samples
          for (Int x=0;x<width; x++)
          {
            pDsty[x] = refMain[x+deltaInt+1];
          }
        }
      }
    }

    // Flip the block if this is the horizontal mode    //水平模式再翻转回来
    if (!bIsModeVer)
    {
      for (Int y=0; y<height; y++)
      {
        for (Int x=0; x<width; x++)
        {
          pTrueDst[x*dstStrideTrue] = pDst[x];
        }
        pTrueDst++;
        pDst+=dstStride;
      }
    }
  }
}

Void TComPrediction::predIntraAng( const ComponentID compID, UInt uiDirMode, Pel* piOrg /* Will be null for decoding */, UInt uiOrgStride, Pel* piPred, UInt uiStride, TComTU &rTu, const Bool bUseFilteredPredSamples, const Bool bUseLosslessDPCM )
{
  const ChannelType    channelType = toChannelType(compID);
  const TComRectangle &rect        = rTu.getRect(isLuma(compID) ? COMPONENT_Y : COMPONENT_Cb);
  const Int            iWidth      = rect.width;    //重建TU的wid
  const Int            iHeight     = rect.height;   //重建TU的hei

  assert( g_aucConvertToBit[ iWidth ] >= 0 ); //   4x  4
  assert( g_aucConvertToBit[ iWidth ] <= 5 ); // 128x128
  //assert( iWidth == iHeight  );

        Pel *pDst = piPred;

  // get starting pixel in block
  const Int sw = (2 * iWidth + 1);

  if ( bUseLosslessDPCM )
  {
    const Pel *ptrSrc = getPredictorPtr( compID, false );
    // Sample Adaptive intra-Prediction (SAP)
    if (uiDirMode==HOR_IDX)
    {
      // left column filled with reference samples
      // remaining columns filled with piOrg data (if available).
      for(Int y=0; y<iHeight; y++)
      {
        piPred[y*uiStride+0] = ptrSrc[(y+1)*sw];
      }
      if (piOrg!=0)
      {
        piPred+=1; // miss off first column
        for(Int y=0; y<iHeight; y++, piPred+=uiStride, piOrg+=uiOrgStride)
        {
          memcpy(piPred, piOrg, (iWidth-1)*sizeof(Pel));
        }
      }
    }
    else // VER_IDX
    {
      // top row filled with reference samples
      // remaining rows filled with piOrd data (if available)
      for(Int x=0; x<iWidth; x++)
      {
        piPred[x] = ptrSrc[x+1];
      }
      if (piOrg!=0)
      {
        piPred+=uiStride; // miss off the first row
        for(Int y=1; y<iHeight; y++, piPred+=uiStride, piOrg+=uiOrgStride)
        {
          memcpy(piPred, piOrg, iWidth*sizeof(Pel));
        }
      }
    }
  }
  else
  {
    const Pel *ptrSrc = getPredictorPtr( compID, bUseFilteredPredSamples );

    if ( uiDirMode == PLANAR_IDX )
    {
      xPredIntraPlanar( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight );//预测Planar模式
    }
    else
    {
      // Create the prediction
            TComDataCU *const pcCU              = rTu.getCU();
      const UInt              uiAbsPartIdx      = rTu.GetAbsPartIdxTU();
      const Bool              enableEdgeFilters = !(pcCU->isRDPCMEnabled(uiAbsPartIdx) && pcCU->getCUTransquantBypass(uiAbsPartIdx));//边缘滤波器
#if O0043_BEST_EFFORT_DECODING
      const Int channelsBitDepthForPrediction = rTu.getCU()->getSlice()->getSPS()->getStreamBitDepth(channelType);
#else
      const Int channelsBitDepthForPrediction = rTu.getCU()->getSlice()->getSPS()->getBitDepth(channelType);
#endif
      xPredIntraAng( channelsBitDepthForPrediction, ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, channelType, uiDirMode, enableEdgeFilters );//角度模式预测

      if( uiDirMode == DC_IDX )
      {
        xDCPredFiltering( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, channelType );//DC模式滤波？
      }
    }
  }

}

/** Check for identical motion in both motion vector direction of a bi-directional predicted CU
  * \returns true, if motion vectors and reference pictures match
 */
Bool TComPrediction::xCheckIdenticalMotion ( TComDataCU* pcCU, UInt PartAddr )
{
  if( pcCU->getSlice()->isInterB() && !pcCU->getSlice()->getPPS()->getWPBiPred() )
  {
    if( pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr) >= 0 && pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr) >= 0)
    {
      Int RefPOCL0 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_0, pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr))->getPOC();
      Int RefPOCL1 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_1, pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr))->getPOC();
      if(RefPOCL0 == RefPOCL1 && pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(PartAddr) == pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(PartAddr))
      {
        return true;
      }
    }
  }
  return false;
}

Void TComPrediction::motionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred, RefPicList eRefPicList, Int iPartIdx )
{
  Int         iWidth;
  Int         iHeight;
  UInt        uiPartAddr;
  const TComSlice *pSlice    = pcCU->getSlice();
  const SliceType  sliceType = pSlice->getSliceType();
  const TComPPS   &pps       = *(pSlice->getPPS());

  if ( iPartIdx >= 0 )
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );
    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( (sliceType == P_SLICE && pps.getUseWP()) || (sliceType == B_SLICE && pps.getWPBiPred()))
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true );
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
      if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
      }
      else
      {
        xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred );
      }
    }
    return;
  }

  for ( iPartIdx = 0; iPartIdx < pcCU->getNumPartitions(); iPartIdx++ )
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );

    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( (sliceType == P_SLICE && pps.getUseWP()) || (sliceType == B_SLICE && pps.getWPBiPred()))
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true );
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
      if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
      }
      else
      {
        xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred );
      }
    }
  }
  return;
}

Void TComPrediction::xPredInterUni ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv* pcYuvPred, Bool bi )
{
  Int         iRefIdx     = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );           assert (iRefIdx >= 0);
  TComMv      cMv         = pcCU->getCUMvField( eRefPicList )->getMv( uiPartAddr );
  pcCU->clipMv(cMv);

  for (UInt comp=COMPONENT_Y; comp<pcYuvPred->getNumberValidComponents(); comp++)
  {
    const ComponentID compID=ComponentID(comp);
    xPredInterBlk  (compID,  pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred, bi, pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID)) );
  }
}

Void TComPrediction::xPredInterBi ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv* pcYuvPred )
{
  TComYuv* pcMbYuv;
  Int      iRefIdx[NUM_REF_PIC_LIST_01] = {-1, -1};

  for ( UInt refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    RefPicList eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    iRefIdx[refList] = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );

    if ( iRefIdx[refList] < 0 )
    {
      continue;
    }

    assert( iRefIdx[refList] < pcCU->getSlice()->getNumRefIdx(eRefPicList) );

    pcMbYuv = &m_acYuvPred[refList];
    if( pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( uiPartAddr ) >= 0 && pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( uiPartAddr ) >= 0 )
    {
      xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, true );
    }
    else
    {
      if ( ( pcCU->getSlice()->getPPS()->getUseWP()       && pcCU->getSlice()->getSliceType() == P_SLICE ) ||
           ( pcCU->getSlice()->getPPS()->getWPBiPred()    && pcCU->getSlice()->getSliceType() == B_SLICE ) )
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, true );
      }
      else
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv );
      }
    }
  }

  if ( pcCU->getSlice()->getPPS()->getWPBiPred()    && pcCU->getSlice()->getSliceType() == B_SLICE  )
  {
    xWeightedPredictionBi( pcCU, &m_acYuvPred[REF_PIC_LIST_0], &m_acYuvPred[REF_PIC_LIST_1], iRefIdx[REF_PIC_LIST_0], iRefIdx[REF_PIC_LIST_1], uiPartAddr, iWidth, iHeight, pcYuvPred );
  }
  else if ( pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType() == P_SLICE )
  {
    xWeightedPredictionUni( pcCU, &m_acYuvPred[REF_PIC_LIST_0], uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
  }
  else
  {
    xWeightedAverage( &m_acYuvPred[REF_PIC_LIST_0], &m_acYuvPred[REF_PIC_LIST_1], iRefIdx[REF_PIC_LIST_0], iRefIdx[REF_PIC_LIST_1], uiPartAddr, iWidth, iHeight, pcYuvPred, pcCU->getSlice()->getSPS()->getBitDepths() );
  }
}

/**
 * \brief Generate motion-compensated block
 *
 * \param compID     Colour component ID
 * \param cu         Pointer to current CU
 * \param refPic     Pointer to reference picture
 * \param partAddr   Address of block within CU
 * \param mv         Motion vector
 * \param width      Width of block
 * \param height     Height of block
 * \param dstPic     Pointer to destination picture
 * \param bi         Flag indicating whether bipred is used
 * \param  bitDepth  Bit depth
 */


Void TComPrediction::xPredInterBlk(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *dstPic, Bool bi, const Int bitDepth )
{
  Int     refStride  = refPic->getStride(compID);
  Int     dstStride  = dstPic->getStride(compID);
  Int shiftHor=(2+refPic->getComponentScaleX(compID));
  Int shiftVer=(2+refPic->getComponentScaleY(compID));

  Int     refOffset  = (mv->getHor() >> shiftHor) + (mv->getVer() >> shiftVer) * refStride;

  Pel*    ref     = refPic->getAddr(compID, cu->getCtuRsAddr(), cu->getZorderIdxInCtu() + partAddr ) + refOffset;

  Pel*    dst = dstPic->getAddr( compID, partAddr );

  Int     xFrac  = mv->getHor() & ((1<<shiftHor)-1);
  Int     yFrac  = mv->getVer() & ((1<<shiftVer)-1);
  UInt    cxWidth  = width  >> refPic->getComponentScaleX(compID);
  UInt    cxHeight = height >> refPic->getComponentScaleY(compID);

  const ChromaFormat chFmt = cu->getPic()->getChromaFormat();

  if ( yFrac == 0 )
  {
    m_if.filterHor(compID, ref, refStride, dst,  dstStride, cxWidth, cxHeight, xFrac, !bi, chFmt, bitDepth);
  }
  else if ( xFrac == 0 )
  {
    m_if.filterVer(compID, ref, refStride, dst, dstStride, cxWidth, cxHeight, yFrac, true, !bi, chFmt, bitDepth);
  }
  else
  {
    Int   tmpStride = m_filteredBlockTmp[0].getStride(compID);
    Pel*  tmp       = m_filteredBlockTmp[0].getAddr(compID);

    const Int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;

    m_if.filterHor(compID, ref - ((vFilterSize>>1) -1)*refStride, refStride, tmp, tmpStride, cxWidth, cxHeight+vFilterSize-1, xFrac, false,      chFmt, bitDepth);
    m_if.filterVer(compID, tmp + ((vFilterSize>>1) -1)*tmpStride, tmpStride, dst, dstStride, cxWidth, cxHeight,               yFrac, false, !bi, chFmt, bitDepth);
  }
}

Void TComPrediction::xWeightedAverage( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartIdx, Int iWidth, Int iHeight, TComYuv* pcYuvDst, const BitDepths &clipBitDepths )
{
  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
    pcYuvDst->addAvg( pcYuvSrc0, pcYuvSrc1, uiPartIdx, iWidth, iHeight, clipBitDepths );
  }
  else if ( iRefIdx0 >= 0 && iRefIdx1 <  0 )
  {
    pcYuvSrc0->copyPartToPartYuv( pcYuvDst, uiPartIdx, iWidth, iHeight );
  }
  else if ( iRefIdx0 <  0 && iRefIdx1 >= 0 )
  {
    pcYuvSrc1->copyPartToPartYuv( pcYuvDst, uiPartIdx, iWidth, iHeight );
  }
}

// AMVP
Void TComPrediction::getMvPredAMVP( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv& rcMvPred )
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();

  if( pcAMVPInfo->iN <= 1 )
  {
    rcMvPred = pcAMVPInfo->m_acMvCand[0];

    pcCU->setMVPIdxSubParts( 0, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    return;
  }

  assert(pcCU->getMVPIdx(eRefPicList,uiPartAddr) >= 0);
  rcMvPred = pcAMVPInfo->m_acMvCand[pcCU->getMVPIdx(eRefPicList,uiPartAddr)];
  return;
}

/** Function for deriving planar intra prediction.
 * \param pSrc        pointer to reconstructed sample array
 * \param srcStride   the stride of the reconstructed sample array
 * \param rpDst       reference to pointer for the prediction sample array
 * \param dstStride   the stride of the prediction sample array
 * \param width       the width of the block
 * \param height      the height of the block
 * \param channelType type of pel array (luma, chroma)
 * \param format      chroma format
 *
 * This function derives the prediction samples for planar mode (intra coding).
 */
//NOTE: Bit-Limit - 24-bit source
Void TComPrediction::xPredIntraPlanar( const Pel* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height )
{
  assert(width <= height);

  Int leftColumn[MAX_CU_SIZE+1], topRow[MAX_CU_SIZE+1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  UInt shift1Dhor = g_aucConvertToBit[ width ] + 2;
  UInt shift1Dver = g_aucConvertToBit[ height ] + 2;

  // Get left and above reference column and row
  for(Int k=0;k<width+1;k++)
  {
    topRow[k] = pSrc[k-srcStride];
  }

  for (Int k=0; k < height+1; k++)
  {
    leftColumn[k] = pSrc[k*srcStride-1];
  }

  // Prepare intermediate variables used in interpolation
  Int bottomLeft = leftColumn[height];
  Int topRight   = topRow[width];

  for(Int k=0;k<width;k++)
  {
    bottomRow[k]  = bottomLeft - topRow[k];
    topRow[k]     <<= shift1Dver;
  }

  for(Int k=0;k<height;k++)
  {
    rightColumn[k]  = topRight - leftColumn[k];
    leftColumn[k]   <<= shift1Dhor;
  }

  const UInt topRowShift = 0;

  // Generate prediction signal
  for (Int y=0;y<height;y++)
  {
    Int horPred = leftColumn[y] + width;
    for (Int x=0;x<width;x++)
    {
      horPred += rightColumn[y];
      topRow[x] += bottomRow[x];

      Int vertPred = ((topRow[x] + topRowShift)>>topRowShift);
      rpDst[y*dstStride+x] = ( horPred + vertPred ) >> (shift1Dhor+1);
    }
  }
}

/** Function for filtering intra DC predictor.
 * \param pSrc pointer to reconstructed sample array
 * \param iSrcStride the stride of the reconstructed sample array
 * \param pDst reference to pointer for the prediction sample array
 * \param iDstStride the stride of the prediction sample array
 * \param iWidth the width of the block
 * \param iHeight the height of the block
 * \param channelType type of pel array (luma, chroma)
 *
 * This function performs filtering left and top edges of the prediction samples for DC mode (intra coding).
 */
Void TComPrediction::xDCPredFiltering( const Pel* pSrc, Int iSrcStride, Pel* pDst, Int iDstStride, Int iWidth, Int iHeight, ChannelType channelType )
{
  Int x, y, iDstStride2, iSrcStride2;

  if (isLuma(channelType) && (iWidth <= MAXIMUM_INTRA_FILTERED_WIDTH) && (iHeight <= MAXIMUM_INTRA_FILTERED_HEIGHT))
  {
    //top-left
    pDst[0] = (Pel)((pSrc[-iSrcStride] + pSrc[-1] + 2 * pDst[0] + 2) >> 2);

    //top row (vertical filter)
    for ( x = 1; x < iWidth; x++ )
    {
      pDst[x] = (Pel)((pSrc[x - iSrcStride] +  3 * pDst[x] + 2) >> 2);
    }

    //left column (horizontal filter)
    for ( y = 1, iDstStride2 = iDstStride, iSrcStride2 = iSrcStride-1; y < iHeight; y++, iDstStride2+=iDstStride, iSrcStride2+=iSrcStride )
    {
      pDst[iDstStride2] = (Pel)((pSrc[iSrcStride2] + 3 * pDst[iDstStride2] + 2) >> 2);
    }
  }

  return;
}

/* Static member function */
Bool TComPrediction::UseDPCMForFirstPassIntraEstimation(TComTU &rTu, const UInt uiDirMode)
{
  return (rTu.getCU()->isRDPCMEnabled(rTu.GetAbsPartIdxTU()) ) &&
          rTu.getCU()->getCUTransquantBypass(rTu.GetAbsPartIdxTU()) &&
          (uiDirMode==HOR_IDX || uiDirMode==VER_IDX);
}

#if MCTS_ENC_CHECK

Void getRefPUPartPos(TComDataCU* pcCU, TComMv& cMv, Int uiPartIdx, Int& ruiPredXLeft, Int& ruiPredYTop, Int& ruiPredXRight, Int& ruiPredYBottom, Int iWidth, Int iHeight)
{
  ruiPredXLeft = pcCU->getCUPelX();
  ruiPredYTop = pcCU->getCUPelY();

  switch (pcCU->getPartitionSize(0))
  {
  case SIZE_2NxN:
    if (uiPartIdx == 0)
    {
      ruiPredXRight = ruiPredXLeft + iWidth;
      ruiPredYBottom = ruiPredYTop + iHeight;
    }
    else
    {
      ruiPredXRight = ruiPredXLeft + iWidth;
      ruiPredYBottom = ruiPredYTop + (iHeight << 1);
      ruiPredYTop += iHeight;
    }
    break;
  case SIZE_Nx2N:
    if (uiPartIdx == 0)
    {
      ruiPredXRight = ruiPredXLeft + iWidth;
      ruiPredYBottom = ruiPredYTop + iHeight;
    }
    else
    {
      ruiPredXRight = ruiPredXLeft + (iWidth << 1);
      ruiPredYBottom = ruiPredYTop + iHeight;
      ruiPredXLeft += iWidth;
    }
    break;
  case SIZE_NxN:
    if (uiPartIdx == 0)
    {
      ruiPredXRight = ruiPredXLeft + iWidth;
      ruiPredYBottom = ruiPredYTop + iHeight;
    }
    else if (uiPartIdx == 1)
    {
      ruiPredXRight = ruiPredXLeft + (iWidth << 1);
      ruiPredYBottom = ruiPredYTop + iHeight;
      ruiPredXLeft += iWidth;
    }
    else if (uiPartIdx == 2)
    {
      ruiPredXRight = ruiPredXLeft + iWidth;
      ruiPredYBottom = ruiPredYTop + (iHeight << 1);
      ruiPredYTop += iHeight;
    }
    else if (uiPartIdx == 3)
    {
      ruiPredXRight = ruiPredXLeft + (iWidth << 1);
      ruiPredYBottom = ruiPredYTop + (iHeight << 1);
      ruiPredXLeft += iWidth;
      ruiPredYTop += iHeight;
    }
    break;
  case SIZE_2NxnU:
    if (uiPartIdx == 0)
    {
      ruiPredXRight = ruiPredXLeft + iWidth;
      ruiPredYBottom = ruiPredYTop + iHeight;
    }
    else
    {
      ruiPredXRight = ruiPredXLeft + iWidth;
      ruiPredYBottom = ruiPredYTop + pcCU->getHeight(0);
      ruiPredYTop += (iHeight / 3);
    }
    break;
  case SIZE_2NxnD:
    if (uiPartIdx == 0)
    {
      ruiPredXRight = ruiPredXLeft + iWidth;
      ruiPredYBottom = ruiPredYTop + iHeight;
    }
    else
    {
      Int oriHeight = iHeight << 2;
      ruiPredXRight = ruiPredXLeft + iWidth;
      ruiPredYBottom = ruiPredYTop + oriHeight;
      ruiPredYTop += (oriHeight >> 2) + (oriHeight >> 1);
    }
    break;
  case SIZE_nLx2N:
    if (uiPartIdx == 0)
    {
      ruiPredXRight = ruiPredXLeft + iWidth;
      ruiPredYBottom = ruiPredYTop + iHeight;
    }
    else
    {
      ruiPredXRight = ruiPredXLeft + pcCU->getWidth(0);
      ruiPredYBottom = ruiPredYTop + iHeight;
      ruiPredXLeft += (iWidth / 3);
    }
    break;
  case SIZE_nRx2N:
    if (uiPartIdx == 0)
    {
      ruiPredXRight = ruiPredXLeft + iWidth;
      ruiPredYBottom = ruiPredYTop + iHeight;
    }
    else
    {
      Int oriWidth = (iWidth << 2);
      ruiPredXRight = ruiPredXLeft + oriWidth;
      ruiPredYBottom = ruiPredYTop + iHeight;
      ruiPredXLeft += (oriWidth >> 2) + (oriWidth >> 1);
    }
    break;
  default:
    ruiPredXRight = ruiPredXLeft + iWidth;
    ruiPredYBottom = ruiPredYTop + iHeight;
    break;
  }

  ruiPredXLeft   += (cMv.getHor() >> 2);
  ruiPredYTop    += (cMv.getVer() >> 2);
  ruiPredXRight  += (cMv.getHor() >> 2) - 1;
  ruiPredYBottom += (cMv.getVer() >> 2) - 1;
}

Bool checkMVPRange(TComMv& cMv, UInt ctuLength, UInt tileXPosInCtus, UInt tileYPosInCtus, UInt tileWidthtInCtus, UInt tileHeightInCtus, Int PredXLeft, Int PredXRight, Int PredYTop, Int PredYBottom, ChromaFormat chromaFormat)
{
  // filter length of sub-sample generation filter to be considered
  const UInt LumaLTSampleOffset = 3;
  const UInt LumaRBSampleOffset = 4;
  const UInt CromaLTSampleoffset = 1;
  const UInt CromaRBSampleoffset = 2;

  // tile position in full pels
  const Int leftTopPelPosX = ctuLength * tileXPosInCtus;
  const Int leftTopPelPosY = ctuLength * tileYPosInCtus;
  const Int rightBottomPelPosX = ((tileWidthtInCtus + tileXPosInCtus) * ctuLength) - 1;
  const Int rightBottomPelPosY = ((tileHeightInCtus + tileYPosInCtus) * ctuLength) - 1;

  // Luma MV range check
  const Bool isFullPelHorLuma = (cMv.getHor() % 4 == 0);
  const Bool isFullPelVerLuma = (cMv.getVer() % 4 == 0);

  const Int lRangeXLeft = leftTopPelPosX + (isFullPelHorLuma ? 0 : LumaLTSampleOffset);
  const Int lRangeYTop = leftTopPelPosY + (isFullPelVerLuma ? 0 : LumaLTSampleOffset);
  const Int lRangeXRight = rightBottomPelPosX - (isFullPelHorLuma ? 0 : LumaRBSampleOffset);
  const Int lRangeYBottom = rightBottomPelPosY - (isFullPelVerLuma ? 0 : LumaRBSampleOffset);

  if (!(PredXLeft >= lRangeXLeft && PredXLeft <= lRangeXRight) || !(PredXRight >= lRangeXLeft && PredXRight <= lRangeXRight))
  {
    return false;
  }
  else if (!(PredYTop >= lRangeYTop && PredYTop <= lRangeYBottom) || !(PredYBottom >= lRangeYTop && PredYBottom <= lRangeYBottom))
  {
    return false;
  }

  if ((chromaFormat != CHROMA_444) && (chromaFormat != CHROMA_400))
  {
    // Chroma MV range check
    const Bool isFullPelHorChroma = (cMv.getHor() % 8 == 0);
    const Bool isFullPelVerChroma = (cMv.getVer() % 8 == 0);

    const Int cRangeXLeft = leftTopPelPosX + (isFullPelHorChroma ? 0 : CromaLTSampleoffset);
    const Int cRangeYTop = leftTopPelPosY + (isFullPelVerChroma ? 0 : CromaLTSampleoffset);
    const Int cRangeXRight = rightBottomPelPosX - (isFullPelHorChroma ? 0 : CromaRBSampleoffset);
    const Int cRangeYBottom = rightBottomPelPosY - (isFullPelVerChroma ? 0 : CromaRBSampleoffset);

    if (!(PredXLeft >= cRangeXLeft && PredXLeft <= cRangeXRight) || !(PredXRight >= cRangeXLeft && PredXRight <= cRangeXRight))
    {
      return false;
    }
    else if ((!(PredYTop >= cRangeYTop && PredYTop <= cRangeYBottom) || !(PredYBottom >= cRangeYTop && PredYBottom <= cRangeYBottom)) && (chromaFormat != CHROMA_422))
    {
      return false;
    }
  }

  return true;
}

Bool TComPrediction::checkTMctsMvp(TComDataCU* pcCU, Int partIdx)
{
  Int   partWidth  = 0;
  Int   partHeight = 0;
  UInt  partAddr   = 0;

  UInt  tileXPosInCtus = 0;
  UInt  tileYPosInCtus = 0;
  UInt  tileWidthtInCtus = 0;
  UInt  tileHeightInCtus = 0;
  
  getTilePosition(pcCU, tileXPosInCtus, tileYPosInCtus, tileWidthtInCtus, tileHeightInCtus);

  const UInt            ctuLength = pcCU->getPic()->getPicSym()->getSPS().getMaxCUWidth();
  const ChromaFormat chromaFormat = pcCU->getPic()->getPicSym()->getSPS().getChromaFormatIdc();

  Int   predXLeft;
  Int   predYTop;
  Int   predXRight;
  Int   predYBottom;

  if (partIdx >= 0)
  {
    pcCU->getPartIndexAndSize(partIdx, partAddr, partWidth, partHeight);

    if (xCheckIdenticalMotion(pcCU, partAddr))
    {
      RefPicList eRefPicList = REF_PIC_LIST_0;
      TComMv      cMv = pcCU->getCUMvField(eRefPicList)->getMv(partAddr);
      getRefPUPartPos(pcCU, cMv, partIdx, predXLeft, predYTop, predXRight, predYBottom, partWidth, partHeight);
      if (!checkMVPRange(cMv, ctuLength, tileXPosInCtus, tileYPosInCtus, tileWidthtInCtus, tileHeightInCtus, predXLeft, predXRight, predYTop, predYBottom, chromaFormat))
      {
        return false;
      }
    }
    else
    {
      for (UInt refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
      {
        RefPicList eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

        TComMv      cMv = pcCU->getCUMvField(eRefPicList)->getMv(partAddr);
        getRefPUPartPos(pcCU, cMv, partIdx, predXLeft, predYTop, predXRight, predYBottom, partWidth, partHeight);
        if (!checkMVPRange(cMv, ctuLength, tileXPosInCtus, tileYPosInCtus, tileWidthtInCtus, tileHeightInCtus, predXLeft, predXRight, predYTop, predYBottom, chromaFormat))
        {
          return false;
        }
      }
    }
    return true;
  }

  for (partIdx = 0; partIdx < pcCU->getNumPartitions(); partIdx++)
  {
    pcCU->getPartIndexAndSize(partIdx, partAddr, partWidth, partHeight);

    if (xCheckIdenticalMotion(pcCU, partAddr))
    {
      RefPicList eRefPicList = REF_PIC_LIST_0;
      TComMv      cMv = pcCU->getCUMvField(eRefPicList)->getMv(partAddr);
      getRefPUPartPos(pcCU, cMv, partIdx, predXLeft, predYTop, predXRight, predYBottom, partWidth, partHeight);
      if (!checkMVPRange(cMv, ctuLength, tileXPosInCtus, tileYPosInCtus, tileWidthtInCtus, tileHeightInCtus, predXLeft, predXRight, predYTop, predYBottom, chromaFormat))
      {
        return false;
      }
    }
    else
    {
      for (UInt refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
      {
        RefPicList eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

        TComMv      cMv = pcCU->getCUMvField(eRefPicList)->getMv(partAddr);
        getRefPUPartPos(pcCU, cMv, partIdx, predXLeft, predYTop, predXRight, predYBottom, partWidth, partHeight);
        if (!checkMVPRange(cMv, ctuLength, tileXPosInCtus, tileYPosInCtus, tileWidthtInCtus, tileHeightInCtus, predXLeft, predXRight, predYTop, predYBottom, chromaFormat))
        {
          return false;
        }
      }
    }
  }
  return true;
}


#endif
//! \}
