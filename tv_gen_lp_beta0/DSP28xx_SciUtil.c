/*
 * File: DSP28xx_SciUtil.c
 *
 * Code generated for Simulink model 'DGenerator_SCI_CPU2_8'.
 *
 * Model version                  : 1.168
 * Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
 * C/C++ source code generated on : Sun Oct 18 17:04:26 2020
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Texas Instruments->C2000
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "DSP28xx_SciUtil.h"

/*
 * Receive character(s) from the SCIa
 * Received character(s) will be write to rcvBuff.
 * Return 0 if characters are received with no error.
 * Return 1 if waiting timeout.
 * Return 2 if data error.(receiving timout or checksum error)
 * Return 3 if a parity error occured.
 * Return 4 if a frame error occured.
 */
extern volatile struct SCI_REGS SciaRegs;
//#pragma DATA_SECTION(SciaRegs,"SciaRegsFile");
//#pragma CODE_SECTION(scia_rcv, "ramfuncs");
//#pragma CODE_SECTION(scia_xmit, "ramfuncs");

int scia_rcv(unsigned int *rcvBuff, int buffLen, int loopMode, int typeLen)
{
  int i, ct, c;
  int errorVal = NOERROR;
  unsigned int byte_cnt = 0;
  long cnt = 0;
  long maxcnt;
  if (loopMode == LONGLOOP) {
    maxcnt = RCVMAXCNTL;
  } else {
    maxcnt = RCVMAXCNTS;
  }

  for (i = 0; i<buffLen; i++)
  {
        cnt = 0;
        while (SciaRegs.SCIFFRX.bit.RXFFST == 0)
        {/* wait until data received */
          if (i == 0)
          {
            if (cnt++ > maxcnt)
            {
                return TIMEOUT;
            }
          }
          else
          {
            if (cnt++ > RCVMAXCNTL)
            {
                return TIMEOUT;//NOTEMPTY;
            }
          }
        }

        if (typeLen > 1) {
          if (byte_cnt == 0) {
            rcvBuff[i/2] = (SciaRegs.SCIRXBUF.all & 0x00FF);
            byte_cnt = 1;
          } else {
            rcvBuff[i/2] |= SciaRegs.SCIRXBUF.all << 8;
            byte_cnt = 0;
          }
        }
        else
        {
            // ??????????????????????????????...

            if(SciaRegs.SCIFFRX.bit.RXFFST > 1)
            {
                            ct = SciaRegs.SCIFFRX.bit.RXFFST;
                            //for(c = 0 ; c < ct; ++c)
                            for(c = 0 ; c < ct; ++c)
                            {
                               rcvBuff[i] = SciaRegs.SCIRXBUF.all;
                               ++i;
                            }
                            --i;
           }
            else
            {
                rcvBuff[i] = SciaRegs.SCIRXBUF.all;
            }
            /////////////////////////////////////////////////
         // if(i == 0)
         // {
         //     rcvBuff[i] = rcvBuff[i];
         // }
        }

        if (SciaRegs.SCIFFRX.bit.RXFFOVF == 1)/* detect FIFO overflow*/
        {
          SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
         // SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;    //Reset the FIFO pointer to zero.
         // SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;  // Re-enable receive FIFO operation.
        }

        if (SciaRegs.SCIRXST.bit.FE)
          errorVal = FRAMERR;
        if (SciaRegs.SCIRXST.bit.PE)
          errorVal = PRTYERR;
        if (SciaRegs.SCIRXST.bit.OE)
          errorVal = OVRNERR;
        if (SciaRegs.SCIRXST.bit.BRKDT)
          errorVal = BRKDTERR;
        if (SciaRegs.SCIRXST.bit.RXERROR == 1)
        {
          SciaRegs.SCICTL1.bit.SWRESET = 1;
          SciaRegs.SCICTL1.bit.SWRESET = 0;
          SciaRegs.SCICTL1.bit.SWRESET = 1;
        }
  }

  return errorVal;
}

int byteswap_L8cmp(unsigned int* outdata, char* recdata, int inportWidth, int
                   typeLen)
{
  int i, j;
  int numWrd = (inportWidth * typeLen)/2;
                                 /* number of words (16 bit length) to receive*/

  /* Little Endian, 8bit swap */
  for (i = 0; i < numWrd; i++) {
    outdata[i] = 0;
    for (j = 0; j<2; j++) {
      outdata[i] += recdata[i*2+j] <<(8*j);
    }
  }

  return 0;
}

/* Transmit character(s) from the SCIa*/
void scia_xmit(char* pmsg, int msglen, int typeLen)
{
   /*
  int i,j,k;
  if (typeLen==1) {
    for (i = 0; i < msglen; i++) {
      while (SciaRegs.SCIFFTX.bit.TXFFST == 16) {  // The buffer is full;
      }

      SciaRegs.SCITXBUF = pmsg[i];
    }

    while(SciaRegs.SCIFFTX.bit.TXFFST != 0)
    {
        ;
    }
  } else {
    for (i = 0; i < (msglen/2); i++) {
      for (j = 0; j<=1; j++) {
        while (SciaRegs.SCIFFTX.bit.TXFFST == 16) { // The buffer is full;
        }

        SciaRegs.SCITXBUF = pmsg[i]>>(8*j);
      }
    }

    while(SciaRegs.SCIFFTX.bit.TXFFST != 0)
    {

    }
  }
  */
    int i,j, len;
     len = msglen;
     if (typeLen==1) {
       for (i = 0; i < msglen; i++) { //test
        // while (SciaRegs.SCIFFTX.bit.TXFFST == 4) //test
         while (SciaRegs.SCIFFTX.bit.TXFFST != 0) //test
         {
         }                                // The buffer is full;

         SciaRegs.SCITXBUF = pmsg[i];
       }

      // while(SciaRegs.SCIFFTX.bit.TXFFST != 0){} //test
     } else {
       for (i = 0; i < (msglen/2); i++) {
         for (j = 0; j<=1; j++) {
           while (SciaRegs.SCIFFTX.bit.TXFFST == 16) {
           }

           SciaRegs.SCITXBUF = pmsg[i]>>(8*j);
         }
       }

       while(SciaRegs.SCIFFTX.bit.TXFFST != 0){}
     }
     SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
