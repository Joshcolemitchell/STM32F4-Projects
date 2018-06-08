/*
/*
 * these are the support routines needed to use the ST USB device
 * support for input and output.  For output it is fairly straightforward
 * and you can use the routine provided in usbd_cdc_if.c.  For receive,
 * we need to put a redirect into the CDC_Receive_FS() routine to call
 * our routine here called USB_Receive_FS() which takes any characters in
 * the USB receive buffer and transfers them to a FIFO here which emulates
 * the DMA FIFO when a UART is being used.
 *
 * After that is taken care of, this routine returns to CDC_Receive_FS()
 * where the USB receive process is reinitialized and started so that the
 * next buffer will be received
 *
 * Also, the two other mods to
 * usbd_cdc_if.c are 1. add #include "usb.h" to find this routine, and
 * 2. increase the input buffer size #define APP_RX_DATA_SIZE to 512
 * from 4 to ensure input buffer does not overflow while other stuff goes
 * on in case lots of data is being sent fast on the link
 */


#include "usb.h"

#define RxBufferSize 1024

char RxBuffer[RxBufferSize];
volatile uint readIndex = RxBufferSize;
volatile uint RxFifoIndex = 0;

//volatile uint8_t RxFIFO[512];
//volatile uint16_t FIFOput=0, FIFOget=0;


void dummy(void);

void dummy(void){
     volatile static int i;

     for (i=0;i<50;i++);

     i++;
}





/*
 * the _write() routine is used by stdio output calls like printf().  It repeatedly
 * tries to send a string until it gets back an OK from the HAL call. I have tried
 * to make this work with interrupt and DMA service and neither works it seems.
 * Not sure why.
 */
int _write(int FD, char * outstr, int len){

     while(CDC_Transmit_FS((uint8_t*)outstr,len) == USBD_BUSY); //start new transmit
 // for some reason printf() needs this recovery time
     dummy();
     return len;
}

/*
 * the _read() routine returns any characters that have been placed into
 * the RxBuffer by the USB receive FIFO process which emulates the UART
 * DMA circular buffer
 *
 * The UART receiver is set up with DMA in circular buffer mode.  This means that
 * it will continue receiving characters forever regardless of whether any are
 * taken out.  It will just write over the previous buffer if it overflows
 *
 * To know whether there are any characters in the buffer we look at the counter
 * register in the DMA channel assigned to the UART Rx buffer.  This count starts
 * at the size of the transfer and is decremented after a transfer to memory is done.
 *
 * We maintain a mirror DMA count register value readIndex.  If they are the same no data is
 * available.  If they are different, the DMA has decremented its counter
 * so we transfer data until they are the same or the rx buffer is full.  We
 * wrap our down counter the same way the DMA does in circular mode.
 */

int _read(int fd, char *instring, uint count){
     uint32_t bytesread = 0;
//     extern USBD_CDC_ItfTypeDef  USBD_Interface_fops_FS;

     if(count > bytesread){
          while(readIndex == (RxBufferSize - RxFifoIndex)){}
          {
               while((count > bytesread) & (readIndex !=(RxBufferSize - RxFifoIndex ))){
                    instring[bytesread] = RxBuffer[RxBufferSize - readIndex];
                    if(readIndex == (0))
                         readIndex = RxBufferSize;
                    else readIndex--;
                    bytesread++;
               }
          }
     }
     return (int)bytesread;
}


/*
 *  the kbhit() routine is used to check if there is a character or more available
 *  in the receive buffer.  If the buffer is empty it returns FALSE, and if there
 *  is at least one character it returns TRUE.  It is useful to check for
 *  activity before stalling a routine waiting for keyboard input if other work
 *  can be done.  It does not take a character from the buffer and does not affect
 *  any state
 */
int kbhit(void){
     if(readIndex == (RxBufferSize - RxFifoIndex))
          return 0;
     else return 1;
     return 0;
}

/*
 *
 */

/*
 * This routine is called by CDC_Receive_FS() which is a callback
 * from the underlying USB Device servicing code which gets called
 * when there is data received on the USB endpoint for the device.
 * This routine therefore acts like a circular buffer DMA for data
 * received on the USB port similar to the behavior of the circulat
 * buffer implemented on the UART receive DMA when a UART is used
 * for console I/O
 */

uint8_t USB_Receive_FS (uint8_t* Buf, uint32_t *Len)
{
     volatile uint32_t counter = 0;

     while(counter < *Len){
          RxBuffer[RxFifoIndex ] = Buf[counter];
          counter++, RxFifoIndex++;
          if(RxFifoIndex  == RxBufferSize)
               RxFifoIndex  = 0;
          }

  return (USBD_OK);
}


