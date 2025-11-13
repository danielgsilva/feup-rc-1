// Link layer protocol implementation

#include "link_layer.h"

#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FLAG 0x7e
#define A_TX 0x03
#define A_RX 0x01
#define C_SEQUENCE_NUMBER(s) s << 6
#define C_SET 0x03
#define C_DISC 0x0b
#define C_UA 0x07
#define C_RR(r) r << 7 | 0x05
#define C_REJ(r) r << 7 | 0x01
#define BCC(a, c) a ^ c
#define ESCAPE 0x7d
#define HEADER_FER 10 // 20 -> 5% | 10 -> 10% | 5 -> 20% | 3 -> 33.3% | 2 -> 50%
#define DATA_FIELD_FER 5 // 20 -> 5% | 10 -> 10% | 5 -> 20% | 3 -> 33.3% | 2 -> 50%
#define T_PROP 0 // 0 <= T_PROP < TIMEOUT 

enum state {START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP};

struct frame {
  unsigned char *frame;
  int frameSize;
};

struct termios oldtio;
int fd;
LinkLayer connection_parameters;
int sequenceNumber = 0;
int iFrameCount = 1;

int alarmEnabled = FALSE;
int alarmCount = 0;
// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = TRUE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);
}

void setUaDiscStateMachine(enum state * currentState, unsigned char addressField, unsigned char controlField, unsigned char byteReceived) {
  switch (*currentState) {
    case START:
      if(byteReceived == FLAG) *currentState = FLAG_RCV;
      break;
    case FLAG_RCV:
      if(byteReceived == addressField) *currentState = A_RCV;
      else if (byteReceived != FLAG) *currentState = START;
      break;
    case A_RCV:
      if(byteReceived == FLAG) *currentState = FLAG_RCV;
      else if(byteReceived == controlField) *currentState = C_RCV;
      else *currentState = START;
      break;
    case C_RCV:
      if(byteReceived == FLAG) *currentState = FLAG_RCV;
      else if(byteReceived == (BCC(addressField, controlField))) *currentState = BCC_OK;
      else *currentState = START;
      break;
    case BCC_OK:
      if(byteReceived == FLAG) *currentState = STOP;
      else *currentState = START;
      break;
    default:
      break;
  }
}

int sendSUFrame(int fd, unsigned char addressField, unsigned char controlField) {
  unsigned char frame[5];
  frame[0] = FLAG;
  frame[1] = addressField;
  frame[2] = controlField;
  frame[3] = BCC(addressField, controlField);
  frame[4] = FLAG;

  return write(fd, frame, 5);
}

int receiveSUFrame(int fd, unsigned char addressField, unsigned char controlField) {
  unsigned char buf[BUFSIZ] = {0};
  int bytes = 0;
  enum state currentState = START;
  do {
    bytes = read(fd, buf, 1);
    if (bytes < 0) return -1;
    else if (bytes) setUaDiscStateMachine(&currentState, addressField, controlField, buf[0]);
  } while(currentState != STOP && alarmEnabled == FALSE);

  return bytes;
}

void byteStuffing (struct frame* frame){
  unsigned char* newFrame = (unsigned char*) malloc(2 * frame->frameSize * sizeof(char));
  newFrame[0] = FLAG;
  unsigned int index = 1;
  for (size_t i = 1; i < frame->frameSize - 1; i++)
  {
    if (frame->frame[i] == FLAG){
      newFrame[index++] = ESCAPE;
      newFrame[index++] = 0x5e;
    } else if (frame->frame[i] == ESCAPE){
      newFrame[index++] = ESCAPE;
      newFrame[index++] = 0x5d;
    } else newFrame[index++] = frame->frame[i];
  }
  newFrame[index++] = FLAG;
  frame->frame = newFrame;
  frame->frameSize = index;
}

void supervisionStateMachine(enum state * currentState, unsigned char* controlField, unsigned char byteReceived) {
  switch (*currentState) {
      case START:
      if(byteReceived == FLAG) *currentState = FLAG_RCV;
      break;
    case FLAG_RCV:
      if(byteReceived == A_RX) *currentState = A_RCV;
      else if (byteReceived != FLAG) *currentState = START;
      break;
    case A_RCV:
      if(byteReceived == FLAG) *currentState = FLAG_RCV;
      else if(byteReceived == (C_RR(1)) || byteReceived == (C_RR(0)) || byteReceived == (C_REJ(0)) || byteReceived == (C_REJ(1))) {
        *controlField = byteReceived;
        *currentState = C_RCV;
      }
      else *currentState = START;
      break;
    case C_RCV:
      if(byteReceived == FLAG) *currentState = FLAG_RCV;
      else if(byteReceived == (BCC(A_RX, *controlField))) *currentState = BCC_OK;
      else *currentState = START;
      break;
    case BCC_OK:
      if(byteReceived == FLAG) *currentState = STOP;
      else *currentState = START;
      break;
    default:
      break;
  }
}

int sendIFrame(int fd, struct frame* frame){
  int bytes = 0;

  // error generation on information frames (at the transmitter)
  if ((iFrameCount % 10) == 0){ // 1 out of 10 frames has an error
    unsigned char errorFrame[frame->frameSize];
    for (size_t i = 0; i < frame->frameSize; i++){
      errorFrame[i] = frame->frame[i];
    }
    //errorFrame[1] = 0x04; // frame with wrong header
    //errorFrame[4] = 0xff; // frame with wrong data
    bytes = write(fd, errorFrame, frame->frameSize);
  } else bytes = write(fd, frame->frame, frame->frameSize);
  printf("send i frame: %d\n", iFrameCount);
  iFrameCount++;

  unsigned char buf[BUFSIZ] = {0};
  alarmEnabled = FALSE;
  alarmCount = 0;
  enum state currentState = START;
  unsigned char controlField;
  alarm(connection_parameters.timeout); 
  do
  {
    if (alarmEnabled){
      alarm(connection_parameters.timeout);
      alarmEnabled = FALSE;
      bytes = write(fd, frame->frame, frame->frameSize);
    }
    if (read(fd, buf, 1)) supervisionStateMachine(&currentState, &controlField, buf[0]);
  } while (currentState != STOP && alarmCount <= connection_parameters.nRetransmissions);
  alarm(0);
  int nextSequenceNumber = sequenceNumber ? 0 : 1;
  if (currentState == STOP){
    if (controlField == (C_REJ(sequenceNumber))){
      return sendIFrame(fd, frame);
    }
    if (controlField == (C_RR(nextSequenceNumber))){
      sequenceNumber = nextSequenceNumber;
      return bytes;
    }
  }

  return -1;
}

void byteDestuffing (struct frame* frame){
  unsigned char* newFrame = (unsigned char*) malloc(frame->frameSize * sizeof(char));
  newFrame[0] = FLAG;
  unsigned int index = 1;
  for (size_t i = 1; i < frame->frameSize - 1; i++)
  {
    if (frame->frame[i] == ESCAPE){
      i++;
      if (frame->frame[i] == 0x5e) newFrame[index++] = FLAG;
      else if (frame->frame[i] == 0x5d) newFrame[index++] = ESCAPE;
    } else newFrame[index++] = frame->frame[i];
  }
  newFrame[index++] = FLAG;
  frame->frame = newFrame;
  frame->frameSize = index;
}

void informationStateMachine(enum state * currentState, int* flagReceived, unsigned char byteReceived) {
  switch (*currentState) {
    case START: {
      if (byteReceived == FLAG) {
       *currentState = FLAG_RCV;
       *flagReceived = TRUE;
       }
      break;
    }
    case FLAG_RCV: {
      if (byteReceived == FLAG) *currentState = (*flagReceived) ? FLAG_RCV : STOP;
      else *flagReceived = FALSE;
      break;
    }
    default: 
      break;
  }
}

int receiveIFrame(int fd, struct frame* frame){
  unsigned char buf[BUFSIZ] = {0};
  int bytes = 0;
  int flagReceived = FALSE;
  enum state currentState = START;
  int index = 0;
  do {
    bytes = read(fd, buf, 1);
    if (bytes < 0) return -1;
    else if (bytes){
      informationStateMachine(&currentState, &flagReceived, buf[0]);
      if (flagReceived && currentState == FLAG_RCV) index = 0;
      if (currentState != START) frame->frame[index++] = buf[0];
    }
  } while (currentState != STOP);
  frame->frameSize = index;
  
  return index;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    connection_parameters = connectionParameters;

    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        exit(-1);
    }

    //struct termios oldtio;
    struct termios newtio;

    // Set alarm function handler
    (void)signal(SIGALRM, alarmHandler);

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 5; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 1 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    if (connectionParameters.role == LlTx){
      sendSUFrame(fd, A_TX, C_SET);
      alarmEnabled = FALSE;
      alarmCount = 0;
      alarm(connection_parameters.timeout); 
      do
      {
        if (alarmEnabled){
          alarm(connection_parameters.timeout);
          alarmEnabled = FALSE;
          sendSUFrame(fd, A_TX, C_SET);
        }
      } while (receiveSUFrame(fd, A_RX, C_UA) <= 0 && alarmCount <= connection_parameters.nRetransmissions);
      alarm(0);
      if (alarmCount > connection_parameters.nRetransmissions){
        printf("retransmission attempts exceeded");
        return -1;
      }
    } else if (connectionParameters.role == LlRx){
        receiveSUFrame(fd, A_TX, C_SET);
        sendSUFrame(fd, A_RX, C_UA);
    }

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
  unsigned char* informationFrame = (unsigned char*) malloc(bufSize * sizeof(char) + 6);
  informationFrame[0] = FLAG;
  informationFrame[1] = A_TX;
  informationFrame[2] = C_SEQUENCE_NUMBER(sequenceNumber);
  informationFrame[3] = BCC(A_TX, informationFrame[2]);
  unsigned int bcc2 = 0;
  for (size_t i = 0; i < bufSize; i++)
  {
    informationFrame[i + 4] = buf[i];
    bcc2 ^= buf[i];
  }
  informationFrame[(bufSize + 6) - 2] = bcc2;
  informationFrame[(bufSize + 6) - 1] = FLAG;

  struct frame* frame = (struct frame*) malloc(sizeof(struct frame));
  frame->frame = informationFrame;
  frame->frameSize = bufSize + 6; 

  byteStuffing(frame);
  free(informationFrame);

  int bytes = sendIFrame(fd, frame);
  free(frame->frame);
  free(frame);

  return bytes;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
  // generation of a simulated propagation delay
  if (T_PROP){
    alarmEnabled = FALSE;
    alarmCount = 0;
    alarm(T_PROP);
    while(!alarmEnabled){}
    alarm(0);
  }

  struct frame* frame = (struct frame*) malloc(sizeof(struct frame));
  unsigned char* informationFrame = (unsigned char*) malloc(2 * (MAX_PAYLOAD_SIZE + 4 + 6) * sizeof(unsigned char));
  frame->frame = informationFrame;
  if (receiveIFrame(fd, frame) < 0) return -1;
  byteDestuffing(frame);

  iFrameCount++;
  // error generation on information frames (at the receiver)
  if ((iFrameCount % HEADER_FER) == 0){
    printf("frame with wrong header simulated\n");
    free(frame->frame);
    free(frame);
    return 0;
  }
  if ((iFrameCount % DATA_FIELD_FER) == 0){
    printf("frame with wrong bcc2 simulated\n");
    sendSUFrame(fd, A_RX, C_REJ(sequenceNumber));
    free(frame->frame);
    free(frame);
    return 0;
  }
  
  if(frame->frame[0] != FLAG || frame->frame[1] != A_TX || frame->frame[3] != (BCC(A_TX, C_SEQUENCE_NUMBER(sequenceNumber)))){
    printf("frame with wrong header ignored\n");
    free(frame->frame);
    free(frame);
    return 0;
  }
  unsigned char previousSequenceNumber = sequenceNumber ? 0 : 1;
  if (frame->frame[2] == (C_SEQUENCE_NUMBER(previousSequenceNumber))){
    printf("duplicate frame\n");
    sendSUFrame(fd, A_RX, C_RR(sequenceNumber));
    free(frame->frame);
    free(frame);
    return 0;
  }
  if (frame->frame[2] != (C_SEQUENCE_NUMBER(sequenceNumber))){
    printf("frame with wrong header ignored\n");
    free(frame->frame);
    free(frame);
    return 0;
  }

  unsigned char bcc2 = 0;
  for(size_t i = 4; i < frame->frameSize - 2; i++) {
    bcc2 ^= frame->frame[i];
    packet[i - 4] = frame->frame[i];
  }
  if (frame->frame[frame->frameSize - 2] != bcc2){
    printf("frame with wrong bcc2\n");
    sendSUFrame(fd, A_RX, C_REJ(sequenceNumber));
    free(frame->frame);
    free(frame);
    return 0;
  }

  sequenceNumber = previousSequenceNumber;
  sendSUFrame(fd, A_RX, C_RR(sequenceNumber));
  int bytes = frame->frameSize - 6;
  free(frame->frame);
  free(frame);

  return bytes;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    if (connection_parameters.role == LlTx){
      if(sendSUFrame(fd, A_TX, C_DISC) < 0) return -1;
      alarmEnabled = FALSE;
      alarmCount = 0;
      alarm(connection_parameters.timeout); 
      do
      {
        if (alarmEnabled){
          alarm(connection_parameters.timeout);
          alarmEnabled = FALSE;
          if(sendSUFrame(fd, A_TX, C_DISC) < 0) return -1;
        }
      } while (receiveSUFrame(fd, A_RX, C_DISC) <= 0 && alarmCount <= connection_parameters.nRetransmissions);
      alarm(0);
      if(sendSUFrame(fd, A_TX, C_UA) < 0) return -1;
      if (alarmCount > connection_parameters.nRetransmissions){
        printf("retransmission attempts exceeded\n");
        return -1;
      }
    } else if (connection_parameters.role == LlRx){
      if(receiveSUFrame(fd, A_TX, C_DISC) < 0) return -1;
      if(sendSUFrame(fd, A_RX, C_DISC) < 0) return -1;
      alarmEnabled = FALSE;
      alarmCount = 0;
      alarm(connection_parameters.timeout); 
      do
      {
        if (alarmEnabled){
          alarm(connection_parameters.timeout);
          alarmEnabled = FALSE;
          if(sendSUFrame(fd, A_RX, C_DISC) < 0) return -1;
        }
      } while (receiveSUFrame(fd, A_TX, C_UA) <= 0 && alarmCount <= connection_parameters.nRetransmissions);
      alarm(0);
      if (alarmCount > connection_parameters.nRetransmissions){
        printf("retransmission attempts exceeded\n");
        return -1;
      }
    }

    if(showStatistics && connection_parameters.role == LlTx) printf("::: show statistics :::\n total i frames sent : %d\n", iFrameCount-1);

    // Restore the old port settings
    sleep(1);
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 1;
}
