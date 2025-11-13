// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#define C_DATA 0x01
#define C_START 0x02
#define C_END 0x03
#define T_FILESIZE 0x00
#define T_FILENAME 0x01

unsigned long sequence_number = 0;

int send_control_packet(const unsigned char control_field, unsigned long file_size, const char *filename)
{
    unsigned char *control_packet = (unsigned char *) malloc(MAX_PAYLOAD_SIZE * sizeof(char));
    if (control_packet == NULL){
        printf("Error on malloc!");
        return -1;
    }

    control_packet[0] = control_field;

    // Add file_size parameter
    control_packet[1] = T_FILESIZE;
    unsigned n_bytes_v = 1;
    while (file_size > 0){
        char remainder = file_size % 256;
        unsigned long valor_byte = file_size / 256;
        file_size = valor_byte;
        for (int j = 2 + n_bytes_v; j > 3; j--){
            control_packet[j] = control_packet[j - 1];
        }
        control_packet[3] = remainder;
        n_bytes_v++;
    }
    control_packet[2] = n_bytes_v; //add length of value of parameter

    // Add file_name parameter
    unsigned index = 3 + n_bytes_v;
    control_packet[index++] = T_FILENAME;
    control_packet[index++] = strlen(filename);
    for(size_t i = 0; i < strlen(filename); i++) {
        control_packet[index++] = filename[i];
    }   
    
    if (llwrite(control_packet, index) == -1) return -1;    //write control packet to receiver
    free(control_packet);
    
    return 1;
}

int send_data_packet(unsigned char *data, unsigned long size_data){
    unsigned char *dataPacket = (unsigned char *) malloc((MAX_PAYLOAD_SIZE + 4) * sizeof(char));
    dataPacket[0] = C_DATA;
    dataPacket[1] = sequence_number % 255;
    dataPacket[2] = size_data / 256;
    dataPacket[3] = (size_data % 256);
    for (int i = 0; i < size_data; i++){
        dataPacket[4 + i] = *(data + i);
    }
    if (llwrite(dataPacket, size_data + 4) == -1) return -1;
    free(dataPacket);

    return 1;
}

int receive_control_packet(unsigned char *packet, unsigned long *file_size, char *file_name){

    if (packet[0] != C_START && packet[0] != C_END){
        printf("error control field");
        return -1;
    }

    //read 1st parameter (file_size)
    if (packet[1] != T_FILESIZE){
        printf("error t field (filesize)");
        return -1;
    }
    unsigned length_file_size = packet[2];
    *(file_size) = 0;
    for(int i = 0; i < length_file_size-1; i++) {
        *(file_size) = *(file_size)*256 + packet[3+i];
    }

    //read 2st parameter (file_name)
    if (packet[3+length_file_size] != T_FILENAME){
        printf("error t field (filename)");
        return -1;
    }
    unsigned length_file_name = packet[4+length_file_size];
    for(int i = 0; i < length_file_name; i++) {
        file_name[i] = (char)packet[5+ length_file_size +i];
    }


    return 1;
}

int receive_data_packet(unsigned char *packet, unsigned char *packet_data_field, unsigned needed_sequence_number, unsigned *total_bytes_read){
    if (needed_sequence_number != packet[1] || packet[0] != C_DATA){
        printf("error control field or sequence number");
        return -1;
    }
    unsigned data_size = packet[2] * 256 + packet[3];
    *total_bytes_read += data_size;
    for (int i = 0; i < data_size; i++){
        packet_data_field[i] = packet[i + 4];
    }

    return 1;
}

int alwrite(const char *filename){
    FILE *file;
    file = fopen(filename, "r");
    if (file == NULL){
        printf("File does not exist!");
        return -1;
    }

    struct stat statbuf;
    if (stat(filename, &statbuf) == -1) {
      perror("stat");
      return -1;
    } 
    long int file_size = statbuf.st_size; // get file_size

    // send start control packet
    if (send_control_packet(C_START, file_size, filename) != 1){
        printf("error send start packet\n");
        return -1;
    }

    // send data packets
    unsigned char data[MAX_PAYLOAD_SIZE] = {0};
    unsigned char byte = 0x00;
    unsigned long byte_count = 0, total_bytes = file_size;
    do{
        if (byte_count == MAX_PAYLOAD_SIZE){
            if (send_data_packet(data, MAX_PAYLOAD_SIZE) == -1){
                printf("error data packet\n");
                return -1;
            }
            byte_count = 0;
            sequence_number++;
        }
        byte = fgetc(file);
        data[byte_count] = byte;
        byte_count++;
        total_bytes--;
    } while (total_bytes != 0);

    //send data_packet with last bytes
    if (send_data_packet(data, byte_count) == -1){
        printf("error last data packet\n");
        return -1;
    }

    // send end control packet
    if (send_control_packet(C_END, file_size, filename) != 1){
        printf("error end packet");
        return -1;
    }

    fclose(file);
    
    return 1;
}

int alread(const char *filename){
    unsigned char *packet = (unsigned char *) malloc((MAX_PAYLOAD_SIZE + 4) * sizeof(unsigned char));
    unsigned char *packet_data_field = (unsigned char *) malloc(MAX_PAYLOAD_SIZE * sizeof(unsigned char));
    unsigned long file_size_start, file_size_end;
    char *file_name_start = (char *) malloc(MAX_PAYLOAD_SIZE * sizeof(char));
    char *file_name_end = (char *) malloc(MAX_PAYLOAD_SIZE * sizeof(char));
    unsigned bytes_read = 0, seq_no = 0, total_bytes_read = 0;

    while (bytes_read <= 0){ 
        if ((bytes_read = llread(packet)) == -1){
            printf("error read start packet");
            return -1;
        }
    }
    // read start control packet
    if(receive_control_packet(packet, &file_size_start, file_name_start) == -1){
        printf("error start packet\n");
        return -1;
    }

    FILE *file;
    //open File
    file = fopen(filename, "w");
    if (file == NULL){
        printf("Cannot open file!");
        return -1;
    }
    // read data packets
    while (TRUE){
        if ((bytes_read = llread(packet)) == -1){
            printf("error llread");
            return -1;
        }
        if (bytes_read){
            if(packet[0] == C_END) break;
            if (receive_data_packet(packet, packet_data_field, seq_no, &total_bytes_read) == -1) return -1;
            if (fwrite(packet_data_field, sizeof(unsigned char), (bytes_read - 4), file) != (bytes_read - 4)){
                printf("error fwrite");
                return -1;  
            }
            seq_no = (seq_no + 1) % 255; 
        }
    }

    // read end control packet
    if(receive_control_packet(packet, &file_size_end,  file_name_end) == -1){
        printf("error end packet");
        return -1;
    }

    //check equality between control packets params
    if(!(file_size_start == file_size_end && !strcmp(file_name_start, file_name_end))){
        printf("Error: control_packets have different params\n");
        return -1;
    }

    //check equality between control packets (file size) and number of bytes actually read
    if(file_size_end != total_bytes_read){
        printf("Error: total_bytes_read is not equal control packet (file size)\n");
        return -1;
    }

    free(packet);
    free(packet_data_field);
    free(file_name_start);
    free(file_name_end);

    fclose(file);

    return 1;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    LinkLayerRole status;
    if (!strcmp(role, "tx")) status = LlTx;
    else if (!strcmp(role, "rx")) status = LlRx;
    connectionParameters.role = status;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    if (llopen(connectionParameters) == -1) return;

    if (connectionParameters.role == LlRx) alread(filename);
    else if (connectionParameters.role == LlTx) alwrite(filename);

    if (llclose(TRUE) == -1){
        printf("error llclose\n");
        return;
    }
}
