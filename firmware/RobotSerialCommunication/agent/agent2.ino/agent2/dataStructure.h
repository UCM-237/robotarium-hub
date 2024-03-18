
const int MAXDATASIZE=255;
const int HEADER_LEN (sizeof(unsigned short)*3);

struct appdata {
  unsigned short InitFlag;
  unsigned short id;
  unsigned short op; //codigo de operacion
  unsigned short len;                       /* longitud de datos */
  unsigned char data [MAXDATASIZE-HEADER_LEN];//datos
};