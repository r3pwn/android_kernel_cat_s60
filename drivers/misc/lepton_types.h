#define FRAME_NUM_PIXEL (82*63*2)


typedef enum LEP_RESULT_E_TAG
{
    LEP_OK = 0,
    LEP_ERROR,

    LEP_END_RESULT,
} LEP_RESULT, *LEP_RESULT_PTR;

typedef enum LEP_FRAME_STATUS_E_TAG
{
    LEP_FRAME_NOT_READY = 0,
    LEP_FRAME_READY,

    LEP_END_FRAME_STATUS,
}LEP_FRAME_STATUS_E, *LEP_FRAME_STATUS_E_PTR;

typedef struct LEP_FRAME_DATA_T_TAG
{
    LEP_RESULT result;
    uint16_t pixels[FRAME_NUM_PIXEL];

} LEP_FRAME_DATA_T, *LEP_FRAME_DATA_T_PTR;


typedef struct LEP_DEBUG_INFO_T_TAG
{
    uint64_t timeSinceLastFrame;
    uint32_t framesPerSecond;
    uint32_t gpioSignalsSinceBoot;

} LEP_DEBUG_INFO_T, *LEP_DEBUG_INFO_T_PTR;




#define MY_MACIG 'G'

#define GET_FRAME_READY_STATUS_IOCTL _IOR(MY_MACIG, 1, LEP_FRAME_STATUS_E*)
#define READ_FRAME_IOCTL _IOR(MY_MACIG, 2, LEP_FRAME_DATA_T*)
#define START_SPI_IOCTL _IOR(MY_MACIG, 3, LEP_RESULT*)
#define STOP_SPI_IOCTL _IOR(MY_MACIG, 4, LEP_RESULT*)
#define GET_DEBUG_INFO_IOCTL _IOR(MY_MACIG, 5, LEP_DEBUG_INFO_T*)

//extern int spi_read_bytes(u8* rx, int nBytes);



