/*******************************************************************************
*
*    FILE: vospiGeneric.h
*
*    DESCRIPTION: Generic interface for VoSPI
*
*    AUTHOR: Hart Thomson
*
*    CREATED: 7/10/2015
*
*    HISTORY: 7/10/2015 Hart - Initial Draft
*
**      Copyright 2015 FLIR Systems - Commercial
**      Vision Systems.  All rights reserved.
**
**      Proprietary - PROPRIETARY - FLIR Systems Inc..
**  
**      This document is controlled to FLIR Technology Level 2.
**      The information contained in this document pertains to a
**      dual use product Controlled for export by the Export
**      Administration Regulations (EAR). Diversion contrary to
**      US law is prohibited.  US Department of Commerce
**      authorization is not required prior to export or
**      transfer to foreign persons or parties unless otherwise
**      prohibited.
**
**      Redistribution and use in source and binary forms, with
**      or without modification, are permitted provided that the
**      following conditions are met:
**
**      Redistributions of source code must retain the above
**      copyright notice, this list of conditions and the
**      following disclaimer.
**
**      Redistributions in binary form must reproduce the above
**      copyright notice, this list of conditions and the
**      following disclaimer in the documentation and/or other
**      materials provided with the distribution.
**
**      Neither the name of the FLIR Systems Corporation nor the
**      names of its contributors may be used to endorse or
**      promote products derived from this software without
**      specific prior written permission.
**
**      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
**      CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
**      WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**      WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
**      PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
**      COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY
**      DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
**      CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
**      PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
**      USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
**      CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
**      CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
**      NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
**      USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
**      OF SUCH DAMAGE.
**
*******************************************************************************/



typedef enum LEPTON_CAMERA_VERSION_E_TAG
{
    LEPTON_2_80x60 = 0,
    LEPTON_3_160x120,

    END_LEPTON_CAMERA_VERSION,
} LEPTON_CAMERA_VERSION_E, *LEPTON_CAMERA_VERSION_E_PTR;

/*  Most of the aren't supported, but allows the client to use the enum 
    value directly returned from the camera.
*/
typedef enum VIDEO_OUTPUT_FORMAT_TAG
{
  VIDEO_OUTPUT_FORMAT_RAW8 = 0,          // Not Supported
  VIDEO_OUTPUT_FORMAT_RAW10,             // Not Supported
  VIDEO_OUTPUT_FORMAT_RAW12,             // Not Supported
  VIDEO_OUTPUT_FORMAT_RGB888,            // Supported
  VIDEO_OUTPUT_FORMAT_RGB666,            // Not Supported
  VIDEO_OUTPUT_FORMAT_RGB565,            // Not Supported
  VIDEO_OUTPUT_FORMAT_YUV422_8BIT,       // Not Supported
  VIDEO_OUTPUT_FORMAT_RAW14,             // Supported
  VIDEO_OUTPUT_FORMAT_YUV422_10BIT,      // Not Supported
  VIDEO_OUTPUT_FORMAT_USER_DEFINED,      // Not Supported
  VIDEO_OUTPUT_FORMAT_RAW8_2,            // Not Supported
  VIDEO_OUTPUT_FORMAT_RAW8_3,            // Not Supported
  VIDEO_OUTPUT_FORMAT_RAW8_4,            // Not Supported
  VIDEO_OUTPUT_FORMAT_RAW8_5,            // Not Supported
  VIDEO_OUTPUT_FORMAT_RAW8_6,            // Not Supported
  
  END_VIDEO_OUTPUT_FORMAT

} VIDEO_OUTPUT_FORMAT_E, *VIDEO_OUTPUT_FORMAT_E_PTR;

typedef enum TELEMETRY_ENABLE_STATE_E_TAG
{
    TELEMETRY_DISABLED = 0,
    TELEMETRY_ENABLED,

    END_TELEMETRY_ENABLE_STATE,
} TELEMETRY_ENABLE_STATE_E, *TELEMETRY_ENABLE_STATE_E_PTR;

typedef enum TLINEAR_ENABLE_STATE_E_TAG
{
    TLINEAR_OUTPUT_DISABLED = 0,
    TLINEAR_OUTPUT_ENABLED,

    TLINEAR_END_ENABLE_STATE,

} TLINEAR_ENABLE_STATE_E, *TLINEAR_ENABLE_STATE_E_PTR;

/*
 *  User sets contents of this struct to configure capture
 *
 */
typedef struct VOSPI_HOOK_CONFIG_T_TAG
{
    LEPTON_CAMERA_VERSION_E     lepVersion;
    VIDEO_OUTPUT_FORMAT_E       vidFormat;

    /* Sets to 63 lines in Lepton 2, 61 lines in Lepton 3 per segment */
    TELEMETRY_ENABLE_STATE_E    telemetryEnableState;

    /* Host-side Kelvin*100 output */
    //TLINEAR_ENABLE_STATE_E      tLinearOutputEnableState;

} VOSPI_HOOK_CONFIG_T, *VOSPI_HOOK_CONFIG_T_PTR;

/*
 *  Internal struct used for capture configuration. User need not modify
 *
 */
typedef struct VIDEO_FORMAT_CONFIG_T_TAG
{
    /* Number of bytes per packet, including header */
    uint16_t packetByteSize;

    /* Number of bytes per packet, not including header */
    uint16_t packetVideoPayloadByteSize;

    /* Video Display rows per frame */
    uint16_t rows;

    /* Video Display cols per frame */
    uint16_t cols;

    /* Number of bytes per segment, including headers */
    uint16_t segmentByteSize;

    /* Number of bytes per segment, not including headers */
    uint16_t segmentVideoByteSize;

    /* Total number of bytes in video display */
    uint16_t videoFrameByteSize;

    /* Number of segments we expect to see that comprises a whole frame */
    uint16_t expectedNumSegments;

    /* Number of ms to wait to take us out of SPI de-sync */
    uint16_t spiTimeoutWait;

    /* Number of telemetry lines per segment */
    uint16_t telemetryPacketsPerSegment;

    /* Number of packets per frame or segment */
    uint16_t packetsPerSegment;

} VIDEO_FORMAT_CONFIG_T, *VIDEO_FORMAT_CONFIG_T_PTR;

extern int lep_vospi_init(void);
extern int lep_vospi_configure(VOSPI_HOOK_CONFIG_T config);

